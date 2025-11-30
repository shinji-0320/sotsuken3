// scratch/simple-flooding.cc (ns-3.44)
// - グリッド分割→SELECT拡散→20msでゲートウェイ確定→1.0sに送信ノードがDATA開始
// - Forward 前ジッタ 0〜6ms
// - 送信ノード以外は指定範囲・指定個数でランダム配置（--nodeCount, --nodeXmin..max, --nodeYmin..max）
// - GW再選出: 旧グリッド離脱を CourseChange + 50msポーリングで即検知→旧g_idへRESELECT送出→旧グリッド内で再選出
// - GATE/BID: 同グリッドGWのGATEがBID後に未返答なら再選出（BID→結果待ち→未受信ならSELECT発火）

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"
#include <set>
#include <map>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>

using namespace ns3;

// ======================= アプリ層ヘッダ ===========================
// GeoGRID 用の自前ヘッダ。メッセージ種別と選出情報/位置情報を24B固定で運ぶ。
class GeoHeader : public Header
{
public:
  enum MsgType : uint8_t { DATA=1, SELECT=2, RESELECT=3, GATE=4, BID=5 };

  GeoHeader() = default;

  // 各種 setter/getter
  void SetType (MsgType t) { m_type = t; }
  MsgType GetType () const { return m_type; }

  void SetSrc (Ipv4Address a) { m_src = a; }
  Ipv4Address GetSrc () const { return m_src; }

  void SetSeq (uint32_t s) { m_seq = s; }
  uint32_t GetSeq () const { return m_seq; }

  void SetGridId (uint32_t gid) { m_g_id = gid; }
  uint32_t GetGridId () const { return m_g_id; }

  void SetRound (uint32_t r) { m_round = r; }
  uint32_t GetRound () const { return m_round; }

  void SetDistance (float d) { m_distance = d; }
  float GetDistance () const { return m_distance; }

  // ヘッダ登録
  static TypeId GetTypeId ()
  {
    static TypeId tid = TypeId ("ns3::GeoHeader")
      .SetParent<Header> ()
      .AddConstructor<GeoHeader> ();
    return tid;
  }
  virtual TypeId GetInstanceTypeId () const override { return GetTypeId(); }

  // ログ印字用
  virtual void Print (std::ostream &os) const override
  {
    os << "type=" << unsigned(m_type) << " src=" << m_src
       << " seq=" << m_seq << " g_id=" << m_g_id
       << " round=" << m_round << " dist=" << m_distance;
  }

  // 固定長 24B で直列化
  virtual uint32_t GetSerializedSize () const override { return 24; }

  // バイト列→ヘッダ
  virtual void Serialize (Buffer::Iterator start) const override
  {
    start.WriteU8 (m_type);
    start.WriteU8 (0); start.WriteU8 (0); start.WriteU8 (0); // pad
    start.WriteHtonU32 (m_src.Get());
    start.WriteHtonU32 (m_seq);
    start.WriteHtonU32 (m_g_id);
    start.WriteHtonU32 (m_round);
    union { float f; uint32_t u; } conv; conv.f = m_distance;
    start.WriteHtonU32 (conv.u);
  }

  // ヘッダ→バイト列
  virtual uint32_t Deserialize (Buffer::Iterator start) override
  {
    m_type = static_cast<MsgType>(start.ReadU8 ());
    start.ReadU8 (); start.ReadU8 (); start.ReadU8 (); // pad
    m_src = Ipv4Address (start.ReadNtohU32 ());
    m_seq = start.ReadNtohU32 ();
    m_g_id = start.ReadNtohU32 ();
    m_round = start.ReadNtohU32 ();
    union { float f; uint32_t u; } conv; conv.u = start.ReadNtohU32 ();
    m_distance = conv.f;
    return GetSerializedSize ();
  }

private:
  // ヘッダ内容
  MsgType     m_type{DATA};
  Ipv4Address m_src{"0.0.0.0"};
  uint32_t    m_seq{0};
  uint32_t    m_g_id{0};
  uint32_t    m_round{0};
  float       m_distance{0.0f};
};

// ======================= アプリ本体 ===========================
// GeoGRID + Flooding のアプリ。通信以外の選出・管理ロジックもここで実装。
class SimpleFloodingApp : public Application
{
public:
  static TypeId GetTypeId ()
  {
    // アトリビュート（CLIやHelperで設定できるパラメータ）
    static TypeId tid = TypeId ("ns3::SimpleFloodingApp")
      .SetParent<Application> ()
      .AddConstructor<SimpleFloodingApp> ()
      // 送受信UDPポート
      .AddAttribute ("BroadcastPort", "UDP port to broadcast on.",
                     UintegerValue (9999),
                     MakeUintegerAccessor (&SimpleFloodingApp::m_port),
                     MakeUintegerChecker<uint16_t> ())
      // 送信ノードの送信周期
      .AddAttribute ("SendInterval", "Interval between sender transmissions.",
                     TimeValue (Seconds (1.0)),
                     MakeTimeAccessor (&SimpleFloodingApp::m_sendInterval),
                     MakeTimeChecker ())
      // 送信ノードフラグ（=データ起点。GW選出には参加しない）
      .AddAttribute ("IsSender", "If true, this node originates DATA periodically.",
                     BooleanValue (false),
                     MakeBooleanAccessor (&SimpleFloodingApp::m_isSender),
                     MakeBooleanChecker ())
      // 送信ノードの通し番号（S_id用に保持）
      .AddAttribute ("SenderIndex", "1-based index in senders; 0 otherwise.",
                     UintegerValue (0),
                     MakeUintegerAccessor (&SimpleFloodingApp::m_senderIndex),
                     MakeUintegerChecker<uint32_t> ())
      // Geocast region の矩形（複数指定時は全体BBOXとして利用）
      .AddAttribute ("grXmin", "Geocast regions bbox xmin.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grXmin),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("grYmin", "Geocast regions bbox ymin.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grYmin),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("grXmax", "Geocast regions bbox xmax.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grXmax),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("grYmax", "Geocast regions bbox ymax.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grYmax),
                     MakeDoubleChecker<double> ())
      // FWDゾーンの基準となる送信ノードID（1始まり）
      .AddAttribute ("S_id", "Source id (1-based) used to anchor the forwarding zone.",
                     UintegerValue (1),
                     MakeUintegerAccessor (&SimpleFloodingApp::m_s_id),
                     MakeUintegerChecker<uint32_t> ());
    return tid;
  }

  SimpleFloodingApp () = default;
  virtual ~SimpleFloodingApp () = default;

  // 複数Geocast regionを扱うための矩形構造体
  struct GeoRegionRect
  {
    double xmin;
    double ymin;
    double xmax;
    double ymax;
  };

  // main() からシミュレーション開始前に呼び出し、
  // 全てのGeocast regionを共有状態に登録する
  static void SetGeocastRegions (const std::vector<GeoRegionRect>& regs);

  // Add-Forwarding zone 用の NCN を設定（全ノードで共有）
  static void SetNCN (uint32_t n);

  // BCL: ゲートウェイノードの転送制限数（0=制限なし, 全ノードで共有）
  static void SetBCL (uint32_t bcl);

  // --- グリッド定義（d = 一辺, s_grid_d に格納） ---
  // 領域を正方セルに分割し、各セルの境界を静的配列に保存する。
  static void DefineGrid (double originX, double originY, double d, uint32_t nx, uint32_t ny)
  {
    s_gridOriginX = originX; s_gridOriginY = originY;
    s_grid_d = d; s_gridNx = nx; s_gridNy = ny;
    s_GXmin.assign (nx * ny, 0.0); s_GXmax.assign (nx * ny, 0.0);
    s_GYmin.assign (nx * ny, 0.0); s_GYmax.assign (nx * ny, 0.0);
    for (uint32_t r = 0; r < ny; ++r) {
      for (uint32_t c = 0; c < nx; ++c) {
        uint32_t idx = r * nx + c;
        double x0 = originX + c * d;
        double y0 = originY + r * d;
        s_GXmin[idx] = x0;        s_GXmax[idx] = x0 + d;
        s_GYmin[idx] = y0;        s_GYmax[idx] = y0 + d;
      }
    }
    s_gridDefined = true;
    // Geocast region とグリッド対応は再計算させる
    s_geoGridComputed = false;
    NS_LOG_UNCOND ("GRID-DEF: origin=(" << originX << "," << originY << ") d=" << d
                    << " Nx=" << nx << " Ny=" << ny << " total=" << (nx*ny));
  }

private:
  // ---------- Forwarding zone 算出 ----------
  // Geocast region 群と送信ノード(S_id)の位置から一度だけ矩形を作る。
  // 「送信ノード＋すべてのGeocast region を含む最小矩形」を FWDゾーンとする。
  static void MaybeComputeZone ()
  {
    if (s_zoneComputed) return;
    if (s_s_id == 0) return;
    uint32_t idx = s_s_id - 1;
    if (idx >= s_senders.size ()) return;
    Ptr<Node> anchor = s_senders[idx]; if (!anchor) return;
    Ptr<MobilityModel> mm = anchor->GetObject<MobilityModel>(); if (!mm) return;
    Vector spos = mm->GetPosition ();
    double Xs = spos.x, Ys = spos.y;

    // 送信ノード座標を初期値にして、全Geocast region とのBBOXをとる
    double xmin = Xs;
    double ymin = Ys;
    double xmax = Xs;
    double ymax = Ys;

    if (!s_geocastRegions.empty())
    {
      for (const auto &reg : s_geocastRegions) {
        xmin = std::min (xmin, reg.xmin);
        ymin = std::min (ymin, reg.ymin);
        xmax = std::max (xmax, reg.xmax);
        ymax = std::max (ymax, reg.ymax);
      }
    }
    else
    {
      // Geocast region が一つ分だけ CLI で与えられているケースなど
      xmin = std::min (xmin, s_grXmin);
      ymin = std::min (ymin, s_grYmin);
      xmax = std::max (xmax, s_grXmax);
      ymax = std::max (ymax, s_grYmax);
    }

    s_rXmin = xmin;
    s_rXmax = xmax;
    s_rYmin = ymin;
    s_rYmax = ymax;
    s_zoneComputed = true;

    NS_LOG_UNCOND ("FWD-ZONE(app): [" << s_rXmin << "," << s_rYmin << "] - ["
                    << s_rXmax << "," << s_rYmax << "]  by s-id=" << s_s_id
                    << "  (GR-ALL-BBOX=[" << s_grXmin << "," << s_grYmin
                    << "]-[" << s_grXmax << "," << s_grYmax << "])");
  }

  // Add-Forwarding zone を一度だけ計算（送信ノード1とGeocast regionの象限＋NCNに基づく）
  static void MaybeComputeAddZone ()
  {
    // NCN=0 のときは Add-Forwarding zone は使わない
    if (s_NCN == 0) {
      s_addFwdGids.clear();
      s_addZoneComputed = true;
      return;
    }

    if (s_addZoneComputed) return;
    if (!s_gridDefined || s_gridNx == 0 || s_gridNy == 0) return;
    if (s_senders.empty() || !s_senders[0]) return; // 送信ノード1がまだ登録されていない

    Ptr<Node> src1 = s_senders[0];
    Ptr<MobilityModel> mm = src1->GetObject<MobilityModel>(); if (!mm) return;
    Vector spos = mm->GetPosition();
    double sx = spos.x;
    double sy = spos.y;

    uint32_t sgid = LocateGid (spos);
    if (sgid == 0) return;

    // Geocast region の(min,max)が送信ノード1から見てどの象限にあるかを集約
    std::set<uint32_t> regionQuadrants;
    if (!s_geocastRegions.empty()) {
      for (const auto &reg : s_geocastRegions) {
        uint32_t qmin = ClassifyQuadrant (reg.xmin, reg.ymin, sx, sy);
        uint32_t qmax = ClassifyQuadrant (reg.xmax, reg.ymax, sx, sy);
        if (qmin != 0) regionQuadrants.insert (qmin);
        if (qmax != 0) regionQuadrants.insert (qmax);
      }
    } else {
      if (s_grXmax > s_grXmin && s_grYmax > s_grYmin) {
        uint32_t qmin = ClassifyQuadrant (s_grXmin, s_grYmin, sx, sy);
        uint32_t qmax = ClassifyQuadrant (s_grXmax, s_grYmax, sx, sy);
        if (qmin != 0) regionQuadrants.insert (qmin);
        if (qmax != 0) regionQuadrants.insert (qmax);
      }
    }

    // Geocast region 全体の中心（BBOXの中心）を距離計算の基準にする
    double gcx, gcy;
    if (s_grXmax > s_grXmin && s_grYmax > s_grYmin) {
      gcx = 0.5 * (s_grXmin + s_grXmax);
      gcy = 0.5 * (s_grYmin + s_grYmax);
    } else {
      // region がない場合は送信ノード1の位置を基準にしておく
      gcx = sx;
      gcy = sy;
    }

    // パターン分類
    uint32_t pattern = 3;
    if (regionQuadrants.size () == 1) {
      pattern = 1;
    } else if (regionQuadrants.size () == 2) {
      bool has1 = regionQuadrants.count (1) != 0;
      bool has2 = regionQuadrants.count (2) != 0;
      bool has3 = regionQuadrants.count (3) != 0;
      bool has4 = regionQuadrants.count (4) != 0;
      // (1,3) または (2,4) 以外ならパターン2
      if (!((has1 && has3) || (has2 && has4))) {
        pattern = 2;
      }
    }

    struct Cand
    {
      uint32_t gid;
      uint32_t quadrant;
      double   dist2;
    };

    std::vector<Cand> allCands;

    auto pushCand = [&](int gidInt)
    {
      if (gidInt < 1 || gidInt > static_cast<int>(s_gridNx * s_gridNy)) return;
      uint32_t gid = static_cast<uint32_t>(gidInt);
      uint32_t idx = gid - 1;
      double cx = 0.5 * (s_GXmin[idx] + s_GXmax[idx]);
      double cy = 0.5 * (s_GYmin[idx] + s_GYmax[idx]);
      uint32_t q = ClassifyQuadrant (cx, cy, sx, sy);
      double dx = cx - gcx;
      double dy = cy - gcy;
      double d2 = dx * dx + dy * dy;
      Cand c { gid, q, d2 };
      allCands.push_back (c);
    };

    int sg = static_cast<int>(sgid);
    // Add-Forwarding zone の候補グリッド（8近傍）
    pushCand (sg - (static_cast<int>(s_gridNx) + 1));
    pushCand (sg - static_cast<int>(s_gridNx));
    pushCand (sg - (static_cast<int>(s_gridNx) - 1));
    pushCand (sg - 1);
    pushCand (sg + 1);
    pushCand (sg + (static_cast<int>(s_gridNx) - 1));
    pushCand (sg + static_cast<int>(s_gridNx));
    pushCand (sg + (static_cast<int>(s_gridNx) + 1));

    if (allCands.empty()) {
      s_addFwdGids.clear();
      s_addZoneComputed = true;
      return;
    }

    // region の象限に入っている候補かどうか
    auto inRegionQuadrant = [&](const Cand& c)->bool {
      if (regionQuadrants.empty()) return true;   // 象限情報がなければ制限しない
      if (c.quadrant == 0) return false;          // 軸上は除外
      return (regionQuadrants.count (c.quadrant) != 0);
    };

    std::vector<Cand> filtered;

    if (pattern == 1) {
      if (s_NCN >= 1 && s_NCN <= 3) {
        for (const auto& c : allCands) {
          if (inRegionQuadrant (c)) filtered.push_back (c);
        }
      } else {
        filtered = allCands; // 4〜8 は象限による制限なし
      }
    } else if (pattern == 2) {
      if (s_NCN >= 1 && s_NCN <= 5) {
        for (const auto& c : allCands) {
          if (inRegionQuadrant (c)) filtered.push_back (c);
        }
      } else {
        filtered = allCands; // 6〜8 は象限による制限なし
      }
    } else {
      // パターン3は常に候補全体から選ぶ
      filtered = allCands;
    }

    if (filtered.empty()) {
      s_addFwdGids.clear();
      s_addZoneComputed = true;
      return;
    }

    // Geocast region 中心との距離 → 近い順（同距離ならg_id小さい順）
    std::sort (filtered.begin(), filtered.end(),
      [](const Cand& a, const Cand& b)
      {
        if (std::abs (a.dist2 - b.dist2) > 1e-9) return a.dist2 < b.dist2;
        return a.gid < b.gid;
      });

    uint32_t numToAdd = std::min (s_NCN, static_cast<uint32_t>(filtered.size()));
    s_addFwdGids.clear();
    for (uint32_t i = 0; i < numToAdd; ++i) {
      s_addFwdGids.insert (filtered[i].gid);
    }

    s_addZoneComputed = true;

    // Add-Forwarding zone に選ばれたグリッドID一覧もログ出力
    std::ostringstream oss;
    oss << "ADD-FWD-ZONE(app): NCN=" << s_NCN
        << " pattern=" << pattern
        << " sgid=" << sgid
        << " selected=" << s_addFwdGids.size()
        << " gids=[";
    bool first = true;
    for (auto gid : s_addFwdGids) {
      if (!first) oss << ",";
      first = false;
      oss << gid;
    }
    oss << "]";
    NS_LOG_UNCOND (oss.str());
  }

  // ---------- 位置→g_id（1始まり） ----------
  // 座標から所属グリッドIDを取得（外れ値は端にクリップ）
  static uint32_t LocateGid (const Vector& pos)
  {
    if (!s_gridDefined || s_grid_d <= 0.0 || s_gridNx == 0 || s_gridNy == 0) return 0;
    int c = static_cast<int>(std::floor((pos.x - s_gridOriginX) / s_grid_d));
    int r = static_cast<int>(std::floor((pos.y - s_gridOriginY) / s_grid_d));
    if (c < 0) c = 0;
    if (c >= static_cast<int>(s_gridNx)) c = s_gridNx - 1;
    if (r < 0) r = 0;
    if (r >= static_cast<int>(s_gridNy)) r = s_gridNy - 1;
    return static_cast<uint32_t>(r) * s_gridNx + static_cast<uint32_t>(c) + 1;
  }

  // 送信ノード座標を原点としたときの象限分類（軸上は0）
  static uint32_t ClassifyQuadrant (double x, double y, double ox, double oy)
  {
    double dx = x - ox;
    double dy = y - oy;
    if (dx > 0.0 && dy > 0.0) return 1;
    if (dx < 0.0 && dy > 0.0) return 2;
    if (dx < 0.0 && dy < 0.0) return 3;
    if (dx > 0.0 && dy < 0.0) return 4;
    return 0; // x==0 または y==0 の場合は象限0として扱う
  }

  // Geocast region とグリッドIDの対応（region内/隣接）を一度だけ計算
  static void MaybeComputeGeoGridSets ();
  // 7-4: 自グリッドがGeocast region内かどうか
  bool IsInsideGeocastRegionGrid () const;
  // 7-5: 自グリッドがGeocast region隣接グリッドかどうか
  bool IsAdjacentToGeocastRegionGrid () const;

  // 7-4 用ヘルパ: 複数Geocast regionの「内部」判定ラッパ
  bool IsInsideAnyGeocastRegion () const { return IsInsideGeocastRegionGrid (); }

  // 7-5 用ヘルパ: 複数Geocast regionの「隣接」判定ラッパ
  bool IsNeighborOfAnyGeocastRegion () const { return IsAdjacentToGeocastRegionGrid (); }

  // ★ 自グリッドが「どのGeocast regionの隣接グリッド集合」に属しているかインデックス一覧を返す
  std::vector<uint32_t> GetAdjacentRegionIndicesForMyGrid () const;

  // ---------- 自グリッド中心までの距離 ----------
  // SELECT 比較用（中心に近いほど良い）
  double DistToGridCenter () const
  {
    double cx = 0.5 * (m_gxmin + m_gxmax);
    double cy = 0.5 * (m_gymin + m_gymax);
    Ptr<MobilityModel> mm = GetNode ()->GetObject<MobilityModel>();
    Vector p = mm->GetPosition ();
    double dx = p.x - cx, dy = p.y - cy;
    return std::sqrt (dx*dx + dy*dy);
  }

  // ====== グリッド越え検知 ======
  // 位置更新トレースで即時チェック
  void OnCourseChange (Ptr<const MobilityModel> /*mm*/)
  {
    CheckGrid();
  }
  // 50ms 間隔の補助ポーリング
  void GridCheckTick ()
  {
    CheckGrid();
    m_gridCheckEvent = Simulator::Schedule (m_gridCheckInterval, &SimpleFloodingApp::GridCheckTick, this);
  }
  // g_id 変化の本体。GWのまま出たら旧グリッドに RESELECT を通知。
  void CheckGrid ()
  {
    if (!s_gridDefined) return;
    Ptr<MobilityModel> mm = GetNode ()->GetObject<MobilityModel>(); if (!mm) return;
    Vector pos = mm->GetPosition ();
    uint32_t newG = LocateGid (pos);
    if (newG == m_g_id) return;

    uint32_t oldG = m_g_id;
    m_g_id = newG;
    if (m_g_id >= 1) {
      uint32_t idx0 = m_g_id - 1;
      m_gxmin = s_GXmin[idx0]; m_gymin = s_GYmin[idx0];
      m_gxmax = s_GXmax[idx0]; m_gymax = s_GYmax[idx0];
    }

    // GWのまま出てしまったら旧グリッド側に再選出を促す
    if (m_isGateway) {
      SendReselect (oldG);
      m_isGateway = false;
      EnsureGateBeaconing();
    }

    NS_LOG_UNCOND ("GRID-LEAVE: " << m_myAddress
                    << " old_g_id=" << oldG << " -> new_g_id=" << m_g_id);
  }
  // 旧グリッドへ RESELECT を通知（旧グリッド内のノードがSELECTを再実施）
  void SendReselect (uint32_t oldGid)
  {
    GeoHeader hdr;
    hdr.SetType (GeoHeader::RESELECT);
    hdr.SetSrc  (m_myAddress);
    hdr.SetSeq  (m_selectSeq++);
    hdr.SetGridId (oldGid);

    Ptr<Packet> p = Create<Packet> (8);
    p->AddHeader (hdr);
    InetSocketAddress bcast (Ipv4Address ("255.255.255.255"), m_port);
    m_socket->SendTo (p, 0, bcast);

    NS_LOG_UNCOND ("RESELECT: " << m_myAddress << " old_g_id=" << oldGid);
  }

  // ====== GATE/BID（入れ替わり抑制＆BID未応答なら再選出） ======
  // GW時のみ GATEを定期的に出し、非GWなら停止
  void EnsureGateBeaconing ()
  {
    if (m_isGateway) {
      if (!m_gateEvent.IsPending()) {
        m_gateEvent = Simulator::Schedule (m_gateInterval, &SimpleFloodingApp::SendGate, this);
      }
    } else {
      if (m_gateEvent.IsPending()) Simulator::Cancel (m_gateEvent);
    }
  }
  // GATE ブロードキャスト（送信ノードは絶対に出さない）
  void SendGate ()
  {
    if (!m_isGateway || m_g_id == 0 || m_isSender) return;
    GeoHeader hdr;
    hdr.SetType (GeoHeader::GATE);
    hdr.SetSrc  (m_myAddress);
    hdr.SetSeq  (m_selectSeq++);
    hdr.SetGridId (m_g_id);

    Ptr<Packet> p = Create<Packet>(8);
    p->AddHeader(hdr);
    InetSocketAddress bcast (Ipv4Address ("255.255.255.255"), m_port);
    m_socket->SendTo (p, 0, bcast);

    // 次のGATEをスケジュールし、最終受信時刻を更新
    m_gateEvent = Simulator::Schedule (m_gateInterval, &SimpleFloodingApp::SendGate, this);
    m_lastGateRx = Simulator::Now();
  }
  // GATE が長く見えないときに非GWが発行する「生存確認/BID」
  void SendBid ()
  {
    if (m_isGateway || m_g_id == 0) return;
    GeoHeader hdr;
    hdr.SetType (GeoHeader::BID);
    hdr.SetSrc  (m_myAddress);
    hdr.SetSeq  (m_selectSeq++);
    hdr.SetGridId (m_g_id);

    Ptr<Packet> p = Create<Packet>(8);
    p->AddHeader(hdr);
    InetSocketAddress bcast (Ipv4Address ("255.255.255.255"), m_port);
    m_socket->SendTo (p, 0, bcast);

    m_lastBidTx = Simulator::Now();  // BID送信時刻を記録
    NS_LOG_UNCOND ("BID: " << m_myAddress << " g_id=" << m_g_id);
  }
  // BID を出した後の応答(GATE)待ち。来なければ SELECT を再始動。
  void OnBidResultTimeout ()
  {
    // BID後に同グリッドのGATEが来なかったら再選出
    if (m_isGateway || m_g_id == 0) return;
    if (m_lastGateRx >= m_lastBidTx) {
      // BID以後にGATE受信済み → 何もしない
      return;
    }
    m_selectRound++;
    static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    const double j = uv->GetValue(0.0, 0.008);
    Simulator::Schedule(Seconds(j), &SimpleFloodingApp::SendSelect, this);
    NS_LOG_UNCOND ("BID-TIMEOUT: " << m_myAddress << " g_id=" << m_g_id
                    << " -> re-run SELECT round=" << m_selectRound);
  }
  // GATE の監視を周期的に行い、タイムアウトでBIDを送信
  void WatchdogTick ()
  {
    // GWでない & 送信ノードでない & しばらくGATEが見えない → BID
    if (!m_isGateway && !m_isSender) {
      if (Simulator::Now() - m_lastGateRx > m_gateTimeout) {
        SendBid();
        if (m_bidAckEvent.IsPending()) Simulator::Cancel (m_bidAckEvent);
        // 結果待ちタイマー（BID後にGATEが返らない場合は再選出）
        m_bidAckEvent = Simulator::Schedule (m_bidResponseWait,
                          &SimpleFloodingApp::OnBidResultTimeout, this);
      }
    }
    m_gateWatchdogEvent = Simulator::Schedule (m_gateInterval, &SimpleFloodingApp::WatchdogTick, this);
  }

  // ---------- 起動処理 ----------
  // ソケット初期化→FWDゾーン算出→g_id把握→
  // 移動トレース接続→監視タイマ開始→非送信のみSELECT初回→送信は即DATA開始
  virtual void StartApplication () override
  {
    m_socket = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
    InetSocketAddress local (Ipv4Address::GetAny (), m_port);
    m_socket->Bind (local);
    m_socket->SetRecvCallback (MakeCallback (&SimpleFloodingApp::HandleRead, this));
    m_socket->SetAllowBroadcast (true);

    Ptr<Ipv4> ipv4 = GetNode ()->GetObject<Ipv4>();
    Ipv4InterfaceAddress if0 = ipv4->GetAddress (1, 0);
    m_myAddress = if0.GetLocal ();

    // Sender 登録（FWDゾーンの基準座標を引くため）
    if (m_isSender && m_senderIndex > 0) {
      if (s_senders.size () < m_senderIndex) s_senders.resize (m_senderIndex);
      s_senders[m_senderIndex - 1] = GetNode ();
    }

    // FWDゾーンを算出できるならする（初回のみ）
    MaybeComputeZone ();

    // 自分の g_id と境界を把握
    Ptr<MobilityModel> mm = GetNode ()->GetObject<MobilityModel>();
    m_mobility = mm;
    if (mm && s_gridDefined) {
      Vector pos = mm->GetPosition ();
      m_g_id = LocateGid (pos);
      if (m_g_id >= 1) {
        uint32_t idx0 = m_g_id - 1;
        m_gxmin = s_GXmin[idx0]; m_gymin = s_GYmin[idx0];
        m_gxmax = s_GXmax[idx0]; m_gymax = s_GYmax[idx0];
        NS_LOG_UNCOND ("GRID-MAP: " << m_myAddress << " in g_id=" << m_g_id
                        << " GX[" << m_g_id << "]min=" << m_gxmin
                        << " GY[" << m_g_id << "]min=" << m_gymin
                        << " GX[" << m_g_id << "]max=" << m_gxmax
                        << " GY[" << m_g_id << "]max=" << m_gymax);
      }
    }

    // CourseChange フック + グリッドポーリング開始
    if (m_mobility) {
      m_mobility->TraceConnectWithoutContext ("CourseChange",
        MakeCallback (&SimpleFloodingApp::OnCourseChange, this));
    }
    m_gridCheckEvent = Simulator::Schedule (m_gridCheckInterval,
      &SimpleFloodingApp::GridCheckTick, this);

    // GATE監視開始
    m_lastGateRx = Simulator::Now();
    m_gateWatchdogEvent = Simulator::Schedule (m_gateInterval,
      &SimpleFloodingApp::WatchdogTick, this);

    // --- 2-1: SELECT（送信ノード以外）→ 起動直後 0〜8ms で送出 ---
    if (!m_isSender) {
      static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
      double j = uv->GetValue (0.0, 0.008);
      Simulator::Schedule (Seconds (j), &SimpleFloodingApp::SendSelect, this);
    }

    // 送信ノードは小さなジッタを入れて送信開始（衝突回避）
    if (m_isSender) {
      static Ptr<UniformRandomVariable> suv = CreateObject<UniformRandomVariable>();
      double startJitter = suv->GetValue(0.0, 0.0001) + 0.00002 * std::max<int>(0, (int)m_senderIndex - 1);
      Simulator::Schedule(Seconds(startJitter), &SimpleFloodingApp::SendOne, this);
    }
  }

  // 停止時：すべてのスケジュールをキャンセル
  virtual void StopApplication () override
  {
    if (m_socket) { m_socket->Close (); m_socket = nullptr; }
    Simulator::Cancel (m_sendEvent);
    for (auto &kv : m_selFinalize) { Simulator::Cancel (kv.second); }

    if (m_gateEvent.IsPending()) Simulator::Cancel (m_gateEvent);
    if (m_gateWatchdogEvent.IsPending()) Simulator::Cancel (m_gateWatchdogEvent);
    if (m_bidAckEvent.IsPending()) Simulator::Cancel (m_bidAckEvent);
    if (m_gridCheckEvent.IsPending()) Simulator::Cancel (m_gridCheckEvent);
  }

  // ---------- 2-1: SELECT 送信 ----------
  // 非送信ノードが自グリッドの中心距離を載せて発行
  void SendSelect ()
  {
    if (m_g_id == 0) return;

    GeoHeader hdr;
    hdr.SetType (GeoHeader::SELECT);
    hdr.SetSrc (m_myAddress);
    hdr.SetSeq (m_selectSeq++);
    hdr.SetGridId (m_g_id);
    hdr.SetRound (m_selectRound);
    hdr.SetDistance (static_cast<float>(DistToGridCenter ()));

    Ptr<Packet> p = Create<Packet> (16);
    p->AddHeader (hdr);
    InetSocketAddress bcast (Ipv4Address ("255.255.255.255"), m_port);
    m_socket->SendTo (p, 0, bcast);

    // 自分自身の候補にも反映
    ConsiderCandidate (m_g_id, m_selectRound, m_myAddress, hdr.GetDistance ());

    NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                    << m_myAddress << " SELECT g_id=" << m_g_id
                    << " dist=" << hdr.GetDistance () << " round=" << m_selectRound);
  }

  // ---------- 2-2: 候補の取り込み & 確定 ----------
  struct SelState {
    bool        initialized{false};
    double      bestDist{0};
    Ipv4Address bestAddr{"0.0.0.0"};
  };

  // g_id と round を 64bit キーにパック
  static uint64_t KeySel (uint32_t gid, uint32_t round)
  {
    return (static_cast<uint64_t>(gid) << 32) | static_cast<uint64_t>(round);
  }

  // 最小距離（同値はIP小）で暫定ベストを更新。20ms後にFinalize。
  void ConsiderCandidate (uint32_t gid, uint32_t round, Ipv4Address cand, double dist)
  {
    uint64_t key = KeySel (gid, round);
    SelState &s = m_sel[key];
    if (!s.initialized) {
      s.initialized = true; s.bestDist = dist; s.bestAddr = cand;
    } else {
      if (dist < s.bestDist - 1e-9) {
        s.bestDist = dist; s.bestAddr = cand;
      } else if (std::abs(dist - s.bestDist) <= 1e-9) {
        if (cand.Get () < s.bestAddr.Get ()) s.bestAddr = cand;
      }
    }
    if (m_selFinalize.find(key) == m_selFinalize.end()) {
      EventId ev = Simulator::Schedule (MilliSeconds (20), &SimpleFloodingApp::FinalizeSelect, this, gid, round);
      m_selFinalize[key] = ev;
    }
  }

  // 20ms の収束後にGWを更新し、GATEのON/OFFを切替
  void FinalizeSelect (uint32_t gid, uint32_t round)
  {
    uint64_t key = KeySel (gid, round);
    auto it = m_sel.find (key);
    if (it == m_sel.end() || !it->second.initialized) return;

    Ipv4Address winner = it->second.bestAddr;
    bool wasGateway = m_isGateway;
    m_isGateway = (winner == m_myAddress);
    if (m_isGateway != wasGateway) {
      NS_LOG_UNCOND ("GW-ELECT: g_id=" << gid << " round=" << round
                      << " gateway=" << winner << " me=" << m_myAddress
                      << (m_isGateway ? " (I AM GW)" : " (not me)"));
    } else {
      NS_LOG_UNCOND ("GW-ELECT: g_id=" << gid << " round=" << round
                      << " gateway=" << winner << " me=" << m_myAddress
                      << (m_isGateway ? " (still GW)" : " (still not)"));
    }
    EnsureGateBeaconing();
    m_selFinalize.erase (key);
  }

  // ---------- DATA 送信 ----------
  // 送信ノードの定期DATA送出
  void SendOne ()
  {
    GeoHeader hdr;
    hdr.SetType (GeoHeader::DATA);
    hdr.SetSrc (m_myAddress);
    hdr.SetSeq (m_nextSeq++);

    Ptr<Packet> p = Create<Packet> (200);
    p->AddHeader (hdr);
    InetSocketAddress bcast (Ipv4Address ("255.255.255.255"), m_port);
    m_socket->SendTo (p, 0, bcast);

    NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                    << m_myAddress << " TX(DATA) seq=" << hdr.GetSeq ());
    
    static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    double perTxJitter = uv->GetValue(0.0, 0.0006);
    m_sendEvent = Simulator::Schedule (m_sendInterval + Seconds(perTxJitter), &SimpleFloodingApp::SendOne, this);
  }

  // GWがDATAを前方へ再送（0〜6msジッタ）
  void Forward (Ptr<Packet> p, const GeoHeader& hdr)
  {
    // ★ このGWが行ったDATA前方転送回数をカウント（BCL 判定用）
    m_forwardCount++;

    InetSocketAddress bcast (Ipv4Address ("255.255.255.255"), m_port);
    m_socket->SendTo (p, 0, bcast);
    NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                    << m_myAddress << " FWD(DATA) src=" << hdr.GetSrc ()
                    << " seq=" << hdr.GetSeq ()
                    << " fwdCount=" << m_forwardCount);
  }

  // ---------- FWDゾーン内判定 ----------
  // Forwarding zone は常に利用する
  bool IsInsideFwdZone () const
  {
    if (!s_zoneComputed) MaybeComputeZone ();
    if (!s_zoneComputed) return true; // 保守的に許可
    Ptr<MobilityModel> mm = GetNode ()->GetObject<MobilityModel>(); if (!mm) return true;
    Vector pos = mm->GetPosition ();
    return (pos.x >= s_rXmin && pos.x <= s_rXmax &&
            pos.y >= s_rYmin && pos.y <= s_rYmax);
  }

  // Add-Forwarding zone 内判定（g_id ベース）
  bool IsInsideAddZone () const
  {
    if (s_NCN == 0) return false;
    if (!s_addZoneComputed) {
      MaybeComputeAddZone ();
    }
    if (!s_addZoneComputed || s_addFwdGids.empty()) return false;
    if (!s_gridDefined) return false;
    Ptr<MobilityModel> mm = GetNode ()->GetObject<MobilityModel>(); if (!mm) return false;
    Vector pos = mm->GetPosition ();
    uint32_t gid = LocateGid (pos);
    if (gid == 0) return false;
    return (s_addFwdGids.find (gid) != s_addFwdGids.end());
  }

  // 7-2用: 「Add-Forwarding zone 内か？」のラッパ（名前を7-2の説明に合わせた）
  bool IsInsideAddFwdZone () const { return IsInsideAddZone (); }

  // --- ユーティリティ: 送信ノードIP→そのg_idを取得（prev==srcのときに使用） ---
  // ※ s_senders に登録された送信ノード配列を走査してIP一致を探す
  static bool FindSenderGidByIp (Ipv4Address ip, uint32_t& outGid)
  {
    for (const auto& nd : s_senders) {
      if (!nd) continue;
      Ptr<Ipv4> ipv4 = nd->GetObject<Ipv4>();
      if (!ipv4) continue;
      // 本サンプルでは ifIndex=1 の0番目アドレスを使用
      Ipv4InterfaceAddress if0 = ipv4->GetAddress (1, 0);
      if (if0.GetLocal () == ip) {
        Ptr<MobilityModel> mm = nd->GetObject<MobilityModel>();
        if (!mm) return false;
        outGid = LocateGid (mm->GetPosition ());
        return true;
      }
    }
    return false;
  }

  // ---------- 受信処理 ----------
  // 全メッセージを受信。前ホップIPとヘッダを見て種別ごとに処理。
  void HandleRead (Ptr<Socket> socket)
  {
    Address from; Ptr<Packet> p;
    while ((p = socket->RecvFrom (from))) {
      // 直前ホップ（Wi-Fiの送信元IP）
      Ipv4Address prevHop = Ipv4Address("0.0.0.0");
      if (InetSocketAddress::IsMatchingType(from)) {
        prevHop = InetSocketAddress::ConvertFrom(from).GetIpv4();
      }

      GeoHeader hdr;
      if (p->GetSize () < hdr.GetSerializedSize()) continue;
      p->PeekHeader (hdr);

      // 重複判定キー（既存仕様に合わせ type も混ぜる）
      uint64_t key = (static_cast<uint64_t>(hdr.GetSrc().Get()) << 32)
                   | (static_cast<uint64_t>(hdr.GetSeq()) ^ (static_cast<uint64_t>(hdr.GetType()) << 24));

      // ★DATA以外は従来どおり「先に」重複排除する（DATAは後段で条件に応じて登録）
      bool isData = (hdr.GetType() == GeoHeader::DATA);
      if (!isData) {
        if (!m_seen.insert (key).second) continue;
      }

      switch (hdr.GetType()) {
      case GeoHeader::SELECT: {
        // 同g_id & 非送信のみ候補取り込み
        if (hdr.GetGridId () == m_g_id && !m_isSender) {
          ConsiderCandidate (hdr.GetGridId (), hdr.GetRound (), hdr.GetSrc (), hdr.GetDistance ());
          NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                          << m_myAddress << " RX(SELECT)"
                          << " prev=" << prevHop
                          << " src="  << hdr.GetSrc ()
                          << " g_id=" << hdr.GetGridId ()
                          << " dist=" << hdr.GetDistance ()
                          << " round="<< hdr.GetRound ());
        }
        break;
      }
      case GeoHeader::RESELECT: {
        // 旧GWからの通知。該当グリッドなら SELECT を再始動。
        if (hdr.GetGridId () == m_g_id && !m_isSender) {
          m_selectRound++;
          static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
          const double j = uv->GetValue(0.0, 0.008);
          Simulator::Schedule (Seconds (j), &SimpleFloodingApp::SendSelect, this);
          NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                        << m_myAddress << " RX(RESELECT)"
                        << " prev=" << prevHop
                        << " g_id=" << hdr.GetGridId ()
                        << " -> re-run SELECT round=" << m_selectRound);
        } else {
          NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                        << m_myAddress << " RX(RESELECT)"
                        << " prev=" << prevHop
                        << " g_id=" << hdr.GetGridId ());
        }
        break;
      }
      case GeoHeader::GATE: {
        // 同g_idのGATEを記録。BID後の応答なら結果待ち解除。
        if (hdr.GetGridId() == m_g_id) {
          m_lastGateRx = Simulator::Now();
          // BID後にGATE受信 → 結果待ちをキャンセル
          if (m_bidAckEvent.IsPending() && m_lastGateRx >= m_lastBidTx) {
            Simulator::Cancel(m_bidAckEvent);
          }
        }
        /*NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                        << m_myAddress << " RX(GATE)"
                        << " prev=" << prevHop
                        << " g_id=" << hdr.GetGridId ());
        */
        break;
      }
      case GeoHeader::BID: {
        // 自分がGWで同g_idなら即GATEで応答（無駄な交代を抑制）
        if (m_isGateway && hdr.GetGridId() == m_g_id) {
          SendGate();
        }
        NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                        << m_myAddress << " RX(BID)"
                        << " prev=" << prevHop
                        << " g_id=" << hdr.GetGridId ());
        break;
      }
      case GeoHeader::DATA: {
        NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                        << m_myAddress << " RX(DATA)"
                        << " prev=" << prevHop
                        << " src="  << hdr.GetSrc ()
                        << " seq="  << hdr.GetSeq ());

        // 7-1: 自身がゲートウェイノードでなければ転送しない
        if (!m_isGateway) {
          NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                          << m_myAddress << " DROP not-gateway");
          break;
        }

        // 7-2: FWDゾーン・Add-FWDゾーンのどちらにも含まれていなければ転送しない
        if (!IsInsideFwdZone ()) {
          if (!IsInsideAddFwdZone ()) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP out-of-zone");
            break;
          }
        }

        const bool directFromSrc = (prevHop == hdr.GetSrc());

        // ========= 直送パス (送信ノード→自分) =========
        if (directFromSrc) {

          // --- ここは単一版と同じ。送信元と同じ g_id の GW だけが「最初の前方転送」を行う ---
          uint32_t sgid = 0;
          if (!FindSenderGidByIp(hdr.GetSrc(), sgid)) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP cannot-resolve-src-grid");
            break; // 送信元 g_id が不明なら保守的にDROP
          }
          if (sgid != m_g_id) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP src-different-grid"
                            << " sgid=" << sgid << " my_gid=" << m_g_id);
            // ここでは m_seen に入れない → 後で他GWから届けば一度だけ転送できる
            break;
          }

          // 7-3: 2回目以降に受信したパケットであれば転送しない（重複判定）
          if (!m_seen.insert (key).second) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP dup-data");
            break;
          }

          // 7-4: 自身がいずれかの Geocast region 内であれば転送しない
          if (IsInsideAnyGeocastRegion()) {  // ★ 単一版 IsInsideGeocastRegion() を複数GR版に差し替え
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP inside-geocast-region");
            break;
          }

          // 7-5: 自身の g_id が「いずれかの Geocast region の隣接グリッド」かどうかで転送制御
          bool isNeighbor = IsNeighborOfAnyGeocastRegion(); // ★ 複数GR対応の隣接判定

          if (!isNeighbor) {
            // 隣接グリッドでなければ BCL による転送回数制限のみ適用
            if (s_BCL > 0 && m_forwardCount >= s_BCL) {
              NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                              << m_myAddress << " DROP BCL-limit"
                              << " fwdCount=" << m_forwardCount
                              << " BCL=" << s_BCL);
              break;
            }
          } else {
            // ★「複数Geocast regionそれぞれの隣接グリッドGW集合ごとに」1回だけ中継する
            uint64_t dataKey =
              (static_cast<uint64_t>(hdr.GetSrc().Get()) << 32) |
              static_cast<uint64_t>(hdr.GetSeq());

            // ★ 自グリッドが隣接しているGeocast regionのインデックス一覧を取得
            std::vector<uint32_t> regionIndices = GetAdjacentRegionIndicesForMyGrid ();

            bool insertedSomewhere = false; // ★ どこか1つのregion集合でも新規登録できたかフラグ
            for (uint32_t idx : regionIndices) {
              if (idx >= s_neighborDataForwardedPerRegion.size()) continue;
              // ★ regionごとの隣接グリッド転送済みDATA集合に登録
              if (s_neighborDataForwardedPerRegion[idx].insert (dataKey).second) {
                insertedSomewhere = true;
              }
            }

            if (!insertedSomewhere) {
              // ★ 自グリッドが属する全ての隣接region集合で既に中継済みならDROP
              NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                              << m_myAddress << " DROP neighbor-dup-data");
              break;
            }

            // ★ デバッグ用に全体集合にも記録（意味的には不要だが未使用警告回避も兼ねる）
            s_neighborDataForwarded.insert (dataKey);
          }

          // 7-6: 上記条件に当てはまらなければ転送（ブロードキャスト、0〜0.6msジッタ）
          static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
          double jitter = uv->GetValue (0.0, 0.0006);
          Ptr<Packet> cp = p->Copy();
          GeoHeader hdrCopy = hdr;
          Simulator::Schedule (Seconds (jitter), [this, cp, hdrCopy]() {
            this->Forward (cp, hdrCopy);
          });
        }

        // ========= 他GWからのパス (中継済み → 自分) =========
        else {

          // 7-3: 2回目以降に受信したパケットであれば転送しない（重複判定）
          if (!m_seen.insert (key).second) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP dup-data");
            break;
          }

          // 7-4: 自身がいずれかの Geocast region 内であれば転送しない
          if (IsInsideAnyGeocastRegion()) {  // ★ 単一版 IsInsideGeocastRegion() を複数GR版に差し替え
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP inside-geocast-region");
            break;
          }

          // 7-5: 自身の g_id が「いずれかの Geocast region の隣接グリッド」かどうかで転送制御
          bool isNeighbor = IsNeighborOfAnyGeocastRegion(); // ★ 複数GR対応の隣接判定

          if (!isNeighbor) {
            // 隣接グリッドでなければ BCL による転送回数制限のみ適用
            if (s_BCL > 0 && m_forwardCount >= s_BCL) {
              NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                              << m_myAddress << " DROP BCL-limit"
                              << " fwdCount=" << m_forwardCount
                              << " BCL=" << s_BCL);
              break;
            }
          } else {
            // ★「複数Geocast regionそれぞれの隣接グリッドGW集合ごとに」1回だけ中継する
            uint64_t dataKey =
              (static_cast<uint64_t>(hdr.GetSrc().Get()) << 32) |
              static_cast<uint64_t>(hdr.GetSeq());

            // ★ 自グリッドが隣接しているGeocast regionのインデックス一覧を取得
            std::vector<uint32_t> regionIndices = GetAdjacentRegionIndicesForMyGrid ();

            bool insertedSomewhere = false; // ★ どこか1つのregion集合でも新規登録できたかフラグ
            for (uint32_t idx : regionIndices) {
              if (idx >= s_neighborDataForwardedPerRegion.size()) continue;
              // ★ regionごとの隣接グリッド転送済みDATA集合に登録
              if (s_neighborDataForwardedPerRegion[idx].insert (dataKey).second) {
                insertedSomewhere = true;
              }
            }

            if (!insertedSomewhere) {
              // ★ 自グリッドが属する全ての隣接region集合で既に中継済みならDROP
              NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                              << m_myAddress << " DROP neighbor-dup-data");
              break;
            }

            // ★ デバッグ用に全体集合にも記録（意味的には不要だが未使用警告回避も兼ねる）
            s_neighborDataForwarded.insert (dataKey);
          }

          // 7-6: 上記条件に当てはまらなければ転送（ブロードキャスト、0〜0.6msジッタ）
          static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
          double jitter = uv->GetValue (0.0, 0.0006);
          Ptr<Packet> cp = p->Copy();
          GeoHeader hdrCopy = hdr;
          Simulator::Schedule (Seconds (jitter), [this, cp, hdrCopy]() {
            this->Forward (cp, hdrCopy);
          });
        }

        break;
      }

      default: break;
      }
    }
  }

private:
  // --- インスタンス状態 ---
  Ptr<Socket> m_socket;
  EventId     m_sendEvent;
  Ipv4Address m_myAddress{"0.0.0.0"};
  uint16_t    m_port{9999};
  Time        m_sendInterval{Seconds(1.0)};
  bool        m_isSender{false};
  uint32_t    m_senderIndex{0};

  uint32_t    m_nextSeq{0};      // DATA seq
  uint32_t    m_forwardCount{0}; // これまでこのGWが行ったDATA前方転送数（BCL判定用）

  // SELECT ラウンドと候補集約
  uint32_t    m_selectRound{1};
  uint32_t    m_selectSeq{0};
  std::map<uint64_t, SelState> m_sel;
  std::map<uint64_t, EventId>  m_selFinalize;
  std::set<uint64_t> m_seen;

  // 自グリッド情報（境界とGW状態）
  uint32_t m_g_id{0};
  double   m_gxmin{0.0}, m_gymin{0.0};
  double   m_gxmax{0.0}, m_gymax{0.0};
  bool     m_isGateway{false};

  // アプリ属性（Geocast関連：全regionのBBOX）
  double m_grXmin{0.0}, m_grYmin{0.0};
  double m_grXmax{0.0}, m_grYmax{0.0};
  uint32_t m_s_id{1};

  // グリッド越え検知（CourseChange + 50ms ポーリング）
  EventId m_gridCheckEvent;
  Time    m_gridCheckInterval{MilliSeconds(50)};
  Ptr<MobilityModel> m_mobility;

  // GATE/BID 制御
  EventId m_gateEvent;
  EventId m_gateWatchdogEvent;
  EventId m_bidAckEvent;                 // BID結果待ちタイマーとして再利用
  Time    m_gateInterval{MilliSeconds(1000)};
  Time    m_gateTimeout{MilliSeconds(3000)};
  Time    m_bidResponseWait{MilliSeconds(500)}; // BIDの応答待ち時間
  Time    m_lastGateRx{Seconds(0)};
  Time    m_lastBidTx{Seconds(0)};      // 直近のBID送信時刻

  // 共有（FWDゾーン）
  static bool   s_zoneComputed;
  static double s_grXmin, s_grYmin, s_grXmax, s_grYmax; // 全regionのBBOX用
  static uint32_t s_s_id;
  static double s_rXmin, s_rYmin, s_rXmax, s_rYmax;
  static std::vector< Ptr<Node> > s_senders;
  static std::vector<GeoRegionRect> s_geocastRegions;   // 個々のGeocast region群
  static uint32_t s_NCN;                                // Add-Forwarding zone 用のNCN
  static bool    s_addZoneComputed;                     // Add-Forwarding zone 計算済みフラグ
  static std::set<uint32_t> s_addFwdGids;               // Add-Forwarding zone に含めるg_id集合
  static uint32_t s_BCL;                                // ★ BCL: ゲートウェイ転送上限（0=無制限）
  static std::set<uint64_t> s_neighborDataForwarded;    // ★ 隣接グリッドGW全体での「既に転送済みDATA(src,seq)」集合（全体用）

  // Geocast region とグリッドIDの対応（region内/隣接）のキャッシュ
  static bool s_geoGridComputed;
  static std::set<uint32_t> s_geoGids;
  static std::set<uint32_t> s_adjacentGids;

  // ★ regionごとの「内部/隣接」グリッド集合と、隣接グリッド用DATA転送済み集合
  static std::vector< std::set<uint32_t> > s_geoGidsPerRegion;      // ★ 各Geocast region内部のg_id集合
  static std::vector< std::set<uint32_t> > s_adjacentGidsPerRegion; // ★ 各Geocast region隣接のg_id集合
  static std::vector< std::set<uint64_t> > s_neighborDataForwardedPerRegion; // ★ 各regionの隣接グリッドGW集合ごとの転送済みDATA(src,seq)

  // 共有（グリッド境界）
  static bool   s_gridDefined;
  static double s_gridOriginX, s_gridOriginY;
  static double s_grid_d;
  static uint32_t s_gridNx, s_gridNy;
  static std::vector<double> s_GXmin, s_GXmax, s_GYmin, s_GYmax;
};

// ====== 静的メンバ定義 ======
// Geocast/FWD ゾーン関連の共有状態
bool   SimpleFloodingApp::s_zoneComputed = false;
double SimpleFloodingApp::s_grXmin = 0.0, SimpleFloodingApp::s_grYmin = 0.0;
double SimpleFloodingApp::s_grXmax = 0.0, SimpleFloodingApp::s_grYmax = 0.0;
uint32_t SimpleFloodingApp::s_s_id = 1;
double SimpleFloodingApp::s_rXmin = 0.0, SimpleFloodingApp::s_rYmin = 0.0;
double SimpleFloodingApp::s_rXmax = 0.0, SimpleFloodingApp::s_rYmax = 0.0;
std::vector< Ptr<Node> > SimpleFloodingApp::s_senders;
std::vector<SimpleFloodingApp::GeoRegionRect> SimpleFloodingApp::s_geocastRegions;
uint32_t SimpleFloodingApp::s_NCN = 0;
bool SimpleFloodingApp::s_addZoneComputed = false;
std::set<uint32_t> SimpleFloodingApp::s_addFwdGids;
uint32_t SimpleFloodingApp::s_BCL = 0;
std::set<uint64_t> SimpleFloodingApp::s_neighborDataForwarded;


// Geocast region とグリッド対応キャッシュ
bool SimpleFloodingApp::s_geoGridComputed = false;
std::set<uint32_t> SimpleFloodingApp::s_geoGids;
std::set<uint32_t> SimpleFloodingApp::s_adjacentGids;

// ★ regionごとの内部/隣接グリッドと隣接グリッド用DATA転送済み集合
std::vector< std::set<uint32_t> > SimpleFloodingApp::s_geoGidsPerRegion;
std::vector< std::set<uint32_t> > SimpleFloodingApp::s_adjacentGidsPerRegion;
std::vector< std::set<uint64_t> > SimpleFloodingApp::s_neighborDataForwardedPerRegion;

// グリッド境界テーブル
bool   SimpleFloodingApp::s_gridDefined = false;
double SimpleFloodingApp::s_gridOriginX = 0.0, SimpleFloodingApp::s_gridOriginY = 0.0;
double SimpleFloodingApp::s_grid_d = 1.0;
uint32_t SimpleFloodingApp::s_gridNx = 0, SimpleFloodingApp::s_gridNy = 0;
std::vector<double> SimpleFloodingApp::s_GXmin, SimpleFloodingApp::s_GXmax;
std::vector<double> SimpleFloodingApp::s_GYmin, SimpleFloodingApp::s_GYmax;

// Geocast region とグリッドIDの対応（region内/隣接）を一度だけ計算
void
SimpleFloodingApp::MaybeComputeGeoGridSets ()
{
  if (s_geoGridComputed) return;
  s_geoGids.clear();
  s_adjacentGids.clear();

  if (!s_gridDefined || s_gridNx == 0 || s_gridNy == 0) {
    s_geoGridComputed = true;
    return;
  }
  if (s_geocastRegions.empty()) {
    s_geoGridComputed = true;
    return;
  }

  auto isInsideAnyRegion = [](double x, double y)->bool {
    for (const auto& r : s_geocastRegions) {
      if (x >= r.xmin && x <= r.xmax &&
          y >= r.ymin && y <= r.ymax) {
        return true;
      }
    }
    return false;
  };

  uint32_t total = s_gridNx * s_gridNy;
  for (uint32_t gid = 1; gid <= total; ++gid) {
    uint32_t idx = gid - 1;
    double cx = 0.5 * (s_GXmin[idx] + s_GXmax[idx]);
    double cy = 0.5 * (s_GYmin[idx] + s_GYmax[idx]);
    if (isInsideAnyRegion (cx, cy)) {
      s_geoGids.insert (gid);
    }
  }

  auto addNeighbor = [&](int gidInt) {
    if (gidInt < 1 || gidInt > static_cast<int>(total)) return;
    uint32_t ng = static_cast<uint32_t>(gidInt);
    if (s_geoGids.count (ng)) return; // regionセル自身は含めない
    s_adjacentGids.insert (ng);
  };

  for (uint32_t gid : s_geoGids) {
    int g = static_cast<int>(gid);
    addNeighbor (g - static_cast<int>(s_gridNx) - 1);
    addNeighbor (g - static_cast<int>(s_gridNx));
    addNeighbor (g - static_cast<int>(s_gridNx) + 1);
    addNeighbor (g - 1);
    addNeighbor (g + 1);
    addNeighbor (g + static_cast<int>(s_gridNx) - 1);
    addNeighbor (g + static_cast<int>(s_gridNx));
    addNeighbor (g + static_cast<int>(s_gridNx) + 1);
  }

  // ★ 各Geocast regionごとの「内部/隣接」グリッド集合と、隣接グリッド用DATA転送済み集合も構築
  s_geoGidsPerRegion.clear();
  s_adjacentGidsPerRegion.clear();
  s_neighborDataForwardedPerRegion.clear();

  const size_t nReg = s_geocastRegions.size();
  s_geoGidsPerRegion.resize (nReg);
  s_adjacentGidsPerRegion.resize (nReg);
  s_neighborDataForwardedPerRegion.resize (nReg);

  // ★ regionごとの内部g_id集合を作成
  for (size_t i = 0; i < nReg; ++i) {
    const auto& r = s_geocastRegions[i];
    for (uint32_t gid = 1; gid <= total; ++gid) {
      uint32_t idx = gid - 1;
      double cx = 0.5 * (s_GXmin[idx] + s_GXmax[idx]);
      double cy = 0.5 * (s_GYmin[idx] + s_GYmax[idx]);
      if (cx >= r.xmin && cx <= r.xmax &&
          cy >= r.ymin && cy <= r.ymax) {
        s_geoGidsPerRegion[i].insert (gid);
      }
    }
  }

  // ★ regionごとの隣接g_id集合を作成（内部セルの8近傍 - 内部セル）
  auto addNeighborPerRegion = [&](size_t regIdx, int gidInt)
  {
    if (gidInt < 1 || gidInt > static_cast<int>(total)) return;
    uint32_t ng = static_cast<uint32_t>(gidInt);
    if (s_geoGidsPerRegion[regIdx].count (ng)) return; // そのregionの内部セルは含めない
    s_adjacentGidsPerRegion[regIdx].insert (ng);
  };

  for (size_t i = 0; i < nReg; ++i) {
    for (uint32_t gid : s_geoGidsPerRegion[i]) {
      int g = static_cast<int>(gid);
      addNeighborPerRegion (i, g - static_cast<int>(s_gridNx) - 1);
      addNeighborPerRegion (i, g - static_cast<int>(s_gridNx));
      addNeighborPerRegion (i, g - static_cast<int>(s_gridNx) + 1);
      addNeighborPerRegion (i, g - 1);
      addNeighborPerRegion (i, g + 1);
      addNeighborPerRegion (i, g + static_cast<int>(s_gridNx) - 1);
      addNeighborPerRegion (i, g + static_cast<int>(s_gridNx));
      addNeighborPerRegion (i, g + static_cast<int>(s_gridNx) + 1);
    }
  }

  s_geoGridComputed = true;
}

bool
SimpleFloodingApp::IsInsideGeocastRegionGrid () const
{
  MaybeComputeGeoGridSets ();
  if (!s_geoGridComputed) return false;
  if (m_g_id == 0) return false;
  return (s_geoGids.count (m_g_id) != 0);
}

bool
SimpleFloodingApp::IsAdjacentToGeocastRegionGrid () const
{
  MaybeComputeGeoGridSets ();
  if (!s_geoGridComputed) return false;
  if (m_g_id == 0) return false;
  return (s_adjacentGids.count (m_g_id) != 0);
}

std::vector<uint32_t>
SimpleFloodingApp::GetAdjacentRegionIndicesForMyGrid () const
{
  // ★ 自グリッドが属するGeocast region隣接集合のインデックスを列挙
  std::vector<uint32_t> out;
  MaybeComputeGeoGridSets ();
  if (!s_geoGridComputed) return out;
  if (m_g_id == 0) return out;
  if (s_adjacentGidsPerRegion.empty()) return out;

  for (uint32_t i = 0; i < s_adjacentGidsPerRegion.size (); ++i) {
    if (s_adjacentGidsPerRegion[i].count (m_g_id)) {
      out.push_back (i);
    }
  }
  return out;
}

// Geocast region の共有設定
void
SimpleFloodingApp::SetGeocastRegions (const std::vector<GeoRegionRect>& regs)
{
  s_geocastRegions = regs;

  if (!s_geocastRegions.empty()) {
    s_grXmin = s_geocastRegions[0].xmin;
    s_grYmin = s_geocastRegions[0].ymin;
    s_grXmax = s_geocastRegions[0].xmax;
    s_grYmax = s_geocastRegions[0].ymax;
    for (size_t i = 1; i < s_geocastRegions.size(); ++i) {
      s_grXmin = std::min (s_grXmin, s_geocastRegions[i].xmin);
      s_grYmin = std::min (s_grYmin, s_geocastRegions[i].ymin);
      s_grXmax = std::max (s_grXmax, s_geocastRegions[i].xmax);
      s_grYmax = std::max (s_grYmax, s_geocastRegions[i].ymax);
    }
  }

  // FWDゾーンは再計算させる
  s_zoneComputed = false;
  // Geocast region とグリッド対応も再計算させる
  s_geoGridComputed = false;
}

// NCN設定（0〜8にクリップし、Add-Forwarding zone を再計算させる）
void
SimpleFloodingApp::SetNCN (uint32_t n)
{
  if (n > 8) {
    s_NCN = 8;
  } else {
    s_NCN = n;
  }
  s_addZoneComputed = false;
}

// BCL設定（0=無制限）
void
SimpleFloodingApp::SetBCL (uint32_t bcl)
{
  s_BCL = bcl;
}

// ========== 送信ノード座標 "x:y,x:y,..." 解析 ==========
// CLI 文字列を Vector の配列に（座標未指定ぶんは x=50*i のデフォルト）
static std::vector<Vector> ParseSrcCoords (const std::string& s)
{
  std::vector<Vector> out;
  if (s.empty()) return out;
  std::string token; std::stringstream ss(s);
  while (std::getline(ss, token, ',')) {
    size_t p = token.find(':'); if (p == std::string::npos) continue;
    std::string xs = token.substr(0, p), ys = token.substr(p+1);
    char* e1 = nullptr; char* e2 = nullptr;
    double x = std::strtod(xs.c_str(), &e1);
    double y = std::strtod(ys.c_str(), &e2);
    if (e1 == xs.c_str() || e2 == ys.c_str()) continue;
    out.emplace_back(x, y, 0.0);
  }
  return out;
}

// ============================== main() ==============================
// ノード生成→Wi-Fi/移動/IP/アプリ投入→シミュレーション実行 まで。
int main (int argc, char *argv[])
{
  uint32_t nNodes   = 0;
  double   stopTime = 2.0;   // 実行時間
  double   interval = 1.0;    // 送信ノード送信周期
  uint32_t nSources = 1;      // 送信ノード数
  uint32_t srcId    = 1;      // FWDゾーンの基準送信ノードID
  std::string srcCoordsStr = "";

  // 旧インタフェース（単一region指定）用
  double grXmin = 0.0, grYmin = 0.0, grXmax = 0.0, grYmax = 0.0;
  // 新インタフェース：複数Geocast region（ここでは最大4個までサンプル）
  double gr1Xmin = 0.0, gr1Ymin = 0.0, gr1Xmax = 0.0, gr1Ymax = 0.0;
  double gr2Xmin = 0.0, gr2Ymin = 0.0, gr2Xmax = 0.0, gr2Ymax = 0.0;
  double gr3Xmin = 0.0, gr3Ymin = 0.0, gr3Xmax = 0.0, gr3Ymax = 0.0;
  double gr4Xmin = 0.0, gr4Ymin = 0.0, gr4Xmax = 0.0, gr4Ymax = 0.0;

  // ランダム配置パラメータ（非送信ノードの数と矩形領域）
  uint32_t nodeCount = 400;                // 送信以外のノード数
  double nodeXmin = -500.0, nodeYmin = -500.0, nodeXmax = 500.0, nodeYmax = 500.0;

  // Add-Forwarding zone 用 NCN (0〜8)
  uint32_t NCN = 0;

  // BCL: ゲートウェイノードの転送制限数（0=制限なし）
  uint32_t BCL = 0;

  // --- CLI 引数の定義と読み込み ---
  CommandLine cmd(__FILE__);
  cmd.AddValue ("nNodes",    "Number of nodes", nNodes);
  cmd.AddValue ("stopTime",  "Simulation stop time (s)", stopTime);
  cmd.AddValue ("interval",  "Source send interval (s)", interval);
  cmd.AddValue ("nSources",  "Number of source nodes from index 0..", nSources);
  cmd.AddValue ("srcId",     "Source node id (1-based) used for FWD-zone calculation", srcId);

  // 旧インタフェース
  cmd.AddValue ("grXmin",    "Geocast region xmin (legacy single-region or bbox)", grXmin);
  cmd.AddValue ("grYmin",    "Geocast region ymin (legacy single-region or bbox)", grYmin);
  cmd.AddValue ("grXmax",    "Geocast region xmax (legacy single-region or bbox)", grXmax);
  cmd.AddValue ("grYmax",    "Geocast region ymax (legacy single-region or bbox)", grYmax);

  // 新インタフェース：複数region
  cmd.AddValue ("gr1Xmin",   "Geocast region #1 xmin", gr1Xmin);
  cmd.AddValue ("gr1Ymin",   "Geocast region #1 ymin", gr1Ymin);
  cmd.AddValue ("gr1Xmax",   "Geocast region #1 xmax", gr1Xmax);
  cmd.AddValue ("gr1Ymax",   "Geocast region #1 ymax", gr1Ymax);

  cmd.AddValue ("gr2Xmin",   "Geocast region #2 xmin", gr2Xmin);
  cmd.AddValue ("gr2Ymin",   "Geocast region #2 ymin", gr2Ymin);
  cmd.AddValue ("gr2Xmax",   "Geocast region #2 xmax", gr2Xmax);
  cmd.AddValue ("gr2Ymax",   "Geocast region #2 ymax", gr2Ymax);

  cmd.AddValue ("gr3Xmin",   "Geocast region #3 xmin", gr3Xmin);
  cmd.AddValue ("gr3Ymin",   "Geocast region #3 ymin", gr3Ymin);
  cmd.AddValue ("gr3Xmax",   "Geocast region #3 xmax", gr3Xmax);
  cmd.AddValue ("gr3Ymax",   "Geocast region #3 ymax", gr3Ymax);

  cmd.AddValue ("gr4Xmin",   "Geocast region #4 xmin", gr4Xmin);
  cmd.AddValue ("gr4Ymin",   "Geocast region #4 ymin", gr4Ymin);
  cmd.AddValue ("gr4Xmax",   "Geocast region #4 xmax", gr4Xmax);
  cmd.AddValue ("gr4Ymax",   "Geocast region #4 ymax", gr4Ymax);

  cmd.AddValue ("srcCoords", "Comma-separated list of source coordinates: x:y,x:y,...", srcCoordsStr);

  cmd.AddValue ("nodeCount", "Number of non-sender nodes to place randomly (overrides nNodes)", nodeCount);
  cmd.AddValue ("nodeXmin",  "Random placement Xmin", nodeXmin);
  cmd.AddValue ("nodeYmin",  "Random placement Ymin", nodeYmin);
  cmd.AddValue ("nodeXmax",  "Random placement Xmax", nodeXmax);
  cmd.AddValue ("nodeYmax",  "Random placement Ymax", nodeYmax);

  cmd.AddValue ("NCN",       "Number of encoded packets (0-8) used for Add-Forwarding zone", NCN);

  // BCL: ゲートウェイのDATA前方転送上限（0=無制限）
  cmd.AddValue ("BCL",       "Forwarding limit per gateway node (0 = no limit)", BCL);

  cmd.Parse (argc, argv);

  // NCN は 0〜8 にクリップしてアプリ側へ共有
  if (NCN > 8) NCN = 8;
  SimpleFloodingApp::SetNCN (NCN);

  // BCL をアプリ側へ共有（0=無制限）
  SimpleFloodingApp::SetBCL (BCL);

  // --- Geocast region の集約（複数矩形対応） ---
  std::vector<SimpleFloodingApp::GeoRegionRect> geocastRegions;
  auto addRegionIfValid =
    [&geocastRegions](double xmin, double ymin, double xmax, double ymax)
    {
      if (xmax > xmin && ymax > ymin) {
        SimpleFloodingApp::GeoRegionRect r { xmin, ymin, xmax, ymax };
        geocastRegions.push_back (r);
      }
    };

  // 旧インタフェース（grXmin..）も region として扱う（指定されていれば）
  addRegionIfValid (grXmin, grYmin, grXmax, grYmax);
  // 新インタフェース: gr1.., gr2.., gr3.., gr4..
  addRegionIfValid (gr1Xmin, gr1Ymin, gr1Xmax, gr1Ymax);
  addRegionIfValid (gr2Xmin, gr2Ymin, gr2Xmax, gr2Ymax);
  addRegionIfValid (gr3Xmin, gr3Ymin, gr3Xmax, gr3Ymax);
  addRegionIfValid (gr4Xmin, gr4Ymin, gr4Xmax, gr4Ymax);

  if (!geocastRegions.empty()) {
    // 全region をアプリ側に登録
    SimpleFloodingApp::SetGeocastRegions (geocastRegions);

    // grXmin.. は「全regionを含む大域矩形(BBOX)」として更新し、アプリ属性に渡す
    grXmin = geocastRegions[0].xmin;
    grYmin = geocastRegions[0].ymin;
    grXmax = geocastRegions[0].xmax;
    grYmax = geocastRegions[0].ymax;
    for (size_t i = 1; i < geocastRegions.size(); ++i) {
      grXmin = std::min (grXmin, geocastRegions[i].xmin);
      grYmin = std::min (grYmin, geocastRegions[i].ymin);
      grXmax = std::max (grXmax, geocastRegions[i].xmax);
      grYmax = std::max (grYmax, geocastRegions[i].ymax);
    }

    NS_LOG_UNCOND ("GEOCAST-REGIONS: count=" << geocastRegions.size()
                    << " BBOX=[" << grXmin << "," << grYmin
                    << "]-[" << grXmax << "," << grYmax << "]");
  } else {
    // region が与えられない場合は、FWDゾーンは「送信ノード周辺のみ」に近い形になる
    SimpleFloodingApp::SetGeocastRegions (geocastRegions); // 空でクリアしておく
  }

  // 総ノード数 = 送信 + 非送信
  if (nodeCount > 0) {
    nNodes = nSources + nodeCount;
  }

  // --- ノード生成と役割分割 ---
  NodeContainer nodes; nodes.Create (nNodes);
  NodeContainer srcNodes, otherNodes;
  for (uint32_t i = 0; i < nodes.GetN (); ++i) {
    (i < nSources ? srcNodes : otherNodes).Add (nodes.Get (i));
  }
  uint32_t nSrc = srcNodes.GetN ();

  // --- グリッド定義（50mセル、[-50,350]×[-50,350]） ---
  const double dCell = 50.0;
  const double gOrgX = -500.0, gOrgY = -500.0;
  const double gEndX = 500.0,  gEndY = 500.0;
  const uint32_t Nx = static_cast<uint32_t>(std::ceil((gEndX - gOrgX) / dCell));
  const uint32_t Ny = static_cast<uint32_t>(std::ceil((gEndY - gOrgY) / dCell));
  SimpleFloodingApp::DefineGrid (gOrgX, gOrgY, dCell, Nx, Ny);

  // --- Wi-Fi（802.11b） ---
  WifiHelper wifi; wifi.SetStandard (WIFI_STANDARD_80211b);
  YansWifiPhyHelper phy;
  // 送信電力を単一値に固定（約24.45 dBm）
  phy.Set ("TxPowerStart", DoubleValue (24.45));
  phy.Set ("TxPowerEnd",   DoubleValue (24.45));
  phy.Set ("TxPowerLevels", UintegerValue (1));
  phy.Set ("TxGain", DoubleValue (0.0));
  phy.Set ("RxGain", DoubleValue (0.0));

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());
  WifiMacHelper mac; mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devs = wifi.Install (phy, mac, nodes);
  
  /*// Radiotap + 全ノードでPCAP
  phy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.EnablePcapAll ("pcap/flooding", true); // true=promiscuous（全部拾う）*/

  // --- 送信ノード：静止配置（CLIの --srcCoords を尊重） ---
  std::vector<Vector> srcPos = ParseSrcCoords (srcCoordsStr);
  for (uint32_t i = srcPos.size(); i < nSrc; ++i) srcPos.emplace_back(50.0 * i, 0.0, 0.0);
  if (nSrc > 0) {
    Ptr<ListPositionAllocator> list = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < nSrc; ++i) list->Add (srcPos[i]);
    MobilityHelper mobSrc; mobSrc.SetPositionAllocator (list);
    mobSrc.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobSrc.Install (srcNodes);
  }
  
  /*
  // --- 非送信ノード：RandomWaypoint（矩形内を移動） ---
  // ※ 速度は今は 5.0 m/s。4km/h にしたい場合は 1.111111 に変更
  {
    if (nodeXmax < nodeXmin) std::swap(nodeXmin, nodeXmax);
    if (nodeYmax < nodeYmin) std::swap(nodeYmin, nodeYmax);

    Ptr<UniformRandomVariable> ux = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> uy = CreateObject<UniformRandomVariable>();
    ux->SetAttribute ("Min", DoubleValue (nodeXmin));
    ux->SetAttribute ("Max", DoubleValue (nodeXmax));
    uy->SetAttribute ("Min", DoubleValue (nodeYmin));
    uy->SetAttribute ("Max", DoubleValue (nodeYmax));

    Ptr<RandomRectanglePositionAllocator> randAlloc = CreateObject<RandomRectanglePositionAllocator>();
    randAlloc->SetX (ux);
    randAlloc->SetY (uy);

    MobilityHelper mobOther;
    mobOther.SetPositionAllocator (randAlloc);
    mobOther.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                               "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=5.0]"),
                               "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
                               "PositionAllocator", PointerValue (randAlloc));
    mobOther.Install (otherNodes);
  }
  */
  
  
  // /*
  // --- 非送信ノード：格子状に固定配置（デバッグ用） ---
  {
    const uint32_t N = otherNodes.GetN();

    const double startX = -475.0;  // 格子の起点X
    const double startY = -475.0;  // 格子の起点Y
    const double pitch  = 50.0;  // x・y 共通の格子間隔（m）

    // ほぼ正方に並べるための列数（必要なら固定列数にしてもOK）
    const uint32_t cols = static_cast<uint32_t>(
        std::ceil(std::sqrt(static_cast<double>(N)))
    );

    Ptr<ListPositionAllocator> list = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < N; ++i) {
      const uint32_t r = i / cols;   // 行
      const uint32_t c = i % cols;   // 列
      const double x = startX + pitch * c; // x は +50 ずつ
      const double y = startY + pitch * r; // y も +50 ずつ
      list->Add(Vector(x, y, 0.0));
    }

    MobilityHelper mobOther;
    mobOther.SetPositionAllocator(list);
    // 移動させず固定
    mobOther.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobOther.Install(otherNodes);
  }
  //*/

  // --- IP スタック / アドレス割当（/22） ---
  InternetStackHelper internet; internet.Install (nodes);
  Ipv4AddressHelper ipv4 ("10.0.0.0", "255.255.252.0");
  Ipv4InterfaceContainer ifaces = ipv4.Assign (devs);

  // --- NetAnim（任意可視化：送信ノードは赤） ---
  AnimationInterface anim ("anim/flooding.xml");
  anim.EnablePacketMetadata (true);
  anim.SetMobilityPollInterval (Seconds (0.5));
  for (uint32_t i = 0; i < nodes.GetN (); ++i) {
    uint32_t id = nodes.Get (i)->GetId (); std::ostringstream oss; oss << "N" << id;
    anim.UpdateNodeDescription (id, oss.str ()); anim.UpdateNodeColor (id, 0, 180, 255);
  }
  for (uint32_t i = 0; i < srcNodes.GetN (); ++i) {
    uint32_t id = srcNodes.Get (i)->GetId (); anim.UpdateNodeColor (id, 255, 80, 80);
  }

  // --- アプリ配備：送信ノード（Start=1.0s, 小ジッタ後にDATA開始） ---
  for (uint32_t i = 0; i < srcNodes.GetN (); ++i) {
    Ptr<SimpleFloodingApp> app = CreateObject<SimpleFloodingApp>();
    app->SetAttribute ("SendInterval", TimeValue (Seconds (interval)));
    app->SetAttribute ("IsSender", BooleanValue (true));
    app->SetAttribute ("SenderIndex", UintegerValue (i + 1));
    app->SetAttribute ("grXmin", DoubleValue (grXmin));
    app->SetAttribute ("grYmin", DoubleValue (grYmin));
    app->SetAttribute ("grXmax", DoubleValue (grXmax));
    app->SetAttribute ("grYmax", DoubleValue (grYmax));
    app->SetAttribute ("S_id",   UintegerValue (srcId));
    srcNodes.Get (i)->AddApplication (app);
    app->SetStartTime (Seconds (1.0));
    app->SetStopTime  (Seconds (stopTime));
  }
  // --- アプリ配備：非送信ノード（起動ほぼ即時→SELECT拡散） ---
  for (uint32_t i = 0; i < otherNodes.GetN (); ++i) {
    Ptr<SimpleFloodingApp> app = CreateObject<SimpleFloodingApp>();
    app->SetAttribute ("SendInterval", TimeValue (Seconds (interval)));
    app->SetAttribute ("IsSender", BooleanValue (false));
    app->SetAttribute ("SenderIndex", UintegerValue (0));
    app->SetAttribute ("grXmin", DoubleValue (grXmin));
    app->SetAttribute ("grYmin", DoubleValue (grYmin));
    app->SetAttribute ("grXmax", DoubleValue (grXmax));
    app->SetAttribute ("grYmax", DoubleValue (grYmax));
    app->SetAttribute ("S_id",   UintegerValue (srcId));
    otherNodes.Get (i)->AddApplication (app);
    app->SetStartTime (Seconds (0.0001));
    app->SetStopTime  (Seconds (stopTime));
  }

  // --- 実行 ---
  Simulator::Stop (Seconds (stopTime));
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}

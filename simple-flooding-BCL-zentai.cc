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
#include <limits>
#include <map>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <unordered_set>

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
      // Geocast 有効/無効
      .AddAttribute ("UseGeocast", "Enable forwarding zone (Geocast-style).",
                     BooleanValue (false),
                     MakeBooleanAccessor (&SimpleFloodingApp::m_useGeocastAttr),
                     MakeBooleanChecker ())
      // Geocast region の矩形
      .AddAttribute ("grXmin", "Geocast region xmin.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grXmin),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("grYmin", "Geocast region ymin.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grYmin),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("grXmax", "Geocast region xmax.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grXmax),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("grYmax", "Geocast region ymax.", DoubleValue (0.0),
                     MakeDoubleAccessor (&SimpleFloodingApp::m_grYmax),
                     MakeDoubleChecker<double> ())
      // FWDゾーンの基準となる送信ノードID（1始まり）
      .AddAttribute ("S_id", "Source id (1-based) used to anchor the forwarding zone.",
                     UintegerValue (1),
                     MakeUintegerAccessor (&SimpleFloodingApp::m_s_id),
                     MakeUintegerChecker<uint32_t> ())
      // NCN: 符号化パケット作成数（Add-Forwarding zone制御用）
      .AddAttribute ("NCN", "Number of coded packets (controls Add-Forwarding zone).",
                     UintegerValue (0),
                     MakeUintegerAccessor (&SimpleFloodingApp::m_ncn),
                     MakeUintegerChecker<uint32_t> ())
      // BCL: ゲートウェイノードの転送制限数（0なら制限なし）
      .AddAttribute ("BCL", "Forwarding limit per gateway node (0 = no limit).",
                     UintegerValue (0),
                     MakeUintegerAccessor (&SimpleFloodingApp::m_bcl),
                     MakeUintegerChecker<uint32_t> ());
    
    return tid;
  }

  SimpleFloodingApp () = default;
  virtual ~SimpleFloodingApp () = default;

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
    NS_LOG_UNCOND ("GRID-DEF: origin=(" << originX << "," << originY << ") d=" << d
                    << " Nx=" << nx << " Ny=" << ny << " total=" << (nx*ny));
  }

private:
  // ---------- Forwarding zone 算出 ----------
  // Geocast region と「送信ノードの所属グリッド」の境界から一度だけ矩形を作る 
  static void MaybeComputeZone ()
  {
    if (!s_useGeocast || s_zoneComputed) return;
    // Add-FWD の計算もここで一度だけ行う（NCNに依存）
    
    if (s_s_id == 0) return;
    uint32_t idx = s_s_id - 1;
    if (idx >= s_senders.size ()) return;
    Ptr<Node> anchor = s_senders[idx]; if (!anchor) return;
    Ptr<MobilityModel> mm = anchor->GetObject<MobilityModel>(); if (!mm) return;
    Vector spos = mm->GetPosition ();

    // 送信ノードの所属グリッドを求め、そのセル境界を取得
    uint32_t sgid = LocateGid (spos);
    s_srcGridId = sgid; // Add-Forwarding: 送信元グリッド保持
    if (sgid == 0) return;
   
    double gxmin = s_GXmin[sgid - 1], gxmax = s_GXmax[sgid - 1];
    double gymin = s_GYmin[sgid - 1], gymax = s_GYmax[sgid - 1];

     
    // X方向: grXmin と グリッドXmin を比較
    //   grXmin >= GXmin → rXmin=GXmin, rXmax=grXmax
    //   grXmin <  GXmin → rXmin=grXmin, rXmax=GXmax
    if (s_grXmin >= gxmin) { s_rXmin = gxmin; s_rXmax = s_grXmax; }
    else                    { s_rXmin = s_grXmin; s_rXmax = gxmax; }

    // Y方向: grYmin と グリッドYmin を比較
    //   grYmin >= GYmin → rYmin=GYmin, rYmax=grYmax
    //   grYmin <  GYmin → rYmin=grYmin, rYmax=GYmax
    if (s_grYmin >= gymin) { s_rYmin = gymin; s_rYmax = s_grYmax; }
    else                    { s_rYmin = s_grYmin; s_rYmax = gymax; }

    
    NS_LOG_UNCOND ("FWD-ZONE(app): [" << s_rXmin << "," << s_rYmin << "] - [" << s_rXmax << "," << s_rYmax
                    << "]  by s-id=" << s_s_id << "  (GR=[" << s_grXmin << "," << s_grYmin
                    << "]-[" << s_grXmax << "," << s_grYmax << "]"
                    << " , SRC-GRID=[" << gxmin << "," << gymin << "]-[" << gxmax << "," << gymax << "])");

    // Add-FWD の初期計算（ゾーン確定時点で一度実施） // Add-FWD: 初回算出
    ComputeAddFwdZone ();
    
    
    if (!s_addZoneGids.empty()) {
      std::ostringstream oss;
      oss << "ADD-FWD GIDs = {";
      bool first=true;
      for (auto g: s_addZoneGids) { if(!first) oss<<","; first=false; oss<<g; }
      oss << "}";
      NS_LOG_UNCOND (oss.str());
    } else {
      NS_LOG_UNCOND ("ADD-FWD GIDs = {} (NCN=" << s_NCN << ")");
    }
    
    s_zoneComputed = true; // Add-Forwarding: 追加ゾーン算出後に確定

    // ★ 共有ゾーンへ N0 で確定した値をコピー（以後は全送信ノードで共有）
    s_fwdRect = { s_rXmin, s_rYmin, s_rXmax, s_rYmax };
    s_addFwdGids.clear();
    for (auto g : s_addZoneGids) s_addFwdGids.insert(g);
    s_zoneInit = true;
  }
  
  
   // ---------- Add-FWD 再計算制御 ----------
  static void EnsureAddFwdComputed ()
  {
    // NCNが変わっていたら再計算 // Add-FWD: 版管理
    if (s_addFwdComputedForNCN != s_NCN) {
      ComputeAddFwdZone ();
      // ★ 再計算に伴い共有ADD-GIDsも更新（矩形は不変）
      if (s_zoneComputed) {
        s_addFwdGids.clear();
        for (auto g : s_addZoneGids) s_addFwdGids.insert(g);
        s_zoneInit = true;
      }
    }
  }

  // ---------- Add-FWD 集合の再計算本体 ----------
  static void ComputeAddFwdZone ()
  {
    s_addZoneGids.clear(); // まず空にする // Add-FWD: クリア

    if (s_srcGridId == 0) {
      s_addFwdComputedForNCN = s_NCN;
      NS_LOG_UNCOND ("ADD-FWD recompute skipped: srcGridId=0 (NCN=" << s_NCN << ")");
      return; 
    }

    // 仕様変更点：
    // 候補 = { sg+(Nx-1), sg-(Nx+1), sg-1, sg-Nx, sg-(Nx-1) } を生成し、
    // Geocast region の中心との距離が近い順（同距離なら gid 小）に並べて、
    // NCN に応じて先頭から k 件（k = clamp(NCN-3, 0..5)）を採用する。
    const int sg  = static_cast<int>(s_srcGridId);
    const int Nx  = static_cast<int>(s_gridNx);
    const int Ny  = static_cast<int>(s_gridNy);
    const int Ntot = Nx * Ny;

    auto inRange = [&](int gid)->bool {
      return (gid >= 1 && gid <= Ntot);
    };

    // Geocast region の中心
    const double grCx = 0.5 * (s_grXmin + s_grXmax);
    const double grCy = 0.5 * (s_grYmin + s_grYmax);

    // グリッド中心（gid:1-based）を返す補助
    auto gridCenter = [&](int gid)->std::pair<double,double> {
      int idx = gid - 1;
      double cx = s_GXmin[idx] + 0.5 * s_grid_d;
      double cy = s_GYmin[idx] + 0.5 * s_grid_d;
      return {cx, cy};
    };

    // 距離^2（実距離でなく平方距離でOK。比較順は同じ）
    auto dist2ToGR = [&](int gid)->double {
      auto [cx, cy] = gridCenter(gid);
      double dx = cx - grCx, dy = cy - grCy;
      return dx*dx + dy*dy;
    };

    // 候補の生成（順序は仕様に沿っているが、後で距離で並べ替える）
    int candRaw[5] = {
      sg + (Nx - 1),
      sg - (Nx + 1),
      sg - 1,
      sg - Nx,
      sg - (Nx - 1)
    };

    // 有効な候補を収集し、(distance2, gid) でソート
    std::vector<std::pair<double,int>> scored;
    scored.reserve(5);
    for (int i = 0; i < 5; ++i) {
      int gid = candRaw[i];
      if (!inRange(gid)) continue;
      scored.emplace_back(dist2ToGR(gid), gid);
    }
    std::sort(scored.begin(), scored.end(),
      [](const std::pair<double,int>& a, const std::pair<double,int>& b){
        if (std::abs(a.first - b.first) > 1e-12) return a.first < b.first; // 距離小さい順
        return a.second < b.second; // 同距離なら gid 小さい方
      });

    // 追加数 k = clamp(NCN-3, 0..5)
    int k = static_cast<int>(s_NCN) - 3;
    if (k < 0) k = 0;
    if (k > 5) k = 5;
    // 実際に存在する候補数にクリップ
    if (k > static_cast<int>(scored.size())) k = static_cast<int>(scored.size());

    for (int i = 0; i < k; ++i) {
      s_addZoneGids.insert(static_cast<uint32_t>(scored[i].second));
    }

    // ログ（確認用）
    std::ostringstream oss; oss << "ADD-FWD GIDs = {"; bool first=true;
    for (auto g: s_addZoneGids) { if(!first) oss<<","; first=false; oss<<g; }
    oss << "} (NCN=" << s_NCN << ", sg-id=" << s_srcGridId
        << ", Nx=" << s_gridNx << ", rule=distance-to-GR-center)";
    NS_LOG_UNCOND (oss.str());

    s_addFwdComputedForNCN = s_NCN;
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
  // ソケット初期化→属性の静的反映→FWDゾーン算出→g_id把握→
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

    // 共通属性を静的へ
    s_useGeocast = m_useGeocastAttr;
    s_grXmin = m_grXmin; s_grYmin = m_grYmin; s_grXmax = m_grXmax; s_grYmax = m_grYmax;
    s_s_id = m_s_id;
    s_NCN = m_ncn; // Add-Forwarding: NCNを共有へ反映
    NS_LOG_UNCOND ("[StartApplication] node=" << GetNode()->GetId()
                   << " m_ncn=" << m_ncn << " -> s_NCN=" << s_NCN
                   << " BCL=" << m_bcl); // Add-FWD + BCL: 起動時ログ
    
    s_addZoneGids.clear(); // Add-Forwarding: 起動時に追加ゾーン集合を一度クリア
    
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

    // 送信ノードは即時送信開始
    if (m_isSender) {
      static Ptr<UniformRandomVariable> suv = CreateObject<UniformRandomVariable>();
      double startJitter = suv->GetValue(0.0, 0.012) + 0.004 * std::max<int>(0, (int)m_senderIndex - 1);
      Simulator::Schedule(Seconds(startJitter), &SimpleFloodingApp::SendOne, this);
    }
  }

  // 停止時：すべてのスケジュールをにキャンセル
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
    double perTxJitter = uv->GetValue(0.0, 0.006);
    m_sendEvent = Simulator::Schedule (m_sendInterval + Seconds(perTxJitter), &SimpleFloodingApp::SendOne, this);
  }

  // GWがDATAを前方へ再送（0〜6msジッタ）
  void Forward (Ptr<Packet> p, const GeoHeader& hdr)
  {
    InetSocketAddress bcast (Ipv4Address ("255.255.255.255"), m_port);
    m_socket->SendTo (p, 0, bcast);
    m_forwardCount++; // BCL: このGWが転送したDATAパケット数をカウント
    NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                    << m_myAddress << " FWD(DATA) src=" << hdr.GetSrc ()
                    << " seq=" << hdr.GetSeq ()
                    << " fwdCount=" << m_forwardCount
                    << " (BCL=" << m_bcl << ")");
  }

  // ---------- FWDゾーン内判定 ----------
  // Geocast無効なら常に許可／有効なら FWD 矩形内のみ許可
  bool IsInsideFwdZone () const
  {
    if (!s_useGeocast) return true;
    if (!s_zoneComputed) MaybeComputeZone ();
    if (!s_zoneComputed) return false;
    Ptr<MobilityModel> mm = GetNode ()->GetObject<MobilityModel>(); if (!mm) return true;
    Vector pos = mm->GetPosition ();

    // 共有ゾーンが初期化済みなら N0 共有矩形で判定（全送信ノード共通）
    if (s_zoneInit) {
      return (pos.x >= s_fwdRect.x0 && pos.x <= s_fwdRect.x1 &&
              pos.y >= s_fwdRect.y0 && pos.y <= s_fwdRect.y1);
    }

    // フォールバック：従来の一時矩形
    return (pos.x >= s_rXmin && pos.x <= s_rXmax &&
            pos.y >= s_rYmin && pos.y <= s_rYmax);
  }
  
  // ---------- Add-Forwarding ゾーン内判定 ----------
  bool IsInsideAddFwdZone () const
  {
    if (!s_useGeocast) return false;  // Geocast無効なら追加ゾーンも無効
    if (!s_zoneComputed) MaybeComputeZone ();
    if (!s_zoneComputed) return false;
    EnsureAddFwdComputed ();          // NCN変化時は再計算

    // 共有ゾーンが初期化済みなら N0 共有 ADD-GIDs を使用
    if (s_zoneInit) {
      return (s_addFwdGids.find(m_g_id) != s_addFwdGids.end());
    }

    // フォールバック：従来の追加集合
    return (s_addZoneGids.find(m_g_id) != s_addZoneGids.end());
  }

  // ---------- Geocast region 内/隣接グリッド判定 ----------
  // 7-4: 自身が Geocast region 内かどうか
  bool IsInsideGeocastRegion () const
  {
    if (!s_useGeocast) return false;
    Ptr<MobilityModel> mm = GetNode ()->GetObject<MobilityModel>(); if (!mm) return false;
    Vector pos = mm->GetPosition ();
    return (pos.x >= s_grXmin && pos.x <= s_grXmax &&
            pos.y >= s_grYmin && pos.y <= s_grYmax);
  }

  // 7-5 用: 自身の所属グリッドが Geocast region「隣接グリッド」かどうか
  // （Geocast 矩形をグリッドサイズ分だけ膨らませた矩形と自グリッドが交差し、
  //   かつ元のGeocast 矩形とは交差しないセルを「隣接グリッド」とみなす）
  bool IsNeighborOfGeocastRegion () const
  {
    if (!s_useGeocast || !s_gridDefined) {
      // Geocast 無効時は BCL を効かせないため true 扱い
      return true;
    }
    // 自グリッド矩形とGeocast region矩形の交差判定
    bool intersectsGR =
      (m_gxmax > s_grXmin && m_gxmin < s_grXmax &&
       m_gymax > s_grYmin && m_gymin < s_grYmax);

    // グリッド一辺分だけ拡大した矩形との交差判定
    double exXmin = s_grXmin - s_grid_d;
    double exXmax = s_grXmax + s_grid_d;
    double exYmin = s_grYmin - s_grid_d;
    double exYmax = s_grYmax + s_grid_d;

    bool intersectsExpanded =
      (m_gxmax > exXmin && m_gxmin < exXmax &&
       m_gymax > exYmin && m_gymin < exYmax);

    // 元GRには含まれず、拡大矩形とは交差していれば「隣接グリッド」
    return (intersectsExpanded && !intersectsGR);
  }
  
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

      // (src,seq,type)で重複排除
      uint64_t key = (static_cast<uint64_t>(hdr.GetSrc().Get()) << 32)
                   | (static_cast<uint64_t>(hdr.GetSeq()) ^ (static_cast<uint64_t>(hdr.GetType()) << 24));

      // DATA以外は従来どおり「先に」重複排除する（DATAは後段で条件に応じて登録）
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
        NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                        << m_myAddress << " RX(GATE)"
                        << " prev=" << prevHop
                        << " g_id=" << hdr.GetGridId ());
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
        // DATA は GW かつ FWDゾーン内のみ前方転送
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

        if (directFromSrc) {
          // 送信元からの「直送」：送信元と同じグリッドにいる場合のみ転送
          uint32_t sgid = 0;
          if (!FindSenderGidByIp(hdr.GetSrc(), sgid)) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP cannot-resolve-src-grid");
            break; // 送信元g_idが不明なら保守的にDROP
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
          // 7-4: 自身がGeocast region内であれば転送しない
          if (IsInsideGeocastRegion ()) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP inside-geocast-region");
            break;
          }
          // 7-5: 自身がGeocast regionの隣接グリッドでなく、
          //      かつ今まで転送した数がBCL以上なら転送しない
          if (m_bcl > 0 && !IsNeighborOfGeocastRegion () && m_forwardCount >= m_bcl) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP BCL-limit"
                            << " fwdCount=" << m_forwardCount
                            << " BCL=" << m_bcl);
            break;
          }

          // 7-6: 上記条件に当てはまらなければ転送（ブロードキャスト）
          static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
          double jitter = uv->GetValue (0.0, 0.006);
          Ptr<Packet> cp = p->Copy();
          GeoHeader hdrCopy = hdr;
          Simulator::Schedule (Seconds (jitter), [this, cp, hdrCopy]() {
            this->Forward (cp, hdrCopy);
          });
        } else {
          // 他GWからの転送：従来どおり「一度だけ」転送
          // 7-3: 2回目以降に受信したパケットであれば転送しない（重複判定）
          if (!m_seen.insert (key).second) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP dup-data");
            break;
          }
          // 7-4: 自身がGeocast region内であれば転送しない
          if (IsInsideGeocastRegion ()) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP inside-geocast-region");
            break;
          }
          // 7-5: 自身がGeocast regionの隣接グリッドでなく、
          //      かつ今まで転送した数がBCL以上なら転送しない
          if (m_bcl > 0 && !IsNeighborOfGeocastRegion () && m_forwardCount >= m_bcl) {
            NS_LOG_UNCOND ("[" << Simulator::Now ().GetSeconds () << "s] "
                            << m_myAddress << " DROP BCL-limit"
                            << " fwdCount=" << m_forwardCount
                            << " BCL=" << m_bcl);
            break;
          }

          // 7-6: 上記条件に当てはまらなければ転送（ブロードキャスト）
          static Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
          double jitter = uv->GetValue (0.0, 0.006);
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

  // アプリ属性（Geocast関連）
  bool   m_useGeocastAttr{false};
  double m_grXmin{0.0}, m_grYmin{0.0};
  double m_grXmax{0.0}, m_grYmax{0.0};
  uint32_t m_s_id{1};
  uint32_t m_ncn{0};          // NCN
  uint32_t m_bcl{0};          // BCL: ゲートウェイごとの転送制限数(0なら制限なし)
  uint32_t m_forwardCount{0}; // BCL: このGWが今までに転送したDATAパケット数

  // グリッド越え検知（CourseChange + 50ms ポーリング）
  EventId m_gridCheckEvent;
  Time    m_gridCheckInterval{MilliSeconds(50)};
  Ptr<MobilityModel> m_mobility;

  // GATE/BID 制御
  EventId m_gateEvent;
  EventId m_gateWatchdogEvent;
  EventId m_bidAckEvent;                 // BID結果待ちタイマーとして再利用
  Time    m_gateInterval{MilliSeconds(200)};
  Time    m_gateTimeout{MilliSeconds(600)};
  Time    m_bidResponseWait{MilliSeconds(200)}; // BIDの応答待ち時間（デフォルトは gateInterval）
  Time    m_lastGateRx{Seconds(0)};
  Time    m_lastBidTx{Seconds(0)};      // 直近のBID送信時刻

  // 共有（FWDゾーン）
  static bool   s_useGeocast;
  static bool   s_zoneComputed;
  static double s_grXmin, s_grYmin, s_grYmax, s_grXmax;
  static uint32_t s_s_id;
  static double s_rXmin, s_rYmin, s_rXmax, s_rYmax;
  static std::vector< Ptr<Node> > s_senders;
  static uint32_t s_NCN;
  static uint32_t s_srcGridId;
  static std::set<uint32_t> s_addZoneGids;
  static uint32_t s_addFwdComputedForNCN; // Add-FWD: 再計算版

  // ★ 共有ゾーン（N0が一度だけ確定し、全送信ノードで共有）
  struct Rect { double x0, y0, x1, y1; };
  static bool s_zoneInit;                       // 共有ゾーン初期化済み
  static Rect s_fwdRect;                        // 共有Forwarding矩形
  static std::unordered_set<uint32_t> s_addFwdGids; // 共有ADD-FWD GIDs
  static Ipv4Address s_primarySrc;              // 参考用（未使用でも保持）
  // 共有（グリッド境界）
  static bool   s_gridDefined;
  static double s_gridOriginX, s_gridOriginY;
  static double s_grid_d;
  static uint32_t s_gridNx, s_gridNy;
  static std::vector<double> s_GXmin, s_GXmax, s_GYmin, s_GYmax;
};

// ====== 静的メンバ定義 ======
// Geocast/FWD ゾーン関連の共有状態
bool   SimpleFloodingApp::s_useGeocast   = false;
bool   SimpleFloodingApp::s_zoneComputed = false;
double SimpleFloodingApp::s_grXmin = 0.0, SimpleFloodingApp::s_grYmin = 0.0;
double SimpleFloodingApp::s_grXmax = 0.0, SimpleFloodingApp::s_grYmax = 0.0;
uint32_t SimpleFloodingApp::s_s_id = 1;

bool SimpleFloodingApp::s_zoneInit = false;  // ★ 共有ゾーンの初期化フラグ
SimpleFloodingApp::Rect SimpleFloodingApp::s_fwdRect {0,0,0,0}; // ★ 共有FWD矩形
std::unordered_set<uint32_t> SimpleFloodingApp::s_addFwdGids;   // ★ 共有ADD-GIDs
Ipv4Address SimpleFloodingApp::s_primarySrc = Ipv4Address("0.0.0.0"); // ★ 参考保持

uint32_t SimpleFloodingApp::s_NCN = 0;            // Add-Forwarding: NCN初期値
uint32_t SimpleFloodingApp::s_srcGridId = 0;      // Add-Forwarding: 送信元g_id
std::set<uint32_t> SimpleFloodingApp::s_addZoneGids; // Add-Forwarding: 追加g_id集合
uint32_t SimpleFloodingApp::s_addFwdComputedForNCN = std::numeric_limits<uint32_t>::max(); // Add-FWD: 未計算を示す

double SimpleFloodingApp::s_rXmin = 0.0, SimpleFloodingApp::s_rYmin = 0.0;
double SimpleFloodingApp::s_rXmax = 0.0, SimpleFloodingApp::s_rYmax = 0.0;
std::vector< Ptr<Node> > SimpleFloodingApp::s_senders;

// グリッド境界テーブル
bool   SimpleFloodingApp::s_gridDefined = false;
double SimpleFloodingApp::s_gridOriginX = 0.0, SimpleFloodingApp::s_gridOriginY = 0.0;
double SimpleFloodingApp::s_grid_d = 1.0;
uint32_t SimpleFloodingApp::s_gridNx = 0, SimpleFloodingApp::s_gridNy = 0;
std::vector<double> SimpleFloodingApp::s_GXmin, SimpleFloodingApp::s_GXmax;
std::vector<double> SimpleFloodingApp::s_GYmin, SimpleFloodingApp::s_GYmax;

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
  uint32_t NCN = 0;           // NCN
  uint32_t BCL = 0;           // BCL: ゲートウェイの転送制限数(0=制限なし)

  bool   useGeocast = false;  // FWDゾーンを使うか
  double grXmin = 0.0, grYmin = 0.0, grXmax = 0.0, grYmax = 0.0;

  // ランダム配置パラメータ（非送信ノードの数と矩形領域）
  uint32_t nodeCount = 8;                // 送信以外のノード数
  double nodeXmin = -50.0, nodeYmin = -50.0, nodeXmax = 50.0, nodeYmax = 50.0;

  // --- CLI 引数の定義と読み込み ---
  CommandLine cmd(__FILE__);
  cmd.AddValue ("nNodes",    "Number of nodes", nNodes);
  cmd.AddValue ("stopTime",  "Simulation stop time (s)", stopTime);
  cmd.AddValue ("interval",  "Source send interval (s)", interval);
  cmd.AddValue ("nSources",  "Number of source nodes from index 0..", nSources);
  cmd.AddValue ("srcId",     "Source node id (1-based) used for FWD-zone calculation", srcId);
  cmd.AddValue ("useGeocast","Enable forwarding zone (0/1)", useGeocast);
  cmd.AddValue ("grXmin",    "Geocast region xmin", grXmin);
  cmd.AddValue ("grYmin",    "Geocast region ymin", grYmin);
  cmd.AddValue ("grXmax",    "Geocast region xmax", grXmax);
  cmd.AddValue ("grYmax",    "Geocast region ymax", grYmax);
  cmd.AddValue ("srcCoords", "Comma-separated list of source coordinates: x:y,x:y,...", srcCoordsStr);

  cmd.AddValue ("nodeCount", "Number of non-sender nodes to place randomly (overrides nNodes)", nodeCount);
  cmd.AddValue ("nodeXmin",  "Random placement Xmin", nodeXmin);
  cmd.AddValue ("nodeYmin",  "Random placement Ymin", nodeYmin);
  cmd.AddValue ("nodeXmax",  "Random placement Xmax", nodeXmax);
  cmd.AddValue ("nodeYmax",  "Random placement Ymax", nodeYmax);
  
  cmd.AddValue ("NCN",       "Number of coded packets (controls Add-Forwarding zone)", NCN); // Add-Forwarding: 受け取り
  cmd.AddValue ("BCL",       "Forwarding limit per gateway node (0 = no limit)", BCL);       // BCL: CLIから転送制限数を指定

  cmd.Parse (argc, argv);

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
  const double gOrgX = -50.0, gOrgY = -50.0;
  const double gEndX = 350.0,  gEndY = 350.0;
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
  
  
   //  /*
    // --- 非送信ノード：格子状に固定配置（デバッグ用） ---
  {
    const uint32_t N = otherNodes.GetN();

    const double startX = -25.0;  // 格子の起点X
    const double startY = -25.0;  // 格子の起点Y
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
 // */
  

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

  // --- アプリ配備：送信ノード（Start=1.0s, 即DATA開始） ---
  for (uint32_t i = 0; i < srcNodes.GetN (); ++i) {
    Ptr<SimpleFloodingApp> app = CreateObject<SimpleFloodingApp>();
    app->SetAttribute ("SendInterval", TimeValue (Seconds (interval)));
    app->SetAttribute ("IsSender", BooleanValue (true));
    app->SetAttribute ("SenderIndex", UintegerValue (i + 1));
    app->SetAttribute ("UseGeocast", BooleanValue (useGeocast));
    app->SetAttribute ("grXmin", DoubleValue (grXmin));
    app->SetAttribute ("grYmin", DoubleValue (grYmin));
    app->SetAttribute ("grXmax", DoubleValue (grXmax));
    app->SetAttribute ("grYmax", DoubleValue (grYmax));
    app->SetAttribute ("NCN",   UintegerValue (NCN));
    app->SetAttribute ("BCL",   UintegerValue (BCL));   // BCL: 送信ノード側にも同じ制限値を配布
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
    app->SetAttribute ("UseGeocast", BooleanValue (useGeocast));
    app->SetAttribute ("grXmin", DoubleValue (grXmin));
    app->SetAttribute ("grYmin", DoubleValue (grYmin));
    app->SetAttribute ("grXmax", DoubleValue (grXmax));
    app->SetAttribute ("grYmax", DoubleValue (grYmax));
    app->SetAttribute ("NCN",   UintegerValue (NCN));
    app->SetAttribute ("BCL",   UintegerValue (BCL));   // BCL: 非送信ノード(GW候補)にも制限値を配布
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

1) GeoHeader（パケットのアプリ層ヘッダ）

      m_type : MsgType(uint8_t)
      メッセージ種別（DATA, SELECT, RESELECT, GATE, BID）
      
      m_src : Ipv4Address
      送信元IP（重複排除や直送判定に使用）
      
      m_seq : uint32_t
      シーケンス番号（重複排除、BCLカウントキー）
      
      m_g_id : uint32_t
      関連グリッドID（SELECT/GATE/BID/RESELECTで使用）
      
      m_round : uint32_t
      SELECTラウンド番号（20ms後の確定で使用）
      
      m_distance : float
      セル中心までの距離（SELECTの優先度比較用）


2) SimpleFloodingApp：インスタンス変数（m_*）
基本I/O

      m_socket : Ptr<Socket> … UDPソケット
      
      m_port : uint16_t … ブロードキャストUDPポート（既定9999）
      
      m_myAddress : Ipv4Address … 自ノードのIPv4
      
  送信・アプリ動作
      
      m_sendInterval : Time … 送信周期（送信ノード用）
      
      m_isSender : bool … 送信ノードかどうか
      
      m_senderIndex : uint32_t … 送信ノードの通し番号（1始まり）
      
      m_nextSeq : uint32_t … DATA用シーケンスの連番
      
  GW選出（SELECT/BID/GATE/RESELECT）
      
      m_selectRound : uint32_t … 現行のSELECTラウンド番号
      
      m_selectSeq : uint32_t … 選出系メッセージのseq
      
      m_sel : map<uint64_t, SelState> … (g_id,round)ごとの最良候補（距離/IP）を保持
      
      m_selFinalize : map<uint64_t, EventId> … 20ms後の確定イベント
      
      m_isGateway : bool … 自ノードがGWか
      
      m_lastGateRx : Time … 直近のGATE受信時刻（GW生存確認）
      
      m_lastBidTx : Time … 直近のBID送信時刻（応答待ちの基準）
      
  受信・重複抑止・BCL
      
      m_seen : set<uint64_t> … ノード内の重複抑止キー集合（(src,seq,type)）
      
      m_bcl : uint32_t … 非隣接グリッドでの転送上限（0=無制限）
      
      m_bclCount : unordered_map<uint32_t,uint32_t> … seqごとの転送回数（非隣接のみカウント）
      
  自セル情報（位置・境界）
      
      m_g_id : uint32_t … 自ノードの所属グリッドID（1始まり）
      
      m_gxmin,m_gymin,m_gxmax,m_gymax : double … 自セル境界
      
      m_mobility : Ptr<MobilityModel> … 位置取得・CourseChangeフック
      
  Geocast属性（ノード単位で受け取り→静的へ反映）
      
      m_useGeocastAttr : bool … Geocast/Forwarding zoneの有効化
      
      m_grXmin,m_grYmin,m_grXmax,m_grYmax : double … Geocast region矩形
      
      m_s_id : uint32_t … Forwarding zoneを定める基準送信ノードID（1始まり）
      
      m_ncn : uint32_t … Add-Forwarding用のNCN（ローカル保持、起動時に共有へ渡す）
      
  タイマ／イベント
      
      m_sendEvent : EventId … 次回送信（DATA）の予約
      
      m_gateEvent : EventId … 周期GATE送信
      
      m_gateWatchdogEvent : EventId … GATE監視（BIDトリガ）
      
      m_bidAckEvent : EventId … BID応答待ちタイムアウト
      
      m_gridCheckEvent : EventId … グリッド越えポーリング（50ms）
      
   時間パラメータ
      
      m_gateInterval : Time … GATE周期（200ms）
      
      m_gateTimeout : Time … GATE未受信でBID発火の閾値（600ms）
      
      m_bidResponseWait : Time … BID後の応答待ち（200ms）
      
      m_gridCheckInterval : Time … グリッドチェック周期（50ms）



3) SimpleFloodingApp：静的変数（s_*）— 共有状態

   Geocast/Forwarding zone
      
      s_useGeocast : bool … Geocast全体の有効/無効
      
      s_zoneComputed : bool … Forwarding zoneが一度でも確定済みか
      
      s_grXmin,s_grYmin,s_grXmax,s_grYmax : double … Geocast region矩形（共有）
      
      s_s_id : uint32_t … 基準送信ノードID（共有）
      
      s_rXmin,s_rYmin,s_rXmax,s_rYmax : double … 旧式フォールバック用の矩形
      
      s_zoneInit : bool … 共有FWD矩形・ADD集合が初期化済みか
      
      s_fwdRect : Rect{ x0,y0,x1,y1 } … 共有Forwarding矩形（全ノードで共通）
      
      s_senders : vector<Ptr<Node>> … 送信ノード参照（送信元g_id逆引きなど）
      
      s_primarySrc : Ipv4Address … 参考保持（未使用でも保管）
      
   Add-Forwarding（NCN制御）
      
      s_NCN : uint32_t … 共有NCN
      
      s_srcGridId : uint32_t … 送信元の所属グリッドID
      
      s_addZoneGids : set<uint32_t> … 再計算用（非共有の一時集合／フォールバック）
      
      s_addFwdComputedForNCN : uint32_t … どのNCNでADD計算済みかの印
      
      s_addFwdGids : unordered_set<uint32_t> … 共有ADD-FWD許可グリッド集合
      
   Geocast region と隣接セル
      
      s_regionGids : set<uint32_t> … regionに重なるセルの集合
      
      s_adjacentGids : set<uint32_t> … region外周1セル（斜め含む）の集合
      
      s_adjacentFwdKeys : unordered_set<uint64_t> … **隣接セル群で“一度だけ転送”**するための共有キー（(src,seq)）
      
   グリッド定義（空間分割）
      
      s_gridDefined : bool … グリッドが定義済みか
      
      s_gridOriginX,s_gridOriginY : double … グリッド原点座標
      
      s_grid_d : double … 一辺長
      
      s_gridNx,s_gridNy : uint32_t … X/Y方向のセル数
      
      s_GXmin,s_GXmax,s_GYmin,s_GYmax : vector<double> … 各セル境界テーブル



4) 補助構造体

      SelState { initialized, bestDist, bestAddr }
      同一(g_id, round)内で最良候補の状態。
      
      Rect { x0,y0,x1,y1 }
      Forwarding zone共有矩形の入れ物。

5) main() のローカル変数（CLIで設定可能）

      実験規模・時間
      
      　nNodes, stopTime, interval
      
      送信ノード
      
     　 nSources, srcId, srcCoordsStr（「x:y,x:y,…」）
      
     　 Geocast
      
     　 useGeocast, grXmin, grYmin, grXmax, grYmax
      
      　Add-FWD / BCL
      
     　 NCN, BCL

      非送信ノードの配置
      
     　 nodeCount, nodeXmin, nodeYmin, nodeXmax, nodeYmax

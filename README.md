simple-flooding-kansei.ccは、既存のgeogridを作成したコード

simple-flooding-NCN.ccは、既存のgeogridにForwarding zoneの拡張機能(送信ノード周り)を追加したもの

simple-flooding-BCL-zentaiは、simple-flooding-NCN.ccにゲートウェイノードの転送制限機能を追加したもの(Geocast region周りはまだ)

simple-flooding-BCL-rinsetsuは、simple-flooding-BCL-zentaiにGeocast region周りにの転送制限機能を追加したもの



実行する際は、.ccファイルをscratchに入れてください。

以下は実行例(-kansei.ccはNCN,BCLは使えない、-NCN.ccはBCLは使えないです。)
./ns3 run "scratch/simple-flooding --stopTime=2 --nSources=3 --nodeCount=4 --srcCoords=0:0,5:5,10:10 --grXmin=100 --grYmin=100 --grXmax=150 --grYmax=150 --useGeocast=1 --nodeXmin=0 --nodeYmin=0 --nodeXmax=50 --nodeYmax=50 --NCN=0 --BCL=0"

stopTimeはシミュレーション時間(秒)
nSourcesは送信ノードの数
nodeCountは送信ノード以外のノード数
srcCoordsは送信ノードの座標(srcCoords=1個目のx座標:1個目のy座標,2個目のx座標:2個目のy座標,・・・で指定)
grXmin,grYmin,grXmax,grYmaxは、Geocast regionの範囲をX,Y座標の最小・最大値で指定
useGeocastは、使うGeocast regionの数で、今のところ「1」のみ対応
nodeXmin,nodeYmin,nodeXmax,nodeYmaxは、送信ノード以外のノードをランダムに配置する範囲をX,Y座標の最小・最大値で指定


NCNは、符号化パケットの作成数を表しており、今のところForwarding zoneの拡張にのみ関与
    0<=NCN<=3の時、Add-Forwarding zoneの追加はなし。元々のForwarding zoneのみ
    NCN>=4の時、グリッド1個をAdd-Forwarding zoneに追加
    NCN>=5の時、グリッド2個をAdd-Forwarding zoneに追加
    NCN>=6の時、グリッド3個をAdd-Forwarding zoneに追加
    NCN>=7の時、グリッド4個をAdd-Forwarding zoneに追加
    NCN>=8の時、グリッド5個をAdd-Forwarding zoneに追加


BCLは、ゲートウェイノードの転送制限数を表しており、この数までしかパケットを転送しない
　　　　BCL=0ならば転送制限なし

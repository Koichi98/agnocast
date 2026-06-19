# Generic R2A Service Bridge

プラグイン不要（generic）で R2A サービスブリッジを提供するための実装まとめ。
ブランチ: `feat/service-wire-out-of-band-header`

## 背景・目的

- 従来、サービスブリッジは **型固有プラグイン専用**で、generic フォールバックが無かった（pub/sub には `GenericPublisher`/`GenericSubscription` による generic フォールバックがある）。
- そのため、agnocast ノードが提供するサービス（例: parameter service = `rcl_interfaces/srv/*`）へ ROS 2 クライアントからアクセスするには、`ros2 agnocast generate-bridge-plugins` で型固有プラグインを生成・ビルドする必要があった。
- 目的: **プラグイン無しで R2A サービスブリッジを成立させる**。方向は R2A（ROS 2 client → agnocast service server）のみ。A2R は対象外。

### 制約

- R2A の ROS 側には「任意型を受ける service サーバ」= `rclcpp::GenericService` が必要だが、これは **Kilted/Rolling 以降**にしか存在しない（PR ros2/rclcpp#2617, 2024-09）。Humble / Jazzy には無い。
- 本実装の対象は **ROS 2 Humble**。よって `GenericService` を vendoring（移植）した。

## アーキテクチャ / データフロー

```text
ROS 2 client
  → [GenericService]              (vendored, 型消去 ROS service サーバ)
  → rmw_serialize(request)
  → [GenericPublisher::publish_service_request]   request topic へゼロコピー発行
  → agnocast service server（例: parameter service）が処理・応答
  → response topic
  → [GenericSubscription]          応答 payload を serialize して受信
  → rmw_deserialize(response)
  → [GenericService::send_response] → ROS 2 client
```

- ブリッジ本体は bridge daemon（IPC namespace ごとの常駐プロセス）内で生成される。
- プラグインが見つからない場合に `create_r2a_service_bridge` が `create_r2a_service_bridge_generic` へ**自動フォールバック**する（manager 側の改修は不要）。

## 設計のポイント

### 1. Out-of-band な service wire レイアウト

agnocast のサービスは内部的に **request topic / response topic 上の pub/sub** で実装され、相関情報（`_sequence_number`、応答ルーティング用の client `_node_name`）をメッセージに付随させる。

従来は継承＋`std::string` で実装していたが、generic 側が型消去で wire を構築・解釈できるよう、次の **合成（composition）+ 固定長**レイアウトに変更した（`agnocast_service_wire.hpp`）。

```cpp
template <typename Payload>
struct ServiceRequestWrapper {
  Payload payload;                      // offset 0（ユーザ型がそのまま先頭に来る）
  int64_t _sequence_number;             // offset = align_up(sizeof(Payload), 8)
  char    _node_name[NODE_NAME_BUFFER_SIZE];  // 固定長・フラット
};
template <typename Payload>
struct ServiceResponseWrapper {
  Payload payload;                      // offset 0
  int64_t _sequence_number;
};
```

- **payload を offset 0 に置く**ことで、ユーザ向け `ipc_shared_ptr<Request>` と発行/解放アドレスが一致し、`ipc_shared_ptr` の aliasing で安全に出し入れできる。
- ヘッダのオフセットが `align_up(sizeof(payload), 8)` で **generic に計算可能**（継承の tail-padding 依存の脆さを回避）。ROS メッセージのアライメントは最大 8 なのでこの計算が C++ レイアウトと一致する。
- `std::string`→`char[N]` でフラット POD 化し、generic 構築・自己完結バイト列を可能にした。

**ユーザ API は不変**: `borrow_loaned_request` / `async_send_request` / service callback / `send_response` / `borrow_loaned_response` のシグネチャは `ServiceT::Request`/`ServiceT::Response` のまま。`_sequence_number`/`_node_name`/wrapper は完全に内部実装。ただしワイヤ互換は破壊するため、サービスを使う全ノードの一斉リビルドが必要。

### 2. vendored GenericService（Humble 向け）

`rclcpp::GenericService`（Rolling）を agnocast に移植（`agnocast::bridge::GenericService`）。

- `rclcpp::ServiceBase` を継承し、`rcl_service_init` / `rcl_take_request_with_info` / `rcl_send_response` を使用（いずれも Humble に存在）。
- Humble には `get_service_typesupport_handle` が無いため、生成シンボル（`rosidl_typesupport_cpp__get_service_type_support_handle__<pkg>__srv__<Type>` 等）を**ロード済みライブラリから直接解決**して内蔵。
- ブリッジ用途に絞り、**deferred 応答 callback のみ**実装（リクエストを受け取り、応答は agnocast から戻ってきた後で `send_response` を呼ぶ）。
- Kilted/Rolling 以降では標準の `rclcpp::GenericService` を使えばよい（この vendoring は不要）。

### 3. 1-in-flight 相関

ブリッジはリクエストを**1件ずつ（in-flight 1）**転送する。応答トピック上の単一応答が pending リクエストに一意に対応するため、**応答の seqno をワイヤから読み戻す必要がなく、`GenericSubscription` を無改造で利用**できる。サービスは低レートなので妥当な v1。

- QoS マッチ・ルーティングはトピック名ベース（型名レジストリはユーザ空間の補助で、サービスは空型名でスキップ）。

## 変更ファイル

新規:

- `src/agnocastlib/include/agnocast/agnocast_service_wire.hpp` — wire レイアウト + generic オフセットヘルパ
- `src/agnocastlib/include/agnocast/bridge/performance/agnocast_generic_service.hpp`
- `src/agnocastlib/src/bridge/performance/agnocast_generic_service.cpp`

変更:

- `agnocast_client.hpp` / `agnocast_service.hpp` — wire リファクタ（合成 wrapper 化）
- `agnocast_publisher.hpp` / `agnocast_publisher.cpp` — `GenericPublisher::publish_service_request`
- `bridge/performance/agnocast_performance_bridge_loader.{hpp,cpp}` — `create_r2a_service_bridge_generic` と generic フォールバック
- `CMakeLists.txt` — vendored ソース追加

## 検証

- `agnocastlib` グリーンビルド（Humble）。
- E2E（`agnocast_sample_interfaces/srv/SumIntArray`、`agnocast_sample_application` の `minimal_server`）:
  - **Generic 経路**（プラグイン `.so` を退避して generic を強制）: `ros2 service call /srv/sum_int_array "{data: [10,20,30]}"` → `sum=60` **PASS**。
  - **Typed 経路（回帰）**: 同じく `sum=60` **PASS**。wire 変更が既存 typed サービスブリッジを壊していないことを確認。

> 注: bridge daemon は IPC namespace ごとに常駐し再ビルドを自動で拾わない。ランタイム検証時は対象 namespace の既存 daemon を停止してから実行すること。

## 制約・今後

- **1-in-flight** によるスループット制約（並行処理は seqno 相関で拡張可能だが、`GenericSubscription` の改造が必要）。
- **A2R サービスブリッジは未対応**（スコープ外）。
- 複数の srv 型・並行呼び出しでの追加検証は未実施。
- Kilted/Rolling をターゲットにするなら vendored GenericService は不要（標準の `rclcpp::GenericService`/`GenericClient` を使用）。

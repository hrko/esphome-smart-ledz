# esphome-smart-ledz

Smart LEDZ デバイスを ESPHome から操作するための外部コンポーネントです。  
ESP32 を Smart LEDZ Mesh のブリッジとして動作させ、Home Assistant から通常の `light` エンティティとして扱えます。

## 対応環境

- ESPHome + ESP32
- `esp-idf` フレームワーク（推奨）
- Smart LEDZ の `mesh_name` / `mesh_password`

## 前提条件

- 照明器具が SmartLEDZ Fit アプリに追加済みであること
- 追加済みの照明器具/グループ情報を含むバックアップ JSON を取得できること

## SmartLEDZ Fit で JSON をエクスポート

1. SmartLEDZ Fit アプリを開く
2. 「設定」を開く
3. 「バックアップ・リストア」を開く
4. 「バックアップ」を選ぶ
5. 「ファイルにバックアップ」を選ぶ
6. 保存された JSON ファイルを ESPHome 設定を編集する端末へ転送する

## クイックスタート

1. SmartLEDZ Fit からエクスポートした JSON を用意する（上記手順）
2. `smartledz-export-spa` を GitHub Pages で開く  
   `https://<GitHubユーザー名>.github.io/esphome-smart-ledz/`  
   例: `https://hrko.github.io/esphome-smart-ledz/`
3. SmartLEDZ Fit のエクスポート JSON を読み込む
4. 変換対象（デバイス/グループ）を選ぶ
5. 出力された `ESPHome YAML` と `secrets.yaml` をコピーする
6. あなたの ESPHome 設定（`esphome` / `esp32` / `wifi` / `api` / `ota` など）に貼り付けて、`uvx esphome compile ...` で確認する

このツールは静的ページです。読み込んだ JSON はブラウザ内だけで処理され、サーバーには送信されません。

機密性を重視する場合は、`tools/smartledz-export-spa/` の `index.html` / `app.js` / `styles.css` をローカルで開いて利用できます。  
この方法ならネットワークを切った状態（機内モード相当）でも実行できます。

## このコンポーネントでできること

- SmartLEDZ Fit に登録済みの照明器具/グループを `light` エンティティとして ESPHome から制御
- `device_type` に応じた ON/OFF、明るさ、色温度、RGB 制御
- `target` を使った個別アドレス/グループアドレス宛ての操作
- ポーリングと通知による状態反映（明るさ/色/電源状態）

## このコンポーネントで扱わないこと

- 新しい照明器具の追加
- グループの作成
- グループへの照明器具の追加・削除
- SmartLEDZ Fit 側の構成管理（上記のような管理操作全般）

照明器具やグループ構成を変更した場合は、SmartLEDZ Fit 側で更新したあとに JSON を再エクスポートし、ESPHome YAML も更新してください。

## 設定リファレンス

### `smart_ledz`

- 必須: `id`, `ble_client_id`, `mesh_name`, `mesh_password`
- 任意: `vendor_id`（デフォルト `0x0211`）
- 任意: `poll_interval`（デフォルト `2s`）
- 任意: `tx_interval`（デフォルト `120ms`）
- 任意: `power_on_settle`（デフォルト `400ms`）

### `light` (`platform: smart_ledz`)

- 必須: `id`, `name`, `smart_ledz_id`, `target`, `device_type`
- 任意: `ct_duv`（デフォルト `0`、`-6/-3/0/3/6` のみ）
- 任意: `ignore_transition`（デフォルト `true`）

`device_type` は次のいずれかを指定します。

- `dimmable`: 調光のみ
- `tunable`: 色温度
- `synca`: RGB + 色温度

## `target` の指定

- 個別デバイス: 例 `0x0001`
- グループ: 例 `0x8001`

SmartLEDZ Fit のエクスポートデータを使う場合、グループの `target` は通常 `0x8000 | グループ下位バイト` です。

## 設定値の取得と YAML 生成

生成ツールは次の 2 つの使い方に対応しています。

- GitHub Pages: `https://<GitHubユーザー名>.github.io/esphome-smart-ledz/`
- ローカル実行: `tools/smartledz-export-spa/index.html`

`mesh_name` / `mesh_password` は `!secret` 化して運用することを推奨します。

## 動作確認

ローカルでの最小確認:

```bash
uvx esphome compile example_smart_ledz.yaml
```

## トラブルシューティング

- 接続できない: `ble_client.mac_address` が Mesh 内デバイスの MAC か、`mesh_name` / `mesh_password` が一致しているかを確認してください。
- 反応が遅い/取りこぼす: `tx_interval` を少し大きく、`power_on_settle` を少し長めに調整してください。
- トランジション中に意図しない挙動になる: `ignore_transition` を `true`/`false` で切り替えて比較してください。

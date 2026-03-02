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
- 任意: `ct_duv`（デフォルト `0`、`-6.0〜6.0`）
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

## 免責事項 (Disclaimer)

本プロジェクト（esphome-smart-ledz）は、個人的な技術研究およびオープンソースのホームオートメーションシステム（Home Assistant等）との相互運用性を確保する目的で作成された非公式のプロジェクトです。

本ソフトウェアをご利用になる場合は、以下の事項に同意したものとみなします。

1. **非公式プロジェクト**
   本プロジェクトは、株式会社遠藤照明（Endo Lighting Corp.）および関連企業とは一切関係ありません。**本ソフトウェアの利用や不具合に関して、メーカー公式のサポート窓口へ問い合わせることは絶対におやめください。**
2. **自己責任と無保証（AS IS）**
   本ソフトウェアは「現状有姿（AS IS）」で提供されます。本ソフトウェアを使用したことによる照明器具本体や周辺機器の故障、不具合、損害（火災や事故を含む）について、作者は一切の責任を負いません。また、本ソフトウェアを使用することでメーカーの正規保証の対象外となる可能性があります。すべて**自己責任**でご利用ください。
3. **知的財産権の尊重**
   本プロジェクトは、システム間の連携を目的としたプロトコル解析に基づいており、メーカーの特許権や著作権等の知的財産権を侵害する意図はありません。
4. **仕様変更による動作停止のリスク**
   照明器具側のファームウェアアップデートや公式アプリの仕様変更により、予告なく本ソフトウェアが動作しなくなる可能性があります。継続的な動作保証やサポートは提供されません。

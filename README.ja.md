# esphome-smart-ledz

Smart LEDZ デバイスを ESPHome から操作するための外部コンポーネントです。  
ESP32 を Smart LEDZ Mesh のブリッジとして動作させ、Home Assistant から通常の `light` エンティティとして扱えます。

Smart LEDZ は Telink Mesh 上に構築された Bluetooth メッシュシステムです。

## このコンポーネントでできること

- SmartLEDZ Fit に登録済みの照明器具/グループを `light` エンティティとして ESPHome から制御
- 照明器具/グループの ON/OFF、明るさ、色温度、RGB 制御

## このコンポーネントで扱わないこと

- 新しい照明器具のメッシュへの追加
- グループの作成
- グループへの照明器具の追加・削除
- SmartLEDZ Fit 側の構成管理（上記のような管理操作全般）

照明器具やグループ構成を変更した場合は、SmartLEDZ Fit 側で更新したあとに JSON を再エクスポートし、ESPHome YAML も更新してください。

## 対応環境

- ESPHome + ESP32
- `esp-idf` フレームワーク（推奨）

## 対応機器と検証状況

`device_type` は次の対応関係です。

- `dimmable`: 無線調光照明
- `tunable`: Tunable LEDZ シリーズ
- `synca`: Synca シリーズ

検証状況:

- 実機での動作確認は、Synca シリーズのスポットライト `SXS3025WB` のみで実施しています。
- 実装は複数台メッシュでの利用を想定していますが、手元の対応機器が 1 台のみのため、単一デバイスのメッシュ構成でのみ確認済みです。

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
2. 変換ツールを開く: https://hrko.github.io/esphome-smart-ledz/
3. SmartLEDZ Fit のエクスポート JSON を読み込む
4. 変換対象（デバイス/グループ）を選ぶ
5. 出力された `ESPHome YAML` と `secrets.yaml` をコピーする
6. ESPHome 端末の設定に貼り付けて、インストールする

変換ツールは静的ページです。読み込んだ JSON はブラウザ内だけで処理され、サーバーには送信されません。機密性を重視する場合は、`tools/smartledz-export-spa/` の `index.html` をローカルで開いて利用できます。この方法ならネットワークを切った状態（機内モード相当）でも実行できます。

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
- 任意: `duv_number`（synca 専用、デフォルトで自動生成）
- 任意: `ignore_transition`（デフォルト `true`）

### `duv_number`（synca 専用）

`duv_number` は DUV をランタイムで変更できる `number` エンティティです。

- `device_type: synca` のときのみ利用可能
- synca ライトでは未指定でも自動生成
- 値域は `-6.0〜6.0` 固定
- 任意: `step`（デフォルト `0.1`）
- 任意: `restore_value`（デフォルト `false`）
- 任意: `initial_value`（デフォルト `ct_duv`）
- 任意: `name`, `id`（未指定時は自動生成）

挙動:

- synca ライトが ON かつ色温度モード中に DUV を変更すると、即時反映されます。
- ライトが OFF の場合は DUV 値のみ更新し、次回のライト操作時に反映されます。

### `target` の指定

- 個別デバイス: 例 `0x0001`
- グループ: 例 `0x8001`（グループは `0x8000` 以上のアドレス空間を使用）

## 運用上の注意

- Bluetooth LE スタックは CPU/メモリ使用量が大きいため、本コンポーネントを動かす ESP32 ボードは本コンポーネント専用で使用することを推奨します。
- `mesh_name` / `mesh_password` は `!secret` 化して運用することを推奨します。

## 内部アーキテクチャ

- Telink Mesh の ESP-IDF セッション層: `https://github.com/hrko/esp-telink-mesh`
- Smart LEDZ プロトコル層: `https://github.com/hrko/smartledz-protocol`
- ESPHome 統合層: `components/smart_ledz/`

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
5. **商標について**
   「ENDO」、「LEDZ」、「Smart LEDZ」、「SmartLEDZ Fit」、「SmartLEDZ Fit Plus」、「Tunable LEDZ」、および「Synca」は、株式会社遠藤照明の商標または登録商標です。その他の記載されている会社名、製品名などは一般に各社の商標または登録商標です。本プロジェクトにおける商標の使用は、相互運用性の対象となるデバイスを特定する目的（商標の言及的利用）のみであり、権利の侵害や、メーカーによる公認・推奨を暗示するものではありません。

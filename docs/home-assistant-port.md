# Home Assistant 統合への移植について

本プロジェクトでは、Home Assistant の Bluetooth 機能を直接使う統合への移植を検討しましたが、通知受信の互換性と保守負担を理由に、2026年10月に移植を打ち切りました。現時点では、ESP32 上で通信を処理し、ESPHome 経由で Home Assistant にエンティティを公開する構成を採用しています。

検証した機器では、通知特性に通常の通知有効化で使用する CCCD（Client Characteristic Configuration Descriptor、UUID `0x2902`）がなく、`0x2901` の記述子が存在しました。既存の ESP-IDF 実装では、端末側の通知登録と通知特性への `0x01` 書き込みにより、状態通知の受信を実機で確認できました。一方、Python への移植では認証まで成功しても、標準的な BLE 通知登録が障害になりました。

| 検証構成 | 結果 |
| --- | --- |
| ESP32-S3 + 既存の ESPHome コンポーネント | 状態通知を繰り返し受信できることを確認 |
| Windows + Bleak 3.0.2 | 標準の通知登録は `Write Not Permitted`。CCCD 書き込みを省略して WinRT のイベントハンドラーだけを登録する実験でも通知を受信できず |
| Ubuntu 22.04 + USB Bluetooth + Bleak 3.0.2 + BlueZ 5.64 | 認証は成功したが通知登録でタイムアウト |
| 同 Linux 環境 + BlueZ 上流版（コミット `7428ca2df9356363079db0f48412c20cbdb9471f`） | 通知登録で同様にタイムアウト |

検証した BlueZ には、通知特性の記述子が1個だけの場合に、その種類を探索せず CCCD と推定する処理がありました。実際に、この環境では対象の記述子が `0x2902` と報告されました。この処理を変更したビルドでの通知受信は未確認です。

Bluetooth プロキシ経由でも、`bleak-esphome` の標準の通知登録には制約があります。2026年10月12日に確認した [`start_notify` の実装](https://github.com/Bluetooth-Devices/bleak-esphome/blob/b2fb6bf4cb2ea4bf52284ce8664d11e3de46fdf4/src/bleak_esphome/backend/client.py)では、`REMOTE_CACHING` 対応（接続 v3）の場合、CCCD を検索して書き込み、CCCD がなければ例外を送出して通知登録を解除します。そのため、今回の機器のように CCCD がない特性では、この経路で通知登録を完了できません。ESP32 の既存実装で受信できることは、汎用 Bluetooth プロキシ経由での受信を保証するものではなく、こちらも通知登録処理の変更と検証が必要になります。これは公開ソースに基づく判断であり、プロキシ経由の実機検証結果ではありません。旧接続方式および Home Assistant 本体での動作も未検証です。

これらは、すべての機器や将来の BLE スタックで移植が不可能であることを示すものではありません。ただし、BlueZ などの修正を必要とする構成では、Home Assistant 統合の配布だけでは対応できず、Home Assistant OS への修正反映や独自環境の保守まで必要になる可能性があります。既存の ESPHome 構成では Home Assistant 側の Bluetooth スタックに依存せずに状態通知を扱えるため、本プロジェクトでは ESPHome コンポーネントとして維持する方針です。この ESP32 は Smart LEDZ 専用ブリッジとして動作し、汎用 Bluetooth プロキシとして接続する構成ではありません。

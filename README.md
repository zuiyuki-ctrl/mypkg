# System Monitor

システムリソース（CPU、メモリ、ディスク使用率）を監視し、ROS 2トピックとして公開するROS 2パッケージです。ROS 2アプリケーションにおけるシステムヘルス監視、リソース追跡、パフォーマンス分析に役立ちます。

## 機能

- **リアルタイムシステム監視**: CPU、メモリ、ディスク使用率を継続的に監視
- **ROS 2トピック公開**: リソースデータを`std_msgs/Float32MultiArray`メッセージとして公開
- **リソースロギング**: リソースデータをCSVファイルに保存するオプションのロギングノード
- **ローンチファイル対応**: 監視ノードとロギングノードを同時に起動できる便利なローンチファイル

## ノード

### system_monitor_node

システムリソースを収集して公開するメインの監視ノードです。

**公開トピック:**
- `/system_resources` (`std_msgs/Float32MultiArray`): 1 Hzでシステムリソースデータを公開
  - メッセージ形式: `[cpu_percent, memory_percent, disk_percent, timestamp]`
  - `cpu_percent`: CPU使用率（0-100）
  - `memory_percent`: メモリ使用率（0-100）
  - `disk_percent`: ルートファイルシステムのディスク使用率（0-100）
  - `timestamp`: 測定時のUnixタイムスタンプ

**パラメータ:**
- なし（デフォルトの監視間隔は1秒）

### resource_logger_node

システムリソースデータを購読してCSVファイルに記録するサブスクライバーノードです。

**購読トピック:**
- `/system_resources` (`std_msgs/Float32MultiArray`): ログに記録するシステムリソースデータ

**出力:**
- ログファイルは`~/.ros/system_monitor_logs/`ディレクトリに保存されます
- ファイル名はタイムスタンプ付き: `resource_log_YYYYMMDD_HHMMSS.csv`
- CSV形式: `timestamp,cpu_percent,memory_percent,disk_percent`

## 依存関係

- `psutil`: システムおよびプロセスユーティリティ用のPythonライブラリ（Python 3）

### インストール

psutilをインストールします:

```bash
pip3 install psutil
```

## 使用方法

### 個別のノードを実行

システムモニターノードを実行:

```bash
ros2 run system_monitor system_monitor_node
```

リソースロガーノードを実行:

```bash
ros2 run system_monitor resource_logger_node
```

### ローンチファイルを使用

両方のノードを同時に起動:

```bash
ros2 launch system_monitor system_monitor.launch.py
```

### トピックの監視と実行例

別のターミナルで公開されているリソースデータを表示:

```bash
ros2 topic echo /system_resources
```

実行すると以下のような出力が表示されます:

```
data:
- 25.5    # CPU使用率: 25.5%
- 45.2    # メモリ使用率: 45.2%
- 60.8    # ディスク使用率: 60.8%
- 1704067200.123  # タイムスタンプ

data:
- 26.1    # CPU使用率: 26.1%
- 45.3    # メモリ使用率: 45.3%
- 60.8    # ディスク使用率: 60.8%
- 1704067201.124  # タイムスタンプ
```

リソースロガーノードを実行している場合、ログファイル（`~/.ros/system_monitor_logs/resource_log_YYYYMMDD_HHMMSS.csv`）には以下のようなCSVデータが記録されます:

```csv
timestamp,cpu_percent,memory_percent,disk_percent
1704067200.123,25.5,45.2,60.8
1704067201.124,26.1,45.3,60.8
1704067202.125,24.8,45.1,60.8
```

トピック情報を確認:

```bash
ros2 topic info /system_resources
ros2 topic hz /system_resources
```

## 使用例

- **システムヘルス監視**: 長時間実行されるROS 2アプリケーションのリソース使用率を監視
- **パフォーマンス分析**: 時間の経過とともにリソース消費を追跡し、ボトルネックを特定
- **リソースアラート**: 公開されたデータを使用して、リソースが不足した際にアラートを発火
- **ロギングと分析**: 履歴データを収集してオフライン分析やレポート作成に活用

## ライセンス

Copyright 2024 zuiyuki-ctrl

Apache License, Version 2.0でライセンスされています。詳細については[LICENSE](LICENSE)ファイルを参照してください。

## コントリビューション

コントリビューションを歓迎します！以下の点にご注意ください:

1. すべてのコードはPEP 8スタイルガイドラインに従ってください
2. すべてのソースファイルに著作権ヘッダーを含めてください
3. `colcon test`を使用してテストが通ることを確認してください
4. コードには適切なdocstringで文書化してください

## トラブルシューティング

**問題**: ログファイルの書き込み時の権限エラー

**解決方法**: `~/.ros/system_monitor_logs/`ディレクトリが書き込み可能であることを確認:
```bash
mkdir -p ~/.ros/system_monitor_logs
chmod 755 ~/.ros/system_monitor_logs
```

## 作成者

zuiyuki-ctrl

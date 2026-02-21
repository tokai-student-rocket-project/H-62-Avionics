# PlatformIO の使い方（簡易ガイド）

## 目次
- [PlatformIO の使い方（簡易ガイド）](#platformio-の使い方簡易ガイド)
  - [目次](#目次)
  - [概要](#概要)
  - [インストール](#インストール)
  - [新規プロジェクトの作成](#新規プロジェクトの作成)
  - [ビルドと書き込み](#ビルドと書き込み)
  - [シリアルモニタ](#シリアルモニタ)
  - [ライブラリ管理](#ライブラリ管理)
  - [トラブルシューティングとコツ](#トラブルシューティングとコツ)
  - [参考（公式）](#参考公式)

---

## 概要
PlatformIO は組み込み開発向けのエコシステムで、複数のボード・フレームワークを統一的に扱えます。コマンドライン（`platformio` / `pio`）または VS Code 拡張で利用できます。

## インストール
- VS Code 拡張: VS Code の拡張から「PlatformIO IDE」をインストール（推奨 GUI）
- CLI（pip）:

```bash
python -m pip install -U pip
python -m pip install -U platformio
```

- インストーラやパッケージマネージャを使う場合は公式サイトを参照してください。

公式サイト: https://platformio.org

公式ドキュメント: https://docs.platformio.org

## 新規プロジェクトの作成
- VS Code: コマンドパレットから "PlatformIO: New Project" を選択して作成

- `platformio.ini` の基本例:

```ini
[env:myboard]
platform = espressif32
board = esp32dev
framework = arduino
```

## ビルドと書き込み

1. VSCodeの左側にあるPlatformIOアイコン (エイリアンの頭のようなアイコン) をクリックする。
2. `Project Tasks` の中から、対象の環境 (`env:***`) を見つける。
3. `Build` をクリックするとコンパイルが実行される。
4. `Upload` をクリックすると、接続されているデバイスにプログラムが書き込まれる。

## シリアルモニタ

1. `Monitor` をクリックすると、シリアルモニターが起動する。

## ライブラリ管理

- `platformio.ini` に `lib_deps` を書くとプロジェクトの依存として自動取得されます。

```ini
lib_deps = 
	adafruit/Adafruit Unified Sensor@^1.0.0
```

## トラブルシューティングとコツ
- ボードIDが不明な場合: `pio boards` で検索
- シリアルポートが競合する場合は他のターミナルを閉じる
- ビルドエラー時は依存と `platformio.ini` の `platform`/`framework` を確認
- キャッシュや古い環境で問題が続く場合は `.pio` フォルダを削除して再ビルド

## 参考（公式）
- PlatformIO 公式サイト: https://platformio.org
- 公式ドキュメント: https://docs.platformio.org

---

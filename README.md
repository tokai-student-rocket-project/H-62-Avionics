# H-62-Avionics

H-62搭載計器　すばる1.3 | 2025年度 夏季 能代 NOGO

すばる1.2 の機能を引き継ぎつつ海打上げ実験用に改良を加えました

- 水密機能の搭載
- 基板径をΦ90に変更
- LYLYGO開発ボードを活用した地上局

## ドキュメント
- [PDR](./Document/PDR/Subaru1.3_PDR.pdf)
- [CDR](./Document/CDR/Subaru1.3_CDR.pdf)
- [報告書](./Document/)

## ディレクトリ構成
### [Document](./Document/)
資料を格納するディレクトリ

### [KiCAD](./KiCAD/)
KiCADのプロジェクトファイルを格納するディレクトリ
- [Flight Module](./KiCAD/FlightModule/)
- [Battery Module](./KiCAD/BatteryModule/)
- [Power Module](./KiCAD/PowerControlModule/)
- [Battery Module](./KiCAD/18650BatteryModule/)
- [Sensing Module](./KiCAD/SensingModule/)
- [Valve Control Module](./KiCAD/ValveControlModule/)
- [Subaru Plate](./KiCAD/SubaruPlate/)


### [lib](./lib/)
ライブラリを格納するディレクトリ

Lib_ から始まるファイルは自作ライブラリとなっています

### [src](./src/)
ソースコードを格納するディレクトリ


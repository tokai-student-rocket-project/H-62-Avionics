![Hero](/Document/Image/Hero.JPG)
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

Lib_ から始まるライブラリはラッパーライブラリとなっています

### [src](./src/)
ソースコードを格納するディレクトリ

**Flight** : 搭載計器のソースコード
- [Flight Module](./src/Flight/FlightModule/FlightModule.cpp)
- [Power Module](./src/Flight/PowerModule/PowerModule.cpp)
- [Sensing Module](./src/Flight/SensingModule/SensingModule.cpp)
- [Valve Control Module](./src/Flight/ValveControlModule/ValveControlModule.cpp)

**Ground** : 地上局のソースコード
- [Ground Flight Module](./src/Ground/CSV/FlightModule/FlightModule.cpp)
- [Ground Sensing Module](./src/Ground/CSV/SensingModule/SensingModule.cpp)
- [Sea Flight Module](./src/Ground/T-BEAM/FlightModule/FlightModule.cpp)
- [Teleplot Ground Flight Module](./src/Ground/Teleplot/FlightModule/FlightModule.cpp)
  - [Teleplot Layout](./src/Ground/Teleplot/FlightModule/teleplot_layout_FlightModule.json)

**LogDumper** : ログ読み出しのソースコード
- [Flight Module](./src/LogDumper/FlightModule/FlightModule.cpp)
- [Sensing Module](./src/LogDumper/SensingModule/SensingModule.cpp)

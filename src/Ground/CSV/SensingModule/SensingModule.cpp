/**
 * @file SensingModule.cpp
 * @brief 地上局（退避所）用のセンシングモジュールメインプログラム。
 * @details
 * このプログラムは、ロケットから送られてくる様々なセンサーデータ（加速度、高度、電力など）を
 * LoRa無線で受け取り、OLEDディスプレイに表示したり、パソコンにCSV形式で記録したりします。
 * ボタン操作で表示する情報を切り替えたり、ロケットにコマンドを送ったりすることもできます。
 * プログラムの動きを理解しやすいように、各部分の役割を詳しく説明しています。
 * @author H-62 Avionics Team
 * @date 2025-08-20
 */

// Arduinoの基本的な機能を使うための準備
#include <Arduino.h>
// SPI通信（センサーなどとデータをやり取りする方法）を使うための準備
#include <SPI.h>
// I2C通信（ディスプレイなどとデータをやり取りする方法）を使うための準備
#include <Wire.h>
// LoRa無線通信（遠くまでデータを送るための技術）を使うための準備
#include <LoRa.h>
// LoRaで送られてくるメッセージを分かりやすくするための準備
#include <MsgPacketizer.h>
// いろいろな処理を順番に、または同時に動かすための準備
#include <TaskManager.h>
// LoRa通信で使う基板の設定を読み込むための準備
#include "LoRaBoards.h"
// GPSのデータを簡単に扱えるようにするための準備
#include <TinyGPS++.h>
// ディスプレイに絵や文字を表示するための基本的な機能の準備
#include <Adafruit_GFX.h>
// 特定のOLEDディスプレイ（SSD1306）を動かすための準備
#include <Adafruit_SSD1306.h>

#define BUTTON_PIN 38                          ///< ボタンが接続されているArduinoのピン番号です。
const unsigned long LONG_PRESS_TIME_MS = 1000; ///< ボタンを「長押し」と判断する時間（ミリ秒）です。

Adafruit_SSD1306 display(128, 64, &Wire, -1); ///< ディスプレイを操作するための特別な「道具」です。
int currentPage = 0;                          ///< ディスプレイに表示する情報の「ページ番号」を覚えています。
const int NUM_PAGES = 9;                      ///< ディスプレイに表示できるページの全部の数です。

TinyGPSPlus onbordGps; ///< このロケットについているGPSからデータを受け取るための特別な「道具」です。

// --- 基準気圧設定 ---
uint32_t primaryPressure = 1014;   ///< 基準となる気圧（最初の気圧）を覚えています。
uint32_t secondaryPressure = 1014; ///< もう一つの基準となる気圧を覚えています。

// --- テレメトリーデータ ---
// ロケットから送られてくる様々なセンサーデータを一時的に保存しておくための変数たちです。
float telemetryRssi = 0.0;                   ///< LoRa無線でデータがどれくらいの強さで届いたか（電波の強さ）を示します。
float telemetrySnr = 0.0;                    ///< LoRa無線でデータがどれくらいきれいに届いたか（ノイズの少なさ）を示します。
float telemetryUptime = 0.0;                 ///< ロケットが動き始めてからどれくらいの時間が経ったか（秒）を示します。
char telemetryIdent = ' ';                   ///< ロケットの識別子（どのロケットからのデータか）を示します。
uint8_t telemetryLoggerUsage = 0;            ///< ロケットのデータ記録装置（ロガー）がどれくらい使われているか（%）を示します。
bool telemetryDoLogging = false;             ///< ロケットが今、データを記録中かどうか（はい/いいえ）を示します。
uint8_t telemetryFramNumber = 0;             ///< ロケットのデータ記録装置の番号を示します。
float telemetryAccelerationX = 0.0;          ///< ロケットのX方向の加速度（速さの変化）を示します。
float telemetryAccelerationY = 0.0;          ///< ロケットのY方向の加速度を示します。
float telemetryAccelerationZ = 0.0;          ///< ロケットのZ方向の加速度を示します。
float telemetryAccelerationNorm = 0.0;       ///< ロケットの全体の加速度の大きさを示します。
float telemetryRoll = 0.0;                   ///< ロケットのロール角（縦軸周りの回転）を示します。
float telemetryPitch = 0.0;                  ///< ロケットのピッチ角（横軸周りの回転）を示します。
float telemetryYaw = 0.0;                    ///< ロケットのヨー角（垂直軸周りの回転）を示します。
float telemetryForceX = 0.0;                 ///< ロケットのX方向にかかる力（推力など）を示します。
float telemetryJerkX = 0.0;                  ///< ロケットのX方向のジャーク（加速度の変化率）を示します。
float telemetryAltitude = 0.0;               ///< ロケットの現在の高度（地面からの高さ）を示します。
float telemetryVerticalSpeed = 0.0;          ///< ロケットの垂直方向の速さ（上昇・下降の速さ）を示します。
float telemetryEstimated = 0.0;              ///< ロケットの推定される何か（文脈による）を示します。
float telemetryApogee = 0.0;                 ///< ロケットが到達した最高高度を示します。
float telemetryExternalVoltage = 0.0;        ///< 外部電源の電圧を示します。
float telemetryBatteryVoltage = 0.0;         ///< バッテリーの電圧を示します。
float telemetryBusVoltage = 0.0;             ///< 内部の電力供給ライン（バス）の電圧を示します。
float telemetryExternalCurrent = 0.0;        ///< 外部電源の電流を示します。
float telemetryBatteryCurrent = 0.0;         ///< バッテリーの電流を示します。
float telemetryBusCurrent = 0.0;             ///< 内部の電力供給ラインの電流を示します。
float telemetryExternalPower = 0.0;          ///< 外部電源の電力（消費量）を示します。
float telemetryBatteryPower = 0.0;           ///< バッテリーの電力（消費量）を示します。
float telemetryBusPower = 0.0;               ///< 内部の電力供給ラインの電力（消費量）を示します。
float telemetryExternalDieTemperature = 0.0; ///< 外部の部品の温度を示します。
float telemetryBatteryDieTemperature = 0.0;  ///< バッテリーの部品の温度を示します。
float telemetryBusDieTemperature = 0.0;      ///< 内部の電力供給ラインの部品の温度を示します。

unsigned long buttonPressTime = 0; ///< ボタンが押され始めた時間を覚えています。
bool longPressSent = false;        ///< ボタンの長押しコマンドがもう送られたかどうか（はい/いいえ）を覚えています。

// --- 関数の宣言（これから作る関数の名前と、どんな情報を受け取るかを先に教えておく） ---
void updateDisplay();   // ディスプレイの表示を更新する関数
void sendLoRaCommand(); // LoRa無線でコマンドを送る関数
void loraRssiBar();     // LoRaの電波強度を棒グラフで表示する関数

/**
 * @brief LoRa無線でデータがどれくらいの強さで届いたか（電波の強さ）を棒グラフで表示します。
 * @details
 * この関数は、ロケットから送られてくる電波の強さ（RSSIという値）を測り、
 * その強さに応じてディスプレイに棒グラフ（アンテナのマークのようなもの）を描きます。
 * 電波が強いほど、棒がたくさん表示されます。
 * @note ディスプレイの右下に表示されます。
 */
void loraRssiBar()
{
    int rssi = LoRa.packetRssi(); // 今届いた電波の強さ（RSSI）を測ります。
    int bars = 0;                 // 表示する棒の数を数えるための変数です。
    // RSSIの値によって、表示する棒の数を決めます。
    if (rssi > -95.8)
        bars = 5; // 電波がとても強い
    else if (rssi > -101.6)
        bars = 4;
    else if (rssi > -107.4)
        bars = 3;
    else if (rssi > -113.2)
        bars = 2;
    else
        bars = 1; // 電波が弱い

    int barWidth = 5;   // 棒1本の幅です。
    int barSpacing = 1; // 棒と棒の間の隙間です。
    int barHeight = 8;  // 棒の高さです。
    // 棒グラフをディスプレイの右端に表示するための開始位置を計算します。
    int startX = 128 - (5 * (barWidth + barSpacing));
    int startY = 64 - barHeight; // 棒グラフをディスプレイの下端に表示するための開始位置を計算します。

    for (int i = 0; i < 5; i++)
    {
        // 棒の高さは、棒の番号（i）によって少しずつ変わります。
        int currentBarHeight = barHeight * (i + 1) / 5;
        // もし、今の棒の番号が表示するべき棒の数（bars）より小さければ、その棒を描きます。
        if (i < bars)
        {
            // ディスプレイに四角い棒を描きます。
            display.fillRect(startX + i * (barWidth + barSpacing), startY + (barHeight - currentBarHeight), barWidth, currentBarHeight, SSD1306_WHITE);
        }
    }
}

/**
 * @brief ディスプレイの各ページの共通のヘッダー（一番上の部分）を描画します。
 * @param[in] title これは、表示したいページの「タイトル」の文字です。
 *                  この関数は、このタイトルを使ってヘッダーを作りますが、
 *                  タイトル自体は変更しません。
 * @details
 * この関数は、ディスプレイの表示を一度全部消してから、
 * 今何ページ目を見ているか（P:ページ番号）、ページのタイトル、
 * そしてLoRaの電波強度を示す棒グラフを画面の一番上に表示します。
 */
void displayHeader(const char *title)
{
    display.clearDisplay();              // ディスプレイの表示を全部消します。
    display.setTextSize(1);              // 文字の大きさを一番小さいサイズに設定します。
    display.setTextColor(SSD1306_WHITE); // 文字の色を白に設定します。
    display.setCursor(0, 0);             // 文字を書き始める位置をディスプレイの左上（0,0）に設定します。
    display.print(F("P"));               // 「P」という文字を表示します。
    display.print(currentPage);          // 今のページ番号を表示します。
    display.print(F(":"));               // 「:」という文字を表示します。
    display.print(title);                // ページのタイトルを表示します。
    loraRssiBar();                       // LoRaの電波強度を示す棒グラフを表示します。
}

/**
 * @brief ページ0に「ステータス情報」を表示します。
 * @details
 * このページでは、LoRaの電波強度（RSSIとSNR）、ロケットが動いている時間（UPTIME）、
 * データ記録の状態（LOGとMEMORY）、そしてデータ記録装置の番号（FRAM）など、
 * ロケットの基本的な状態を一覧で確認できます。
 */
void displayPage0()
{
    displayHeader(" Status -");    // ページのヘッダーを「Status -」として表示します。
    display.setCursor(0, 10);      // 文字を書き始める位置を少し下にずらします。
    display.print(F("RSSI   : ")); // 「RSSI   : 」という文字を表示します。
    display.print(telemetryRssi);  // 受信したRSSIの値を表示します。
    display.println(F("dBm"));     // 「dBm」という単位を表示し、次の行に移ります。
    // 以下、同様に他のステータス情報を表示します。
    display.print(F("SNR    : "));
    display.print(telemetrySnr);
    display.println(F("dB"));
    display.print(F("UPTIME : "));
    display.print(telemetryUptime);
    display.println(F("s"));
    display.print(F("LOG    : "));
    display.println(telemetryDoLogging ? "ON" : "OFF"); // ロギング中なら「ON」、そうでなければ「OFF」を表示します。
    display.print(F("MEMORY : "));
    display.print(telemetryLoggerUsage);
    display.println(F("%"));
    display.print(F("FRAM: "));
    display.println(telemetryFramNumber);
    display.display(); // ディスプレイに実際に表示します。
}

/**
 * @brief ページ1に「高度情報」を表示します。
 * @details
 * このページでは、ロケットの現在の高度（ALT）、垂直方向の速さ（V/S）、
 * 最高高度（APO）
 * 高度に関する重要な情報を確認できます。
 */
void displayPage1()
{
    displayHeader(" Altitude -"); // ページのヘッダーを「Altitude -」として表示します。
    display.setCursor(0, 10);     // 文字を書き始める位置を少し下にずらします。
    display.print(F("ALT: "));
    display.print(telemetryAltitude);
    display.println(F("m"));
    display.print(F("V/S: "));
    display.print(telemetryVerticalSpeed);
    display.println(F("m/s"));
    display.print(F("APO: "));
    display.print(telemetryApogee);
    display.println(F("m"));
    display.display();
}

/**
 * @brief ページ2に「加速度情報」を表示します。
 * @details
 * このページでは、ロケットがどれくらいの速さで加速しているか（加速度）を、
 * X、Y、Zの3つの方向と、全体の大きさ（N）で確認できます。
 */
void displayPage2()
{
    displayHeader(" Accel [m/s2] -"); // ページのヘッダーを「Accel [m/s2] -」として表示します。
    display.setCursor(0, 10);         // 文字を書き始める位置を少し下にずらします。
    display.print(F("X: "));
    display.println(telemetryAccelerationX);
    display.print(F("Y: "));
    display.println(telemetryAccelerationY);
    display.print(F("Z: "));
    display.println(telemetryAccelerationZ);
    display.print(F("N: "));
    display.println(telemetryAccelerationNorm);
    display.display();
}

/**
 * @brief ページ3に「姿勢情報」を表示します。
 * @details
 * このページでは、ロケットがどの方向を向いているか（姿勢）を、
 * ロール、ピッチ、ヨーという3つの角度で確認できます。
 */
void displayPage3()
{
    displayHeader(" Orientation [deg] -"); // ページのヘッダーを「Orientation [deg] -」として表示します。
    display.setCursor(0, 10);              // 文字を書き始める位置を少し下にずらします。
    display.print(F("ROLL : "));
    display.println(telemetryRoll);
    display.print(F("PITCH: "));
    display.println(telemetryPitch);
    display.print(F("YAW  : "));
    display.println(telemetryYaw);
    display.display();
}

/**
 * @brief ページ4に「内部電力供給ライン（バス）の電力情報」を表示します。
 * @details
 * このページでは、ロケット内部の電子部品に電力を供給しているラインの電圧（V）、
 * 電流（I）、そして電力（P）を確認できます。
 */
void displayPage4()
{
    displayHeader(" Bus Power -"); // ページのヘッダーを「Bus Power -」として表示します。
    display.setCursor(0, 10);      // 文字を書き始める位置を少し下にずらします。
    display.print(F("V: "));
    display.print(telemetryBusVoltage);
    display.println(F(" V"));
    display.print(F("I: "));
    display.print(telemetryBusCurrent / 1000.0);
    display.println(F(" A"));
    display.print(F("P: "));
    display.print(telemetryBusPower);
    display.println(F(" W"));
    display.display();
}

/**
 * @brief ページ5に「バッテリーの電力情報」を表示します。
 * @details
 * このページでは、ロケットのバッテリーの電圧（V）、電流（I）、
 * そして電力（P）を確認できます。
 */
void displayPage5()
{
    displayHeader(" Battery Power -"); // ページのヘッダーを「Battery Power -」として表示します。
    display.setCursor(0, 10);
    display.print(F("V: "));
    display.print(telemetryBatteryVoltage);
    display.println(F(" V"));
    display.print(F("I: "));
    display.print(telemetryBatteryCurrent / 1000.0);
    display.println(F(" A"));
    display.print(F("P: "));
    display.print(telemetryBatteryPower);
    display.println(F(" W"));
    display.display();
}

/**
 * @brief ページ6に「外部電源の電力情報」を表示します。
 * @details
 * このページでは、ロケットに接続されている外部電源の電圧（V）、電流（I）、
 * そして電力（P）を確認できます。
 */
void displayPage6()
{
    displayHeader(" External Power -"); // ページのヘッダーを「External Power -」として表示します。
    display.setCursor(0, 10);
    display.print(F("V: "));
    display.print(telemetryExternalVoltage);
    display.println(F(" V"));
    display.print(F("I: "));
    display.print(telemetryExternalCurrent / 1000.0);
    display.println(F(" A"));
    display.print(F("P: "));
    display.print(telemetryExternalPower);
    display.println(F(" W"));
    display.display();
}

/**
 * @brief ページ7に「温度情報」を表示します。
 * @details
 * このページでは、ロケット内部の電子部品（バス、バッテリー、外部）の温度を
 * それぞれ確認できます。
 */
void displayPage7()
{
    displayHeader(" Temperature -"); // ページのヘッダーを「Temperature -」として表示します。
    display.setCursor(0, 10);
    display.print(F("BUS: "));
    display.print(telemetryBusDieTemperature);
    display.println(" C");

    display.print(F("BAT: "));
    display.print(telemetryBatteryDieTemperature);
    display.println(" C");

    display.print(F("EXT: "));
    display.print(telemetryExternalDieTemperature);
    display.println(" C");

    display.display();
}

/**
 * @brief ページ8に「ロケット搭載GPSの情報」を表示します。
 * @details
 * このページでは、ロケットに搭載されているGPSが取得した位置情報（緯度、経度）や
 * 捕捉している衛星の数を確認できます。もし有効なGPSデータがなければ、その旨を表示します。
 */
void displayPage8()
{
    displayHeader(" Onboard GPS -"); // ページのヘッダーを「Onboard GPS -」として表示します。
    display.setCursor(0, 10);
    // GPSデータが有効かどうかを確認します。
    if (onbordGps.location.isValid())
    {
        display.print(F("LAT: "));
        display.println(onbordGps.location.lat(), 6); // 緯度を表示します（小数点以下6桁まで）。
        display.print(F("LNG: "));
        display.println(onbordGps.location.lng(), 6); // 経度を表示します（小数点以下6桁まで）。
        display.print(F("SAT: "));
        display.println(onbordGps.satellites.value()); // 捕捉している衛星の数を表示します。
    }
    else
    {
        // 有効なGPSデータがないことを表示します。
        display.print(F("LAT: "));
        display.println("-.------");
        display.print(F("LNG: "));
        display.println("-.------");
        display.print(F("SAT: "));
        display.println("-");
    }
    display.display();
}

/**
 * @brief 現在のページ番号に応じて、対応する表示関数を呼び出し、ディスプレイを更新します。
 * @details
 * この関数は、`currentPage`という変数に保存されているページ番号を見て、
 * そのページに合った情報を表示する関数（例: `displayPage0()`など）を呼び出します。
 * これにより、ボタンを押すたびに表示内容が切り替わります。
 */
void updateDisplay()
{
    switch (currentPage) // 今のページ番号によって、どの表示関数を呼ぶか決めます。
    {
    case 0:
        displayPage0(); // ページ0なら、ステータス情報を表示します。
        break;
    case 1:
        displayPage1(); // ページ1なら、高度情報を表示します。
        break;
    case 2:
        displayPage2(); // ページ2なら、加速度情報を表示します。
        break;
    case 3:
        displayPage3(); // ページ3なら、姿勢情報を表示します。
        break;
    case 4:
        displayPage4(); // ページ4なら、バス電力情報を表示します。
        break;
    case 5:
        displayPage5(); // ページ5なら、バッテリー電力情報を表示します。
        break;
    case 6:
        displayPage6(); // ページ6なら、外部電力情報を表示します。
        break;
    case 7:
        displayPage7(); // ページ7なら、温度情報を表示します。
        break;
    case 8:
        displayPage8(); // ページ8なら、搭載GPS情報を表示します。
        break;
    }
}

/**
 * @brief ロケットに基準気圧設定コマンドをLoRa無線で送信します。
 * @details
 * この関数は、`primaryPressure`と`secondaryPressure`という変数に保存されている
 * 基準気圧の値を、LoRa無線を使ってロケットに送ります。
 * これにより、ロケットの気圧センサーの基準値を設定できます。
 */
void sendLoRaCommand()
{
    // 送信するメッセージ（パケット）を作成します。0xF4はコマンドの種類を示す番号です。
    const auto &packet = MsgPacketizer::encode(0xF4, primaryPressure, secondaryPressure);
    LoRa.beginPacket();                                 // LoRa無線での送信を開始します。
    LoRa.write(packet.data.data(), packet.data.size()); // 作成したメッセージをLoRa無線に乗せます。
    LoRa.endPacket();                                   // LoRa無線での送信を終了します。
}

/**
 * @brief ロケット搭載GPSのデータを定期的に更新するタスクです。
 * @details
 * この関数は、ロケット搭載GPSから送られてくる生データ（SerialGPS）を読み込み、
 * `onbordGps`という特別な道具を使って、緯度や経度などの分かりやすい情報に変換します。
 * もし今、GPS情報が表示されているページ（ページ8）であれば、ディスプレイも更新します。
 */
void taskGpsUpdate()
{
    // SerialGPSからデータが来る限り、読み込んでGPSデータを更新します。
    while (SerialGPS.available())
    {
        onbordGps.encode(SerialGPS.read()); // 1文字ずつ読み込んでGPSデータを解析します。
    }

    // もし今、GPS情報が表示されているページ（ページ8）であれば、ディスプレイを更新します。
    if (currentPage == 8)
    {
        updateDisplay();
    }
}

/**
 * @brief ボタンの状態をチェックし、短押し・長押しに応じた処理を行うタスクです。
 * @details
 * この関数は、ボタンが押されているかどうかを常に監視しています。
 * - **短押し**された場合：ディスプレイの表示ページを次のページに切り替えます。
 * - **長押し**された場合：ロケットに基準気圧設定コマンドをLoRa無線で送信します。
 */
void taskButtonCheck()
{
    bool buttonPressed = !digitalRead(BUTTON_PIN); // ボタンが押されているか（はい/いいえ）を調べます。

    if (buttonPressed) // もしボタンが押されていたら
    {
        if (buttonPressTime == 0) // 初めてボタンが押された瞬間なら
        {
            buttonPressTime = millis(); // 今の時間を記録します。
            longPressSent = false;      // 長押しコマンドはまだ送られていないことにします。
        }
        else if ((millis() - buttonPressTime > LONG_PRESS_TIME_MS) && !longPressSent) // 長押しと判断できる時間が経っていて、まだコマンドを送っていなければ
        {
            // 長押し時の動作
            sendLoRaCommand();    // LoRaコマンドを送ります。
            longPressSent = true; // 長押しコマンドは送られたことにします。
        }
    }
    else // もしボタンが押されていなかったら
    {
        if (buttonPressTime != 0 && !longPressSent) // ボタンが離された瞬間で、長押しではなかったら
        {
            // 短押し時の動作
            currentPage = (currentPage + 1) % NUM_PAGES; // ページ番号を1つ増やし、最後のページなら最初のページに戻ります。
            updateDisplay();                             // ディスプレイの表示を更新します。
        }
        buttonPressTime = 0; // ボタンが押され始めた時間をリセットします。
    }
}

/**
 * @brief プログラムが起動したときに一度だけ実行される、初期設定を行う関数です。
 * @details
 * ここでは、Arduinoのピンの設定、ディスプレイの起動、LoRa無線の設定、
 * そして定期的に実行したい処理（タスク）の登録などを行います。
 * プログラムが正しく動くための準備を全てここで行います。
 */
void setup()
{
    setupBoards();                     // LoRa通信で使う基板の初期設定を行います。
    pinMode(BUTTON_PIN, INPUT_PULLUP); // ボタンが接続されているピンを、入力として使えるように設定します。

    // ディスプレイを起動します。もし起動に失敗したら、エラーメッセージを表示してプログラムを停止します。
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // 無限ループに入り、プログラムを停止させます。
    }
    display.clearDisplay();                   // ディスプレイの表示を全部消します。
    display.setCursor(0, 0);                  // 文字を書き始める位置を左上に設定します。
    display.setTextSize(1);                   // 文字の大きさを設定します。
    display.setTextColor(SSD1306_WHITE);      // 文字の色を白に設定します。
    display.println("GROUND SENSING MODULE"); // プログラムの名前を表示します。
    display.println("System Start");          // システムが起動したことを表示します。
    display.println();
    display.println("--- System Start ---");
    display.println("====================");
    display.println("   H-62  Avionics   ");
    display.println("     SUBARU 1.3     ");
    display.println("====================");
    display.display(); // ディスプレイに表示します。
    delay(3000);       // 3秒間待ちます。

    // LoRa無線通信のピンを設定し、通信を開始します。
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(923.8E6)) // 923.8E6MHzでLoRa通信を開始します。
    {
        Serial.println("Starting LoRa failed!"); // 失敗したらエラーメッセージを表示します。
        display.clearDisplay();
        display.println("LoRa Failed!");
        display.display();
        while (1)
            ; // 無限ループに入り、プログラムを停止させます。
    }
    LoRa.setSignalBandwidth(500E3); // LoRaの通信速度に関する設定です。

    // パソコンにCSV形式でデータを送るための準備として、一番上の行（ヘッダー）を送ります。
    Serial.println("millis,ident,loggerUsage,doLogging,framNumber,accelerationX_mps2,accelerationY_mps2,accelerationZ_mps2,accelerationNorm_mps2,roll_deg,pitch_deg,yaw_deg,forceX_N,jerkX_mps3,altitude_m,verticalSpeed_mps,estimated,apogee,externalVoltage_V,batteryVoltage_V,busVoltage_V,externalCurrent_A,batteryCurrent_A,busCurrent_A,externalPower_W,batteryPower_W,busPower_W,externalDieTemperature_C,batteryDieTemperature_C,busDieTemperature_C,rssi,snr");

    // LoRa無線で特定のメッセージ（ID: 0x0A）を受け取ったときに、何をするかを設定します。
    // ここでは、受け取ったセンサーデータを変数に保存し、ディスプレイを更新し、パソコンにCSV形式で送る処理を行います。
    MsgPacketizer::subscribe(LoRa, 0x0A,
                             []( // ここからが、メッセージを受け取ったときに実行される処理です。
                                 uint32_t millis, char ident, uint8_t loggerUsage, bool doLogging, uint8_t framNumber,
                                 int16_t accelerationX_mps2, int16_t accelerationY_mps2, int16_t accelerationZ_mps2, int16_t accelerationNorm_mps2,
                                 int16_t roll_deg, int16_t pitch_deg, int16_t yaw_deg,
                                 int16_t forceX_N, int16_t jerkX_mps3,
                                 int16_t altitude_m, int16_t verticalSpeed_mps, int16_t estimated, int16_t apogee,
                                 int16_t externalVoltage_V, int16_t batteryVoltage_V, int16_t busVoltage_V,
                                 int16_t externalCurrent_mA, int16_t batteryCurrent_mA, int16_t busCurrent_mA,
                                 int16_t externalPower_W, int16_t batteryPower_W, int16_t busPower_W,
                                 int16_t externalDieTemperature_C, int16_t batteryDieTemperature_C, int16_t busDieTemperature_C)
                             {
                                 // 受け取ったデータを、上で宣言した「telemetry」で始まる変数たちに保存します。
                                 // 必要に応じて、単位を合わせるために計算（例: 100で割る）も行います。
                                 telemetryRssi = LoRa.packetRssi();
                                 telemetrySnr = LoRa.packetSnr();
                                 telemetryUptime = (float)millis / 1000.0;
                                 telemetryIdent = ident;
                                 telemetryLoggerUsage = loggerUsage;
                                 telemetryDoLogging = doLogging;
                                 telemetryFramNumber = framNumber;
                                 telemetryAccelerationX = (float)accelerationX_mps2 / 10.0;
                                 telemetryAccelerationY = (float)accelerationY_mps2 / 10.0;
                                 telemetryAccelerationZ = (float)accelerationZ_mps2 / 10.0;
                                 telemetryAccelerationNorm = (float)accelerationNorm_mps2 / 10.0;
                                 telemetryRoll = (float)roll_deg / 10.0;
                                 telemetryPitch = (float)pitch_deg / 10.0;
                                 telemetryYaw = (float)yaw_deg / 10.0;
                                 telemetryForceX = (float)forceX_N / 10.0;
                                 telemetryJerkX = (float)jerkX_mps3 / 10.0;
                                 telemetryAltitude = (float)altitude_m / 10.0;
                                 telemetryVerticalSpeed = (float)verticalSpeed_mps / 10.0;
                                 telemetryEstimated = (float)estimated / 10.0;
                                 telemetryApogee = (float)apogee / 10.0;
                                 telemetryExternalVoltage = (float)externalVoltage_V / 100.0;
                                 telemetryBatteryVoltage = (float)batteryVoltage_V / 100.0;
                                 telemetryBusVoltage = (float)busVoltage_V / 100.0;
                                 telemetryExternalCurrent = (float)externalCurrent_mA / 100.0;
                                 telemetryBatteryCurrent = (float)batteryCurrent_mA / 100.0;
                                 telemetryBusCurrent = (float)busCurrent_mA / 100.0;
                                 telemetryExternalPower = (float)externalPower_W / 10.0;
                                 telemetryBatteryPower = (float)batteryPower_W / 10.0;
                                 telemetryBusPower = (float)busPower_W / 10.0;
                                 telemetryExternalDieTemperature = (float)externalDieTemperature_C / 10.0;
                                 telemetryBatteryDieTemperature = (float)batteryDieTemperature_C / 10.0;
                                 telemetryBusDieTemperature = (float)busDieTemperature_C / 10.0;

                                 updateDisplay(); // ディスプレイの表示を更新します。

                                 // パソコンにCSV形式でデータを送ります。
                                 Serial.print(millis / 1000.0);
                                 Serial.print(",");
                                 Serial.print(ident);
                                 Serial.print(",");
                                 Serial.print(loggerUsage);
                                 Serial.print(",");
                                 Serial.print(doLogging);
                                 Serial.print(",");
                                 Serial.print(framNumber);
                                 Serial.print(",");
                                 Serial.print(telemetryAccelerationX);
                                 Serial.print(",");
                                 Serial.print(telemetryAccelerationY);
                                 Serial.print(",");
                                 Serial.print(telemetryAccelerationZ);
                                 Serial.print(",");
                                 Serial.print(telemetryAccelerationNorm);
                                 Serial.print(",");
                                 Serial.print(telemetryRoll);
                                 Serial.print(",");
                                 Serial.print(telemetryPitch);
                                 Serial.print(",");
                                 Serial.print(telemetryYaw);
                                 Serial.print(",");
                                 Serial.print(telemetryForceX);
                                 Serial.print(",");
                                 Serial.print(telemetryJerkX);
                                 Serial.print(",");
                                 Serial.print(telemetryAltitude);
                                 Serial.print(",");
                                 Serial.print(telemetryVerticalSpeed);
                                 Serial.print(",");
                                 Serial.print(telemetryEstimated);
                                 Serial.print(",");
                                 Serial.print(telemetryApogee);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalVoltage);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryVoltage);
                                 Serial.print(",");
                                 Serial.print(telemetryBusVoltage);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalCurrent);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryCurrent);
                                 Serial.print(",");
                                 Serial.print(telemetryBusCurrent);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalPower);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryPower);
                                 Serial.print(",");
                                 Serial.print(telemetryBusPower);
                                 Serial.print(",");
                                 Serial.print(telemetryExternalDieTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryBatteryDieTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryBusDieTemperature);
                                 Serial.print(",");
                                 Serial.print(telemetryRssi);
                                 Serial.print(",");
                                 Serial.println(telemetrySnr);

                                 //  // Teleplot用にシリアル出力 (コメントアウトされているので、今は使われていません)
                                 //  Serial.print(">LoRa_RSSI_dBm:");
                                 //  Serial.println(telemetryRssi);
                                 //  Serial.print(">LoRa_SNR_dBm:");
                                 //  Serial.println(telemetrySnr);
                                 //  Serial.print(">upTime_sec:");
                                 //  Serial.println(telemetryUptime);
                                 //  Serial.print(">ident:");
                                 //  Serial.println(telemetryIdent);
                                 //  Serial.print(">doLogging_bool:");
                                 //  Serial.println(telemetryDoLogging);
                                 //  Serial.print(">loggerUsage_%:");
                                 //  Serial.println(telemetryLoggerUsage);
                                 //  Serial.print(">framNumber:");
                                 //  Serial.println(telemetryFramNumber);
                                 //  Serial.print(">acceleration_mps2_x:");
                                 //  Serial.println(telemetryAccelerationX);
                                 //  Serial.print(">acceleration_mps2_y:");
                                 //  Serial.println(telemetryAccelerationY);
                                 //  Serial.print(">acceleration_mps2_z:");
                                 //  Serial.println(telemetryAccelerationZ);
                                 //  Serial.print(">acceleration_mps2_norm:");
                                 //  Serial.println(telemetryAccelerationNorm);
                                 //  Serial.print(">orientation_deg_roll:");
                                 //  Serial.println(telemetryRoll);
                                 //  Serial.print(">orientation_deg_pitch:");
                                 //  Serial.println(telemetryPitch);
                                 //  Serial.print(">orientation_deg_yaw:");
                                 //  Serial.println(telemetryYaw);
                                 //  Serial.print(">forceX_N:");
                                 //  Serial.println(telemetryForceX);
                                 //  Serial.print(">jerkX_mps3:");
                                 //  Serial.println(telemetryJerkX);
                                 //  Serial.print(">altitude_m:");
                                 //  Serial.println(telemetryAltitude);
                                 //  Serial.print(">verticalSpeed_mps:");
                                 //  Serial.println(telemetryVerticalSpeed);
                                 //  Serial.print(">apogee_m:");
                                 //  Serial.println(telemetryApogee);
                                 //  Serial.print(">estimated_s:");
                                 //  Serial.println(telemetryEstimated);
                                 //  Serial.print(">externalVoltage_V:");
                                 //  Serial.println(telemetryExternalVoltage);
                                 //  Serial.print(">batteryVoltage_V:");
                                 //  Serial.println(telemetryBatteryVoltage);
                                 //  Serial.print(">busVoltage_V:");
                                 //  Serial.println(telemetryBusVoltage);
                                 //  Serial.print(">externalCurrent_A:");
                                 //  Serial.println(telemetryExternalCurrent);
                                 //  Serial.print(">batteryCurrent_A:");
                                 //  Serial.println(telemetryBatteryCurrent);
                                 //  Serial.print(">busCurrent_A:");
                                 //  Serial.println(telemetryBusCurrent);
                                 //  Serial.print(">externalPower_W:");
                                 //  Serial.println(telemetryExternalPower);
                                 //  Serial.print(">batteryPower_W:");
                                 //  Serial.println(telemetryBatteryPower);
                                 //  Serial.print(">busPower_W:");
                                 //  Serial.println(telemetryBusPower);
                                 //  Serial.print(">externalDieTemperature_C:");
                                 //  Serial.println(telemetryExternalDieTemperature);
                                 //  Serial.print(">batteryDieTemperature_C:");
                                 //  Serial.println(telemetryBatteryDieTemperature);
                                 //  Serial.print(">busDieTemperature_C:");
                                 //  Serial.println(telemetryBusDieTemperature);
                                 //  Serial.println();
                                 //  Serial.flush();
                             });

    updateDisplay();                             // ディスプレイの表示を更新します。
    Tasks.add(&taskGpsUpdate)->startFps(5);      // GPSデータを5秒に1回更新するタスクを登録します。
    Tasks.add(&taskButtonCheck)->startFps(20);   // ボタンの状態を20秒に1回チェックするタスクを登録します。
    Tasks.add("update-display", &updateDisplay); // ディスプレイを更新するタスクを登録します。
}

/**
 * @brief プログラムがずっと動き続けるためのメインループ関数です。
 * @details
 * この関数は、プログラムが動いている間、繰り返し実行されます。
 * ここでは、登録されたタスク（GPS更新やボタンチェックなど）を動かしたり、
 * LoRa無線で新しいメッセージが届いていないかを確認したりします。
 */
void loop()
{
    Tasks.update(); // 登録されたタスクを動かします。

    // LoRa無線で新しいメッセージが届いていたら
    if (LoRa.parsePacket())
    {
        MsgPacketizer::parse(); // そのメッセージを解析し、対応する処理（setup関数で登録した処理）を実行します。
    }
}
//------------------------------------------------------------
//    姿勢制御フィルタリングプログラム
//                Arduino　IDE　1.6.11
//
//　　　Arduino　　　　　　　　LSM9DS1基板　
//　　　　3.3V　　　------　　　　3.3V
//　　　　GND       ------   　　 GND
//　　　　SCL       ------        SCL
//　　　　SDA       ------        SDA
//
//　センサーで取得した値をシリアルモニターに表示する
//
//　　　　
//----------------------------------------------------------//


#include <SPI.h>                                //SPIライブラリ
#include <Wire.h>                               //I2Cライブラリ
#include <SparkFunLSM9DS1.h>                  //LSM9DS1ライブラリ：https://github.com/sparkfun/LSM9DS1_Breakout
#include <SD.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>



//#define ADAddr 0x48//

#define LSM9DS1_M  0x1E // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // SPIアドレス設定 if SDO_AG is LOW

//#define PRINT_CALCULATED //表示用の定義
//#define DEBUG_GYRO //ジャイロスコープの表示

//#define PRINT_SPEED 250 // 250 ms between prints
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//-------------------------------------------------------------------------
//Global valiables

LSM9DS1 imu;
int SAMPLETIME = 10;
int RECORD_INTERVAL = 100;
int WRITE_INTERVAL = 1000;
//MicroSD 
const int chipSelect = 4;//Arduino UNO
//const int chipSelect = 10;//Arduino Micro


//ジャイロセンサーの積分値
float pitch_g = 0.0;
float roll_g = 0.0;

//相補フィルタの保持値
float prev_pitch = 0.0;
float prev_roll = 0.0;
 
//----------------------------------------------------------------------
void setup(void) {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  /*while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }*/


  //Serial.begin(115200);                                 //シリアルモニタ通信速度設定


  //SD Card Initialize ====================================
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  //=======================================================

  //LSM9DS1 Initialize=====================================
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())                                     //センサ接続エラー時の表示

  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
    while (1)
      ;
  }
  //=======================================================
}

/**
 * loop
 * ずっと繰り返される関数（何秒周期？）
 * 【概要】
 * 　10msでセンサーデータをサンプリング。
 * 　記録用に、100ms単位でデータ化。
 * 　蓄積したデータをまとめて、1000ms単位でSDカードにデータを出力する。
 * 　
 */
void loop(void) {

  int t, t2;
  String record = "";

  for(t2 = 0; t2 < WRITE_INTERVAL;){
    for(t = 0; t < WRITE_INTERVAL / SAMPLETIME; t += SAMPLETIME){
      readGyro();
      readAccel();
      readMag();
  
      //空回りで、10msで値を更新しつづける    
      printAttitude (imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz), imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz) + "\n";
  
      delay(SAMPLETIME);
    }
    
    // 1/10秒のうち1/100秒で代表値を取得
    //記録用の値を取得
    record += printAttitude (imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz), imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz) + "\n";
    t2 += t;
  }
  

    
  //Write MicroSD =================================
    // make a string for assembling the data to log:
  //String dataString = "";

  // read three sensors and append to the string:
  /*for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }*/

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  /*File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(buffer);
    dataFile.close();
    // print to the serial port too:
    Serial.println(buffer);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
*/
  //======================================================
  Serial.print(record);
  Serial.println("================================");
  //delay(WRITE_INTERVAL);
}

//--------------------　Gyro DATA ------------------------------------
void readGyro()
{

  imu.readGyro();

#ifdef PRINT_CALCULATED
  Serial.print("G: ");
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");



#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif

}
//-------------------　Accel DATA ----------------------
void readAccel()
{
  // To read from the accelerometer, you must first call the
  // readAccel() function. When this exits, it'll update the
  // ax, ay, and az variables with the most current data.
  imu.readAccel();

#ifdef PRINT_CALCULATED
  Serial.print("A: ");

  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");




#elif defined PRINT_RAW
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}
//--------------　Mag DATA ------------------
void readMag()
{

  imu.readMag();
#ifdef PRINT_CALCULATED
  Serial.print("M: ");

  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
  
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif


}
//---------------------------------------------------------
/**
 * printAttitude
 * 取得したデータをシリアル出力する関数
 * gx : ジャイロスコープ X値
 * gy : ジャイロスコープ Y値
 * gz : ジャイロスコープ Z値
 * ax : 加速度センサー X値
 * ay : 加速度センサー Y値
 * az : 加速度センサー Z値
 * mx : 地磁気センサー X値
 * my : 地磁気センサー Y値
 * mz : 地磁気センサー Z値
 */

String printAttitude(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{

  String output = "";

  //重力加速度から求めた角度ををカルマンフィルタの初期値とする
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
    heading = (mx < 0) ? 180.0 : 0;
  else
    heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;


#ifdef DEBUG_GYRO
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  //Serial.print("Heading: ");
  //Serial.println(heading, 2);
#endif

  //*** Gyro ***
  float gyro_x =  gx * SAMPLETIME / 1000;
  float gyro_y = gy * SAMPLETIME / 1000;

#ifdef DEBUG_GYRO
  //ジャイロセンサーから求めた角度
  pitch_g = pitch_g + gyro_x;  
  roll_g = roll_g + gyro_y;

  Serial.print("PitchG, RollG: ");
  Serial.print(pitch_g, 2);
  Serial.print(", ");
  Serial.print(roll_g, 2);
  Serial.println("");
#endif

  //相補フィルタの出力
  prev_pitch = complementFilter( prev_pitch, gyro_x, pitch );
  prev_roll = complementFilter( prev_roll, gyro_y, roll );

  output = prev_pitch;
  output += ",";
  output += prev_roll;

#ifdef DEBUG_GYRO
  Serial.println("Filtered");
  Serial.print("Pitch, Roll: ");
  Serial.print(prev_pitch, 2);
  Serial.print(", ");
  Serial.print(prev_roll, 2);
  Serial.println("");
#endif

  return output;
}


/**
 * 相補フィルタ
 * prev_val : 前回のOutput
 * deg_g : ジャイロセンサで得た角度
 * deg_a : 加速度センサーで得た角度
 */
 float complementFilter(float prev_val, float deg_g, float deg_a){
    float output = 0.95 * (prev_val + deg_g) + 0.05 * deg_a;
    return output;
 }

/**
 * カルマンフィルタの計算式
 */
/*float Q_angle  =  0.001;
float Q_gyro   =  0.003;
float R_angle  =  0.03; 
float x_angle;
float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
float  y, S;
float K_0, K_1;
float dt=0.005;
float kal_deg;

float kalmanCalculate(float newAngle, float newRate){
                
        x_angle += dt * (newRate - x_bias);
       
        P_00 +=  dt * (dt * P_11 - P_01 - P_10 + Q_angle);
        P_01 -=  dt * P_11;
        P_10 -=  dt * P_11;
        P_11 +=  Q_gyro * dt;

        y = newAngle - x_angle;
        S = P_00 + R_angle;
        K_0 = P_00 / S;
        K_1 = P_10 / S;

        x_angle +=  K_0 * y;
        x_bias  +=  K_1 * y;
        P_00 -= K_0 * P_00;
        P_01 -= K_0 * P_01;
        P_10 -= K_1 * P_00;
        P_11 -= K_1 * P_01;

    return x_angle;
}*/

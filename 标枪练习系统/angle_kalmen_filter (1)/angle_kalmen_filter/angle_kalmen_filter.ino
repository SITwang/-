#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <cmath>

// WiFi Credentials
const char *ssid = "esp32s3";            // Replace with your WiFi name
const char *password = "12345678";  // Replace with your WiFi password

// MQTT Broker Settings
// MQTT Broker Settings
const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_topic = "test/esp32";
const char *mqtt_username = "esp32";
const char *mqtt_password = "esp32esp32";
const int mqtt_port = 1883;

Adafruit_MPU6050 mpu;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
/*计算偏移量*/
float i;                                    //计算偏移量时的循环次数
float ax_offset = 0, ay_offset = 0;         //x,y轴的加速度偏移量
float gx_offset = 0, gy_offset = 0;         //x,y轴的角速度偏移量

/*参数*/
float rad2deg = 57.29578;                   //弧度到角度的换算系数
float roll_v = 0, pitch_v = 0;              //换算到x,y轴上的角速度

/*定义微分时间*/
float now = 0, lasttime = 0, dt = 0;        //定义微分时间

/*三个状态，先验状态，观测状态，最优估计状态*/
float gyro_roll = 0, gyro_pitch = 0;        //陀螺仪积分计算出的角度，先验状态
float acc_roll = 0, acc_pitch = 0;          //加速度计观测出的角度，观测状态
float k_roll = 0, k_pitch = 0;              //卡尔曼滤波后估计出最优角度，最优估计状态

/*误差协方差矩阵P*/
float e_P[2][2] ={{1,0},{0,1}};             //误差协方差矩阵，这里的e_P既是先验估计的P，也是最后更新的P

/*卡尔曼增益K*/
float k_k[2][2] ={{0,0},{0,0}};             //这里的卡尔曼增益矩阵K是一个2X2的方阵
// read the analog / millivolts value for pin 2:
  //int analogValue;
void setup(void) {
  /*打开串口和实现I2C通信*/
  Wire.begin(27, 26);//SDA->23,SCL->5,可以根据情况自行修改

  //打开串口
  Serial.begin(115200);//串口波特率
  delay(100);
  //set the resolution to 12 bits (0-4096)
 
  pinMode(22, OUTPUT);	
	digitalWrite(22, LOW);
  connectToWiFi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setKeepAlive(60);
  mqtt_client.setCallback(mqttCallback); // Corrected callback function name
  connectToMQTT();
  /*判断是否连接到MPU6050并且初始化*/
  while (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);//加速度量程±2G
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);//角速度量程±250°/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);//采样频率21Hz

  //计算偏移量
  for (i = 1; i <= 2000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);//获取加速度、角速度、温度
    ax_offset = ax_offset + a.acceleration.x;//计算x轴加速度的偏移总量
    ay_offset = ay_offset + a.acceleration.y;//计算y轴加速度的偏移总量
    gx_offset = gx_offset + g.gyro.x;
    gy_offset = gy_offset + g.gyro.y;
  }
  ax_offset = ax_offset / 2000; //计算x轴加速度的偏移量
  ay_offset = ay_offset / 2000; //计算y轴加速度的偏移量
  gx_offset = gx_offset / 2000; //计算x轴角速度的偏移量
  gy_offset = gy_offset / 2000; //计算y轴角速度的偏移量
  delay(1000);
  analogReadResolution(12);
}
  /*获取角速度和加速度 */
  sensors_event_t a, g, temp;
  int time1;
void loop() {
  char rollStr[10],pitchStr[10],accStr[10];
  
  if (!mqtt_client.connected()) {
        connectToMQTT();
    }
    mqtt_client.loop();
  /*计算微分时间*/
  now = millis();                           //当前时间(ms)
  dt = (now - lasttime) / 1000.0;           //微分时间(s)
  lasttime = now;                           //上一次采样时间(ms)
  mpu6050();
  delay(5);
  int analogValue = analogRead(34);
 // int analogVolts = analogReadMilliVolts(25);
  if(analogValue>100){
    while(analogValue>100){
      analogValue = analogRead(34);
       Serial.printf("ADC analog value = %d\n",analogValue);
      mpu6050();

    }
   sendData();
  }
  
  time1++;
  if(time1%50==0){
    //打印角度
  Serial.print("roll: ");
  Serial.print(k_roll);
  Serial.print(",");
  Serial.print("pitch: ");
  Serial.println(k_pitch);
  Serial.print("x轴加速度: ");
  Serial.print(fabs(a.acceleration.y - ax_offset) );
  Serial.print(",");
  Serial.print("y轴加速度: ");
  Serial.println(fabs(a.acceleration.y - ay_offset));
  //姿态可视化
//  Serial.print(k_roll);
//  Serial.print("/");
//  Serial.println(k_pitch);
  }
  
}

void sendData() {
  // 构建 JSON 格式的数据字符串
  String payload = "{";
  payload += "\"roll\":" + String(k_roll, 2) + ",";
  payload += "\"pitch\":" + String(k_pitch, 2) + ",";
  payload += "\"acceleration\":" + String(fabs(a.acceleration.x - ax_offset), 2);//x轴加速度和pitch角 或y轴加速度和roll角
  payload += "}";

  // 打印发送的数据，便于调试
  Serial.println("Sending data: ");
  Serial.println(payload);

  // 发送 MQTT 消息
  mqtt_client.publish(mqtt_topic, payload.c_str());
}

void mpu6050(){
  mpu.getEvent(&a, &g, &temp); // 获取加速度、角速度、温度

  /*step1:计算先验状态*/
  /*计算x,y轴上的角速度*/
  roll_v = (g.gyro.x-gx_offset) + ((sin(k_pitch)*sin(k_roll))/cos(k_pitch))*(g.gyro.y-gy_offset) + ((sin(k_pitch)*cos(k_roll))/cos(k_pitch))*g.gyro.z; // roll轴的角速度
  pitch_v = cos(k_roll)*(g.gyro.y-gy_offset) - sin(k_roll)*g.gyro.z; // pitch轴的角速度
  gyro_roll = k_roll + dt*roll_v; // 先验roll角度
  gyro_pitch = k_pitch + dt*pitch_v; // 先验pitch角度

  /*step2:计算先验误差协方差矩阵P*/
  e_P[0][0] = e_P[0][0] + 0.0025; // Q矩阵的对角阵元素
  e_P[0][1] = e_P[0][1] + 0;
  e_P[1][0] = e_P[1][0] + 0;
  e_P[1][1] = e_P[1][1] + 0.0025;

  /*step3:更新卡尔曼增益K*/
  k_k[0][0] = e_P[0][0] / (e_P[0][0] + 0.3);
  k_k[0][1] = 0;
  k_k[1][0] = 0;
  k_k[1][1] = e_P[1][1] / (e_P[1][1] + 0.3);

  /*step4:计算最优估计状态*/
  /*观测状态*/
  // 计算roll角度并规范化到0-360度范围内
  acc_roll = atan((a.acceleration.y - ay_offset) / (a.acceleration.z)) * rad2deg;
  acc_roll = normalize_angle(acc_roll);  // 规范化roll角度

  // 计算pitch角度并规范化到0-360度范围内
  acc_pitch = -1 * atan((a.acceleration.x - ax_offset) / sqrt(sq(a.acceleration.y - ay_offset) + sq(a.acceleration.z))) * rad2deg;
  acc_pitch = normalize_angle(acc_pitch);  // 规范化pitch角度

  /*最优估计状态*/
  k_roll = gyro_roll + k_k[0][0] * (acc_roll - gyro_roll);
  k_pitch = gyro_pitch + k_k[1][1] * (acc_pitch - gyro_pitch);

  /*step5:更新协方差矩阵P*/
  e_P[0][0] = (1 - k_k[0][0]) * e_P[0][0];
  e_P[0][1] = 0;
  e_P[1][0] = 0;
  e_P[1][1] = (1 - k_k[1][1]) * e_P[1][1];
}

void connectToWiFi() {
   WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi");
}

void connectToMQTT() {
    while (!mqtt_client.connected()) {
        String client_id = "esp32-client-" + String(WiFi.macAddress());
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
            mqtt_client.subscribe(mqtt_topic);
            mqtt_client.publish(mqtt_topic, "Hi EMQX I'm ESP32 ^^"); // Publish message upon successful connection
        } else {
            Serial.print("Failed, rc=");
            Serial.print(mqtt_client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void mqttCallback(char *mqtt_topic, byte *payload, unsigned int length) {
    Serial.print("Message received on mqtt_topic: ");
    Serial.println(mqtt_topic);
    Serial.print("Message: ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char) payload[i]);
    }
    
    Serial.println("\n-----------------------");
    if((char) payload[0]=='1'){
      Serial.println("receive data");
    }
}

float normalize_angle(float angle) {
    // 先将角度限制在0到360之间
    angle = fmod(angle, 360);
    // Serial.print("angle:");
    // Serial.println(angle);
    if (angle < 0) {
        angle += 360; // 如果是负角度，加上360
    }

    // 将角度限制在0到180度之间
    if (angle > 180) {
        angle = 360 - angle; // 如果角度大于180，将其转换到0到180之间
    }

    return angle;
}


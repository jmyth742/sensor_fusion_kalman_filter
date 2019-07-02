#include <ArduinoJson.h>
#include <MatrixMath.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#define left_wheel 15
#define right_wheel 13
#define INTERVAL_MESSAGE1 500
#define forward 1
#define right 2
#define left 3
#define stopped 4
unsigned long time_1 = 0;


/* B             D
      in
       *
          fin
           *
   A(0)          C  */

MPU6050 mpu;
int SCL_PIN=D1;
int SDA_PIN=D2;
const int MPU_addr=0x68;
const char* ssid     = "home";
const char* password = "fiap4202";

//const char* mqttServer = "192.168.0.105";
const char* mqttServer = "192.168.43.112";
const int mqttPort = 1883;
const char* mqttUser = "username";
const char* mqttPassword = "password";
boolean camera_flag ;

WiFiClient espClient;
PubSubClient client(espClient);

double calibrssi ;
///             Need to be filled up ///////
double AB=2.5;double AC=2;//double BD;double CD;
////////////////////////////////////////////////////////////////////////
double da = 0;double db = 0;double dc = 0;double dd = 0;
String a ="C"; String b ="B"; String c ="home"; //String d ="D";
String myssid= "C";int incomingByte = 0;

unsigned long now, lastTime = 0;
int16_t ax, ay, az, gx, gy, gz;             // Accelerometer gyroscope raw data
double aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    // Angle variable
double roll=0, pitch=0, yaw=0;
long axo = 0, ayo = 0, azo = 0;             // Accelerometer offset
long gxo = 0, gyo = 0, gzo = 0;             // Gyro offset
int incr =0; String command;
int y=0;
long choice; boolean flag = true;

double distx=0;
double velx=0;

double pi = 3.1415926;
double AcceRatio = 16384.0;            // Accelerometer scale factor (to convert data form g to in m/s^2)
//double AcceRatio = 1670.13;   
double GyroRatio = 131.0;                    // Gyroscope scale factor (to convert data in m/s)



/////////////////////      Kalman    Variables      ////////////////

//sigma variables

double sigma2imuacc=0.002642197;
double sigma2imuomega=3.52855e-6;

double sigma2wifi1node=0.074310442; //NODE A
double sigma2wifi2node=0.031153347; //NODE B
double sigma2wifi3node=0.055833536; //NODE C

//measures variables
double omegaimu;
double accimu;
double ximu, yimu, thetaimu, vimu;
double xwifi, ywifi, thetawifi;
double rho1, rho2, rho3;
double dt;
double xrobot;
double yrobot;
double thetarobot;

//node positions and rho
double nodepos[3][2];
double rhopred[3][1];

//matrices initialization

double state[4][1];
double Pk[4][4];
double A[4][4];
double B[4][2];
double u[2][1];
double H[3][4];
double Qimu[2][2];
double R[3][3];
double z[3][1];

double I[4][4];

//matrices for kalman calculations inside the loop

double stateknext[4][1];
double Pkpiu1[4][4];

double savestate[4][1];
double savePk[4][4];




int i=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

   Wire.begin();
  Serial.begin(115200);

  pinMode(right_wheel, OUTPUT);
  pinMode(left_wheel, OUTPUT);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
      Serial.println("connected");yield();
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());yield();
      delay(2000);
    }
  }
  client.publish("test", "we are testing the comms");
  client.subscribe("positions");
 
  Serial.println("Initialize MPU6050");
  while(!mpu.beginSoftwareI2C(SCL_PIN,SDA_PIN,MPU6050_SCALE_250DPS , MPU6050_RANGE_2G,MPU_addr))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
    }

    // Calibration
    unsigned short times = 1000;             // The number of samples
    for(int i=0;i<times;i++)
    {
        Vector RawAccel = mpu.readRawAccel();
        Vector RawGyro = mpu.readRawGyro();
        //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read the six-axis original value
        ax = RawAccel.XAxis;  ay = RawAccel.YAxis;  az = RawAccel.ZAxis;
        gx = RawGyro.XAxis;  gy = RawGyro.YAxis;  gz = RawGyro.ZAxis;
        axo += ax; ayo += ay; azo += az;      // Sampling
        gxo += gx; gyo += gy; gzo += gz;
    }
    axo /= times; ayo /= times; azo /= times; // Calculate accelerometer offset
    gxo /= times; gyo /= times; gzo /= times; // Calculate the gyro offset


   ////////////////////////// Kalman stuff /////////////////////////////////////////////////////////////

  //Variables initialization
  rho1=0;
  rho2=0;
  rho3=0;

  ximu=0;
  yimu=0;
  thetaimu=0;
  vimu=1;
  accimu=0;
  omegaimu=0;
  
 
  //State initialization
  state[0][0]=ximu;
  state[1][0]=yimu;
  state[2][0]=vimu;
  state[3][0]=thetaimu;
  //Matrix.Print((double*)state, 4, 1, "state");

  //A matrix initialization   
  A[0][0]=1; A[0][1]=0; A[0][2]=cos(state[3][0])*dt; A[0][3]=0;
  A[1][0]=0; A[1][1]=1; A[1][2]=sin(state[3][0])*dt; A[1][3]=0;  
  A[2][0]=0; A[2][1]=0; A[2][2]=1; A[2][3]=0;
  A[3][0]=0; A[3][1]=0; A[3][2]=0; A[3][3]=1;
  //Matrix.Print((double*)A, 4, 4, "A");

  //B matrix initialization
  B[0][0]=0; B[0][1]=0;
  B[1][0]=0; B[1][1]=0;
  B[2][0]=dt; B[2][1]=0; 
  B[3][0]=0; B[3][1]=dt;
  //Matrix.Print((double*)B, 4, 2, "B");
  
  //input vector u initialization
  u[0][0]=accimu;
  u[1][0]=omegaimu;
  //Matrix.Print((double*)u, 2, 1, "u");

  //node position
  nodepos[0][0]=0.0000;  nodepos[0][1]=0.0000;
  nodepos[1][0]=0.0000;  nodepos[1][1]=2.5000;
  nodepos[2][0]=2.0000;  nodepos[2][1]=0.0000;
  //Matrix.Print((double*)nodepos, 3, 2, "nodepos");

  //rhopred 
  rhopred[0][0]=0;
  rhopred[1][0]=0;
  rhopred[2][0]=0;
  //Matrix.Print((double*)rhopred, 3, 1, "rhopred");
  
  //H matrix
  H[0][0]=0.001; H[0][1]=0.001; H[0][2]=0; H[0][3]=0;
  H[1][0]=0.001; H[1][1]=0.001; H[1][2]=0; H[1][3]=0;
  H[2][0]=0.001; H[2][1]=0.001; H[2][2]=0; H[2][3]=0;
  //Matrix.Print((double*)H, 3, 4, "H");

  //Qimu  
  Qimu[0][0]=sigma2imuacc; Qimu[0][1]=0;
  Qimu[1][0]=0; Qimu[1][1]=sigma2imuomega;
  //Matrix.Print((double*)Qimu, 2, 2, "Qimu");

  //P matrix
  Pk[0][0]=0.1;
  Pk[0][1]=0.1;
  Pk[0][2]=0.1;
  Pk[0][3]=0.1;
  Pk[1][0]=0.1;
  Pk[1][1]=0.1;
  Pk[1][2]=0.1;
  Pk[1][3]=0.1;
  Pk[2][0]=0.1;
  Pk[2][1]=0.1;
  Pk[2][2]=0.1;
  Pk[2][3]=0.1;
  Pk[3][0]=0.1;
  Pk[3][1]=0.1;
  Pk[3][2]=0.1;
  Pk[3][3]=0.1;
  //Matrix.Print((double*)Pk, 4, 4, "Pk");

  //Identity Matrix
  I[0][0]=1;
  I[0][1]=0;
  I[0][2]=0;
  I[0][3]=0;
  I[1][0]=0;
  I[1][1]=1;
  I[1][2]=0;
  I[1][3]=0;
  I[2][0]=0;
  I[2][1]=0;
  I[2][2]=1;
  I[2][3]=0;
  I[3][0]=0;
  I[3][1]=0;
  I[3][2]=0;
  I[3][3]=1;

  //R wifi matrix
  R[0][0]=sigma2wifi1node; R[0][1]=0; R[0][2]=0;
  R[1][0]=0; R[1][1]=sigma2wifi2node; R[1][2]=0;
  R[2][0]=0; R[2][1]=0; R[2][2]=sigma2wifi3node;
  //Matrix.Print((double*)R, 3, 3, "R");
  
  //z camera measures
  z[0][0]=rho1;
  z[1][0]=rho2;
  z[2][0]=rho3;
  //Matrix.Print((double*)z, 3, 1, "z");

  //INITIALIZATION 
  Matrix.Copy((double*)state, 4, 1, (double*)savestate);
  Matrix.Copy((double*)Pk, 4, 4, (double*)savePk);


  ///////////// initial position and orientation /////////////

      /// Getting wifi x and y and theta ///////////
    
    getOrientation();
    getPos();

      state[0][0]=xwifi;
      state[1][0]=ywifi;
      state[2][0]=vimu;
      state[3][0]=thetawifi;

    //Serial.print(xwifi,10);Serial.print("     ");Serial.print(ywifi,10);Serial.print("     ");Serial.println(thetawifi,10);

    //Serial.println();
    //Serial.println();

          String message="";
      message = String(xwifi,8) + " " + String(ywifi,8) + " " + String(thetawifi,8);
      char copy [message.length()];
      message.toCharArray(copy, message.length());
      
      client.publish("kalman", copy);
      Serial.println(copy);
      //camera_flag = false;
  

}

void loop() {
  
  client.loop();
  kalman();
  
  //circlemove();

}

int count=0;

void kalman(){
  //matrices for calculations
double Astate[4][1]; 
double Bu[4][1]; 
double statekpiu1meno[4][1];
double Atranspose[4][4];
double APk[4][4];
double APkAtr[4][4];
double Btranspose[2][4];
double BQimu[4][2];
double BQimuBtr[4][4];
double Pkpiu1meno[4][4];
double Htranspose[4][3];
double HPk[3][4];
double HPkHtr[3][3];
double Skpiu1[3][3];
double PkHtr[4][3];      
double Skpiu1inv[3][3];
double Wkpiu1[4][3];
double zminusrhopred[3][1];
double WkzminusH[4][1];

double WkH[4][4];
double IminusWkH[4][4];



    unsigned long now = millis();
    dt = (now - lastTime)/ 1000.0;           // Differential time(s)
    lastTime = now;  

    //taking new measures
    Vector RawGyro = mpu.readRawGyro();
    Vector RawAccel = mpu.readRawAccel();
    ax = RawAccel.XAxis;  ay = RawAccel.YAxis;  az = RawAccel.ZAxis;
    gx = RawGyro.XAxis;  gy = RawGyro.YAxis;  gz = RawGyro.ZAxis;
    
    omegaimu=((gz-gzo)/GyroRatio)/360*2*pi;
    accimu=(ax-axo)/AcceRatio;
    
    //Serial.print(omegaimu,6);Serial.print("    ");Serial.print(accimu,6);Serial.println();

count++;
if(count==0){
        rho1=calculateDistance((double)checkRSSI(a));
        rho2=calculateDistance((double)checkRSSI(b));
        rho3=calculateDistance((double)checkRSSI(c));
}
if((count%100)==0){
        rho1=calculateDistance((double)checkRSSI(a));
        rho2=calculateDistance((double)checkRSSI(b));
        rho3=calculateDistance((double)checkRSSI(c));
}
    //updating state and Pk
    Matrix.Copy((double*)savestate, 4, 1, (double*)state);
    Matrix.Copy((double*)savePk, 4, 4, (double*)Pk);

    //updating matrices

    A[0][2]=cos(state[3][0])*dt;
    A[1][2]=sin(state[3][0])*dt;
    
    B[2][0]=dt;
    B[3][1]=dt;

    u[0][0]=omegaimu;
    u[1][0]=accimu;

    z[0][0]=rho1;
    z[1][0]=rho2;
    z[2][0]=rho3;

    //PREDICTION STEP
    
    //First row statekpiu1meno

    //Matrix.Print((double*)A, 4, 4, "A");
    //Matrix.Print((double*)B, 4, 2, "B");
    //Matrix.Print((double*)u, 2, 1, "u");
    
    Matrix.Multiply((double*) A, (double*) state, 4, 4, 1, (double*) Astate);
    //Matrix.Print((double*)Astate, 4, 1, "Astate");
    
    Matrix.Multiply((double*) B, (double*) u, 4, 2, 1, (double*) Bu);
    //Matrix.Print((double*)Bu, 4, 1, "Bu");
    
    Matrix.Add((double*) Astate, (double*) Bu, 4, 1, (double*) statekpiu1meno);
    //Matrix.Print((double*)statekpiu1meno, 4, 1, "statekpiu1meno");
    

    //Second row Pkpiu1meno
    
    Matrix.Transpose((double*) A, 4, 4, (double*) Atranspose);
    //Matrix.Print((double*)Atranspose, 4, 4, "Atranspose");
    
    Matrix.Transpose((double*) B, 4, 2, (double*) Btranspose);
    //Matrix.Print((double*)Btranspose, 2, 4, "Btranspose");

    Matrix.Multiply((double*) A, (double*) Pk, 4, 4, 4, (double*) APk);
    //Matrix.Print((double*)APk, 4, 4, "APk");
    
    Matrix.Multiply((double*) APk, (double*) Atranspose, 4, 4, 4, (double*) APkAtr);
    //Matrix.Print((double*)APkAtr, 4, 4, "APkAtr");

    Matrix.Multiply((double*) B, (double*) Qimu, 4, 2, 2, (double*) BQimu);
    //Matrix.Print((double*)BQimu, 4, 2, "BQimu");
    
    Matrix.Multiply((double*) BQimu, (double*) Btranspose, 4, 2, 4, (double*) BQimuBtr);
    //Matrix.Print((double*)BQimuBtr, 4, 4, "BQimuBtr");

    Matrix.Add((double*) APkAtr, (double*) BQimuBtr, 4, 4, (double*) Pkpiu1meno);
    //Matrix.Print((double*)Pkpiu1meno, 4, 4, "Pkpiu1meno");


    //UPDATE STEP

    //rhopred 

    rhopred[0][0]=(sqrt(((statekpiu1meno[0][0]-nodepos[0][0])*(statekpiu1meno[0][0]-nodepos[0][0]))+((statekpiu1meno[1][0]-nodepos[0][1])*(statekpiu1meno[1][0]-nodepos[0][1]))));
    rhopred[1][0]=(sqrt(((statekpiu1meno[0][0]-nodepos[1][0])*(statekpiu1meno[0][0]-nodepos[1][0]))+((statekpiu1meno[1][0]-nodepos[1][1])*(statekpiu1meno[1][0]-nodepos[1][1]))));
    rhopred[2][0]=(sqrt(((statekpiu1meno[0][0]-nodepos[2][0])*(statekpiu1meno[0][0]-nodepos[2][0]))+((statekpiu1meno[1][0]-nodepos[2][1])*(statekpiu1meno[1][0]-nodepos[2][1]))));

    //Matrix.Print((double*)rhopred, 3, 1, "rhopred");   
  
    //H matrix

    H[0][0]=(statekpiu1meno[0][0]-nodepos[0][0])/rhopred[0][0]; H[0][1]=(statekpiu1meno[1][0]-nodepos[0][0])/rhopred[0][0]; H[0][2]=0; H[0][3]=0;
    H[1][0]=(statekpiu1meno[0][0]-nodepos[1][0])/rhopred[1][0]; H[1][1]=(statekpiu1meno[1][0]-nodepos[1][0])/rhopred[1][0]; H[1][2]=0; H[1][3]=0;
    H[2][0]=(statekpiu1meno[0][0]-nodepos[2][0])/rhopred[2][0]; H[2][1]=(statekpiu1meno[1][0]-nodepos[2][0])/rhopred[2][0]; H[2][2]=0; H[2][3]=0;

    //Matrix.Print((double*)H, 3, 4, "H"); 

    
    //Third row Skpiu1
    
    Matrix.Transpose((double*) H, 3, 4, (double*) Htranspose);
    //Matrix.Print((double*)Htranspose, 4, 3, "Htranspose");
    
    Matrix.Multiply((double*) H, (double*) Pkpiu1meno, 3, 4, 4, (double*) HPk);
    //Matrix.Print((double*)HPk, 3, 4, "HPk");
    
    Matrix.Multiply((double*) HPk, (double*) Htranspose, 3, 4, 3, (double*) HPkHtr);
    //Matrix.Print((double*)HPkHtr, 3, 3, "HPkHtr");

    Matrix.Add((double*) HPkHtr, (double*) R, 3, 3, (double*) Skpiu1);
    //Matrix.Print((double*)Skpiu1, 3, 3, "Skpiu1");
    

    //Fourth row Wkpiu1
       
    Matrix.Multiply((double*) Pkpiu1meno, (double*) Htranspose, 4, 4, 3, (double*) PkHtr);
    //Matrix.Print((double*)PkHtr, 4, 3, "PkHtr");



Skpiu1inv[0][0] = (Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[1][2] * Skpiu1[2][1]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[0][1] = (-Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[0][2] * Skpiu1[2][1]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[0][2] = (Skpiu1[0][1] * Skpiu1[1][2] - Skpiu1[0][2] * Skpiu1[1][1]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[1][0] = (-Skpiu1[1][0] * Skpiu1[2][2] + Skpiu1[1][2] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[1][1] = (Skpiu1[0][0] * Skpiu1[2][2] - Skpiu1[0][2] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[1][2] = (-Skpiu1[0][0] * Skpiu1[1][2] + Skpiu1[0][2] * Skpiu1[1][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[2][0] = (Skpiu1[1][0] * Skpiu1[2][1] - Skpiu1[1][1] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[2][1] = (-Skpiu1[0][0] * Skpiu1[2][1] + Skpiu1[0][1] * Skpiu1[2][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);
Skpiu1inv[2][2] = (Skpiu1[0][0] * Skpiu1[1][1] - Skpiu1[0][1] * Skpiu1[1][0]) / (Skpiu1[0][0] * Skpiu1[1][1] * Skpiu1[2][2] - Skpiu1[0][0] * Skpiu1[1][2] * Skpiu1[2][1] - Skpiu1[1][0] * Skpiu1[0][1] * Skpiu1[2][2] + Skpiu1[2][0] * Skpiu1[0][1] * Skpiu1[1][2] + Skpiu1[1][0] * Skpiu1[0][2] * Skpiu1[2][1] - Skpiu1[2][0] * Skpiu1[0][2] * Skpiu1[1][1]);



    

    //Matrix.Copy((double*)Skpiu1, 3, 3, (double*)Skpiu1inv);
    //Matrix.Print((double*)Skpiu1inv, 3, 3, "Skpiu1inv");
    
    //Matrix.Invert((double*) Skpiu1inv, 3);
    //Matrix.Print((double*)Skpiu1inv, 3, 3, "Skpiu1inv");
    
    Matrix.Multiply((double*) PkHtr, (double*) Skpiu1inv, 4, 3, 3, (double*) Wkpiu1);
    //Matrix.Print((double*)Wkpiu1, 4, 3, "Wkpiu1");

 
    //Fifth row stateknext

    Matrix.Subtract((double*) z, (double*) rhopred, 3, 1, (double*) zminusrhopred);
    //Matrix.Print((double*)zminusrhopred, 3, 1, "zminusrhopred");
    
    Matrix.Multiply((double*) Wkpiu1, (double*) zminusrhopred, 4, 3, 1, (double*) WkzminusH);
    //Matrix.Print((double*)WkzminusH, 4, 1, "WkzminusH");

    Matrix.Add((double*) statekpiu1meno, (double*) WkzminusH, 4, 1, (double*) stateknext);
    //Matrix.Print((double*)stateknext, 4, 1, "stateknext");
    

    //Sixth row Pkpiu1
   
    Matrix.Multiply((double*) Wkpiu1, (double*) H, 4, 3, 4, (double*) WkH);
    //Matrix.Print((double*)WkH, 4, 4, "WkH");
    
    Matrix.Subtract((double*) I, (double*) WkH, 4, 4, (double*) IminusWkH);
    //Matrix.Print((double*)IminusWkH, 4, 4, "IminusWkH");
    
    Matrix.Multiply((double*) IminusWkH, (double*) Pkpiu1meno, 4, 4, 4, (double*) Pkpiu1);
    //Matrix.Print((double*)Pkpiu1, 4, 4, "Pkpiu1");

    //saving the state
    Matrix.Copy((double*)stateknext, 4, 1, (double*)savestate);  
    //Saving the matrix P
    Matrix.Copy((double*)Pkpiu1, 4, 4, (double*)savePk);


    double xrobot=stateknext[0][0];
    double yrobot=stateknext[1][0];
    double thetarobot=stateknext[3][0];

    //thetarobot=thetarobot*360/(2*pi);

    //Serial.print(xrobot,10);Serial.print("     ");Serial.print(yrobot,10);Serial.print("     ");Serial.println(thetarobot,10);

    //Serial.println();
    //Serial.println();

      String message="";
      message = String(xrobot,8) + " " + String(yrobot,8) + " " + String(thetarobot,8);
      char copy [message.length()];
      message.toCharArray(copy, message.length());
      
      client.publish("kalman", copy);
      Serial.println(copy);
      //camera_flag = false;
    

}

///////////// Wifi Functions    ////////////////////
double calculateDistance(double rssi)
{
  double txPower = -50.57;
  double n = 2;
  return (double)pow(10.0, ((txPower - rssi) / (10 * n)));
}


void callback(char* topic, byte* payload, unsigned int length) {
  
  Serial.print("Message arrived in topic: ");
  String input;
  //Serial.println(topic);
  //Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    //Serial.write((char)payload[i]);
    //Serial.print((char)payload[i]);
    input +=(char)payload[i];
  }
  
  //Serial.println();
  Serial.println(input);
  DynamicJsonBuffer jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(input);
  String json;
  //xcam = root["x"];
 // ycam = root["y"];
 // thetacam = root["theta"];
  //camera_flag = true;
  
}

void getPos()
{ 
  da = calculateDistance((double)checkRSSI(a));
  db = calculateDistance((double)checkRSSI(b));
  dc = calculateDistance((double)checkRSSI(c));
  
  ywifi=(1/(2*AB))*sqrt(abs((db+da+AB)*(-db+da+AB)*(-da+db+AB)*(-AB+da+db)));
  xwifi =(1/(2*AC))*sqrt(abs((dc+da+AC)*(-dc+da+AC)*(-da+dc+AC)*(-AC+da+dc)));
}

void getOrientation(){

  double  yin=0;
  double  xin=0;
  double  xfin=0;
  double  yfin=0;
  double  dist=0;
  double  sinalpha=0;
  double  cosalpha=0;
  double alpha=0; //orientation
  String fv = "First Values";
  String lv = "Last Values";
  
  double ain=calculateDistance((double)checkRSSI(a));
  double bin=calculateDistance((double)checkRSSI(b));
  double cin=calculateDistance((double)checkRSSI(c));
  
  yin=(1/(2*AB))*sqrt(abs((bin+ain+AB)*(-bin+ain+AB)*(-ain+bin+AB)*(-AB+ain+bin)));
  xin=(1/(2*AC))*sqrt(abs((cin+ain+AC)*(-cin+ain+AC)*(-ain+cin+AC)*(-AC+ain+cin)));

  /// need to do some moves ///////////
  gostraight();
  delay(500);
  block();
  delay(100);
  ///////////////////////////////////
  
  double afin=calculateDistance((double)checkRSSI(a));
  double bfin=calculateDistance((double)checkRSSI(b));
  double cfin=calculateDistance((double)checkRSSI(c));
  yfin=(1/(2*AB))*sqrt(abs((bfin+afin+AB)*(-bfin+afin+AB)*(-afin+bfin+AB)*(-AB+afin+bfin)));
  xfin=(1/(2*AC))*sqrt(abs((cfin+afin+AC)*(-cfin+afin+AC)*(-afin+cfin+AC)*(-AC+afin+cfin)));
  alpha=atan2(yfin-yin,xfin-xin);
  //Serial.println(alpha);
  thetawifi = alpha;
}

// Return RSSI or 0 if target SSID not found
int32_t checkRSSI(String target_ssid) {
  byte available_networks = WiFi.scanNetworks();
  for (int network = 0; network < available_networks; network++) {
    if (WiFi.SSID(network).equals(target_ssid)){
      return WiFi.RSSI(network);
      }
    }
    return 0;
    }

// Return all RSSI 
int32_t getAllRSSI() {
  byte available_networks = WiFi.scanNetworks();
  for (int network = 0; network < available_networks; network++) {
    return WiFi.RSSI(network);
    
  }
  return 0;
}

void calibrateRSSI( int times, float distance){
  Serial.println("Calibration starting ");
  for ( int i=0;i<times;i++){
    Serial.println(i);
    if(checkRSSI(myssid) == 0){
      Serial.println("Not Found");
    }
    else{
      calibrssi+= double( checkRSSI(myssid)); 
    }
  }
  calibrssi /= times;
  Serial.println("");
  Serial.print("For distance ");Serial.println(distance);
  Serial.print(" Calibrated RSSI is ");Serial.println(calibrssi);
  Serial.println("Calibration finished ");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////  Move Functions      ////////////////////////////////////




//////////// Moving Functions /////////////
void turnright()
{
      digitalWrite(right_wheel, LOW);
      digitalWrite(left_wheel, HIGH); 
}
void turnleft()
{
      digitalWrite(right_wheel, HIGH);
      digitalWrite(left_wheel, LOW);   
}
void block()
{
      digitalWrite(right_wheel, LOW);
      digitalWrite(left_wheel, LOW);    
}
void gostraight()
{
      digitalWrite(right_wheel, HIGH);
      digitalWrite(left_wheel,  HIGH);

}


void circlemove()
{ 
    
    if(incr<=100)
    {
      gostraight();
      //command = "1";
      //Serial.println(command);
      incr++;
    }

       
    if(incr>100 && incr<=150)
    {
      block();
      //command = "2";
     //Serial.println(command);
      incr++;
    }

       
    if(incr>150 && incr<=250)         
    {
      turnleft();
       //     command = "3";
      //Serial.println(command);
       incr++;
    }

        if(incr>250 &&  incr<=450)
    {
      block();
       //     command = "4";
     // Serial.println(command);
       incr++;
    }
        if( incr>4500)
    {
         //   command = "5";
     // Serial.println(command);
      incr=0;  
    }
}


void stayinside()
{ 
    // left edge
    if(xrobot<0.2 && cos(thetarobot)<0.5 && sin(thetarobot)<0)
    {turnleft();}
    if(xrobot<0.2 && cos(thetarobot)<0.5 && sin(thetarobot)>0)
    {turnright();}

    //right edge
    if(xrobot>1.8 && cos(thetarobot)>-0.5 && sin(thetarobot)<0)
    {turnright();}
    if(xrobot>1.8 && cos(thetarobot)>-0.5 && sin(thetarobot)>0)
    {turnleft();}

    //lower edge
    if(yrobot<0.2 && cos(thetarobot)<0 && sin (thetarobot)<0.5)
    {turnright();}
    if(yrobot<0.2 && cos(thetarobot)>0 && sin (thetarobot)<0.5)
    {turnleft();}

    //upper edge
    if(yrobot>2.3 && cos(thetarobot)<0 && sin (thetarobot)>-0.5)
    {turnleft();}
    if(yrobot>2.3 && cos(thetarobot)>0 && sin (thetarobot)>-0.5)
    {turnright();}
}


void moveAround(){
  // Serial.println("paok");
   if( flag){
    choice= random(1, 4);
    flag = false;
   }
    if(millis() >time_1+INTERVAL_MESSAGE1){
        choice= random(1, 5);
        time_1 = millis();
        Serial.println(choice);
        Serial.println(time_1/1000);
    }
    switch (choice) {
    case 1:
    //checkBorders();
    gostraight();
      break;
    case 2:
    turnright();
      break;
    case 3:
    turnleft;
      break;
    case 4:
    block;
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

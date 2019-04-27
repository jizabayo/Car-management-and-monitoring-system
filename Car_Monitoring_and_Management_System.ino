/****************** References ****************************************
  Author: Jean Paul Izabayo, Carnegie Mellon University Africa
  April 21, 2019
************************************************************************************/

/****************** References ****************************************
  https://lastminuteengineers.com/mq2-gas-senser-arduino-tutorial/
  https://lastminuteengineers.com/a6-gsm-gprs-module-arduino-tutorial/
  https://www.quora.com/What-gases-do-cars-produce
  https://sandboxelectronics.com/?p=165
************************************************************************************/

#include <SoftwareSerial.h> //including the software serial library, which will help the GSM to communicate
#include <TinyGPS++.h>
SoftwareSerial GSMserial(7, 8); // Tx & Rx
SoftwareSerial gpsSerial(2, 3);// Create a software serial port called "gpsSerial"
TinyGPSPlus gps;

#define MQ2pin (0) //connects the gas sensor to the A0
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         READ_SAMPLE_INTERVAL         (50)    //define the time interal(in milisecond) between each samples in the calibration phase
#define         CALIBRATION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
//normal operation
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)


float           COCurve[3]  =  {2.3, 0.72, -0.34};  //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float           SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
float sensorValue;  //variable to store sensor value
float Ro = 10;     //Ro is initialized to 10 kilo ohms

void setup()
{
  Serial.begin(9600); // sets the serial port to 9600
  GSMserial.begin(9600); //sets the serial (GSM)
  gpsSerial.begin(9600);// Start the software serial port at the GPS's default baud
  Serial.println("Gas sensor starting!");
  Serial.print("Calibrating...\n");
  Ro = MQCalibration(MQ2pin);                       //Calibrating the sensor. Please make sure the sensor is in clean air
  //when you perform the calibration
  Serial.print("Calibration is done...\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  delay(10000); // allow the MQ-6 to warm up


}

void loop()
{
  GSMserial.println("AT");
  updateGSMSerial();
  GSMserial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateGSMSerial();
  GSMserial.println("AT+CMGS=\"+250789779262\"");//Put the country code and phone number to sms
  updateGSMSerial();
  Serial.print("CO:");
  GSMserial.print("CO detected: "); //text content
  updateGSMSerial();
  Serial.print(MQGetGasPercentage(MQRead(MQ2pin) / Ro, GAS_CO) );
  GSMserial.print(MQGetGasPercentage(MQRead(MQ2pin) / Ro, GAS_CO));
  updateGSMSerial();
  Serial.print( "PPM" );
  GSMserial.print("PPM");
  updateGSMSerial();
  Serial.print("    ");
  GSMserial.print("    ");
  updateGSMSerial();
  Serial.print("SMOKE:");
  GSMserial.print("SMOKE:");
  updateGSMSerial();
  Serial.print(MQGetGasPercentage(MQRead(MQ2pin) / Ro, GAS_SMOKE) );
  GSMserial.print(MQGetGasPercentage(MQRead(MQ2pin) / Ro, GAS_SMOKE));
  updateGSMSerial();
  Serial.print( "PPM" );
  GSMserial.print("PPM");
  updateGSMSerial();
  Serial.print("\n");
  GSMserial.print("\n");
  updateGSMSerial();
  GSMserial.write(26);

  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("No GPS detected");
    GSMserial.print("No GPS detected");
    while (true);
  }

  Serial.println("");
  delay(8000); // wait 20s for next reading
}

void updateGSMSerial()
{
  delay(500);
  while (Serial.available())
  {
    GSMserial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (GSMserial.available())
  {
    Serial.write(GSMserial.read());//Forward what Software Serial received to Serial Port
  }
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("Latitude: ");
    GSMserial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    GSMserial.print(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    GSMserial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    GSMserial.print(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    GSMserial.print("Altitude: ");
    Serial.println(gps.altitude.meters());
    GSMserial.print(gps.altitude.meters());
  }
  else
  {
    Serial.println("Location: Not Available");
    GSMserial.print("Location: Not Available");
  }

  Serial.print("Date: ");
  GSMserial.print("Date: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    GSMserial.print(gps.date.month());
    Serial.print("/");
    GSMserial.print("/");
    Serial.print(gps.date.day());
    GSMserial.print(gps.date.day());
    Serial.print("/");
    GSMserial.print("/");
    Serial.println(gps.date.year());
    GSMserial.print(gps.date.year());
  }
  else
  {
    Serial.println("Not Available");
    GSMserial.print("Not Available");
  }

  Serial.print("Time: ");
  GSMserial.print("Time: ");
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    GSMserial.print(gps.time.hour());
    Serial.print(":");
    GSMserial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    GSMserial.print(gps.time.minute());
    Serial.print(":");
    GSMserial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    GSMserial.print(gps.time.second());
    Serial.print(".");
    GSMserial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.println(gps.time.centisecond());
    GSMserial.print(gps.time.centisecond());
  }
  else
  {
    Serial.println("Not Available");
    GSMserial.print("Not Available");
  }

  Serial.println();
  Serial.println();
  delay(1000);
}


float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBRATION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBRATION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}

float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}

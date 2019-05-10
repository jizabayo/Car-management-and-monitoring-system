/****************** References ****************************************
  Author: Jean Paul Izabayo, Carnegie Mellon University Africa
  May 7, 2019
************************************************************************************/

/****************** References ****************************************
  https://lastminuteengineers.com/mq2-gas-senser-arduino-tutorial/
  https://lastminuteengineers.com/a6-gsm-gprs-module-arduino-tutorial/
  https://www.quora.com/What-gases-do-cars-produce
  https://sandboxelectronics.com/?p=165
  https://mechatrofice.com/arduino/send-gps-location-via-sms?unapproved=6572&moderation-hash=89f694df8c5c054561afa5fa13e81e59#comment-6572

************************************************************************************/

#include <SoftwareSerial.h>
#include <TinyGPS.h>

int state = 0;
float latitude, longitude;
int year;
byte month, day, hour, minute, second;

TinyGPS gps;
SoftwareSerial GPSSerial(4, 5);
SoftwareSerial GSMSerial(2, 3);

#define MQ2pin (0) //connects the gas sensor to the A0
#define         load_resistance                     (5)     //define the load resistance on the board, in kilo ohms
#define         time_interval         (50)    //define the time interal(in milisecond) between each samples in the calibration phase
#define         calibration_times     (50)    //define how many samples you are going to take in the calibration phase
#define         factor          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet
#define         sample_times           (5)     //define the time interal(in milisecond) between each samples in 
//normal operation
#define         time_interval         (50)    //define how many samples you are going to take in normal operation
#define         calibration_interval  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         GAS_CO                     (1)
#define         GAS_smoke                    (2)


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
  GSMSerial.begin(9600);
  GPSSerial.begin(9600);
  Serial.begin(9600);
  Ro = calibration_mq(MQ2pin);                       //Calibrating the sensor. Please make sure the sensor is in clean air
  //when you perform the calibration
  Serial.print("Calibration is done!\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  Serial.println("Initializing.");
  delay(10000);
}

void loop()
{
  while (GPSSerial.available())
  {
    int c = GPSSerial.read();
    if (gps.encode(c))
    {
      gps.f_get_position(&latitude, &longitude);
    }
  }

  GSMSerial.print("\r");
  delay(1000);
  GSMSerial.print("AT+CMGF=1\r");
  delay(1000);
  /*Here we put mobile phone number*/
  GSMSerial.print("AT+CMGS=\"+250787608032\"\r");
  delay(1000);
  //The text of the message to be sent.
  GSMSerial.print("Latitude :");
  Serial.print("Latitude :");
  GSMSerial.println(latitude, 6);
  Serial.println(latitude, 6);
  GSMSerial.print("Longitude: ");
  Serial.print("Longitude: ");
  GSMSerial.println(longitude, 6);
  Serial.println(longitude, 6);
  GSMSerial.println("  ");
  //date and time
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second);

  // Print data and time
  GSMSerial.print("Date: ");
  Serial.println("Date: ");
  Serial.print(day, DEC);
  GSMSerial.print(day, DEC);
  GSMSerial.print("/");
  Serial.print("/");
  Serial.print(month, DEC);
  GSMSerial.print(month, DEC);
  Serial.print("/");
  GSMSerial.print("/");
  Serial.println(year);
  GSMSerial.println(year);
  GSMSerial.println("  ");

  Serial.print("  Time: ");
  //GSMSerial.println("Current time");
  GSMSerial.print("Time: ");
  Serial.print(hour, DEC);
  GSMSerial.print(hour, DEC);
  Serial.print(":");
  GSMSerial.print(":");
  Serial.print(minute, DEC);
  GSMSerial.print(minute, DEC);
  Serial.print(":");
  GSMSerial.print(":");
  Serial.print(second, DEC);
  GSMSerial.println(second, DEC);
  Serial.print("\n");

  GSMSerial.println("  ");
  Serial.print("CO:");
  GSMSerial.println("CO detected"); //text content
  GSMSerial.print("CO: ");
  Serial.print(gas_percentage(MQRead(MQ2pin) / Ro, GAS_CO) );
  GSMSerial.print(gas_percentage(MQRead(MQ2pin) / Ro, GAS_CO));

  Serial.print( "PPM" );
  GSMSerial.println(" PPM");

  Serial.print("    ");
  //GSMSerial.print("    ");

  Serial.print("SMOKE:");
  GSMSerial.print("SMOKE: ");

  Serial.print(gas_percentage(MQRead(MQ2pin) / Ro, GAS_smoke) );
  GSMSerial.print(gas_percentage(MQRead(MQ2pin) / Ro, GAS_smoke));

  Serial.print( "PPM" );
  GSMSerial.print(" PPM");

  Serial.print("\n");
  GSMSerial.print("\n");

  GSMSerial.write(0x1A);
  delay(10000);


}

float MQ_resistance_cal(int raw_adc)
{
  return ( ((float)load_resistance * (1023 - raw_adc) / raw_adc));
}

float calibration_mq(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < calibration_times; i++) {      //take multiple samples
    val += MQ_resistance_cal(analogRead(mq_pin));
    delay(calibration_interval);
  }
  val = val / calibration_times;                 //calculate the average value

  val = val / factor;                      //divided by factor yields the Ro
  //according to the chart in the datasheet

  return val;
}

float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < sample_times; i++) {
    rs += MQ_resistance_cal(analogRead(mq_pin));
    delay(time_interval);
  }

  rs = rs / sample_times;

  return rs;
}

int  MQ_percentage(float rs_ro_ratio, float *pcurve)
{

  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1])
                     / pcurve[2]) + pcurve[0])));


}

int gas_percentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_CO ) {

    return MQ_percentage(rs_ro_ratio, COCurve);

  }

  else if ( gas_id == GAS_smoke ) {


    return MQ_percentage(rs_ro_ratio, SmokeCurve);
  }
  return 0;
}

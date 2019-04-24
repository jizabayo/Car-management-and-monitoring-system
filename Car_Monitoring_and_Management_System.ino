//source: https://lastminuteengineers.com/mq2-gas-senser-arduino-tutorial/ and https://lastminuteengineers.com/a6-gsm-gprs-module-arduino-tutorial/

// Author: Jean Paul Izabayo, Carnegie Mellon University Africa
//April 21, 2019

#include <SoftwareSerial.h> //including the software serial library, which will help the GSM to communicate
SoftwareSerial GSMserial(3, 2); // Tx & Rx
#define MQ2pin (0) //connects the gas sensor to the A0

float sensorValue;  //variable to store sensor value

void setup()
{
  Serial.begin(9600); // sets the serial port to 9600
  GSMserial.begin(9600); //sets the serial (GSM)
  Serial.println("Gas sensor starting!");
  delay(20000); // allow the MQ-6 to warm up

}

void loop()
{
  sensorValue = analogRead(MQ2pin); // read analog input pin 0

  Serial.print("Sensor Value: ");
  Serial.print(sensorValue);

  if (sensorValue > 20)
  {
    GSMserial.println("AT");
    updateGSMSerial();
    GSMserial.println("AT+CMGF=1"); // Configuring TEXT mode
    updateGSMSerial();
    GSMserial.println("AT+CMGS=\"+250787608032\"");//Put the country code and phone number to sms
    updateGSMSerial();
    GSMserial.print("Smoke detected: "); //text content
    GSMserial.print(sensorValue);
    GSMserial.print(" PPM");
    updateGSMSerial();
    GSMserial.write(26);
  }

  Serial.println("");
  delay(8000); // wait 7s for next reading
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

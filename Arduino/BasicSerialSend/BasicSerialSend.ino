
void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(9600);
 }

void loop()
{
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {

    Serial.write("12.4");
  }
}


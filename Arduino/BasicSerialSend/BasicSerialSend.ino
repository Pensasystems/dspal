
void setup()
{
  // start serial port at 9600 bps:
  Serial.begin(115200);
 }

void loop()
{
   delay(250); 
   Serial.write("1240");
}


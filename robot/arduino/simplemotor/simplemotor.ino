int input=0;
void setup()
{
  //LED ON
  pinMode(13,OUTPUT);
  //initialize serial communication
  Serial.begin(9600);
  //wait for serial communication
  while(!Serial){}
}
void loop()
{
  if(Serial.available())
  {
    input=Serial.parseInt();
  }
    Serial.println(input);
    //go
    if(input==1)
    {
        analogWrite(5,255);
        analogWrite(6,0);
        delay(100);
        Serial.write("stuck in HIGH\n");
    }
    //stop
    else if(input==0)
    {
        analogWrite(5,0);
        analogWrite(6,0);
        delay(100);
        Serial.write("stuck in LOW\n");
    }
}

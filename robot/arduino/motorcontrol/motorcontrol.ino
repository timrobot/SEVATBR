//MOTOR CONTROL
void setup()
{
  //begin serial communication
 Serial.begin(9600);
 while(!Serial){} 
}
//stop the movement of the robot
void halt()
{
  analogWrite(5,0);
  analogWrite(6,0);
  analogWrite(10,0);
  analogWrite(11,0);
}
//move left
void left()
{
  analogWrite(5,255);
  analogWrite(6,0);
  analogWrite(10,255);
  analogWrite(11,0);
 delay(100); 
}
//move right
void right()
{
  analogWrite(5,0);
  analogWrite(6,255);
  analogWrite(10,0);
  analogWrite(11,255);
  delay(100);
}
//move forward
void forward()
{
  analogWrite(5,0);
  analogWrite(6,255);
  analogWrite(10,255);
  analogWrite(11,0);
  delay(100); 
}
//move back
void reverse()
{
  analogWrite(5,255);
  analogWrite(6,0);
  analogWrite(10,0);
  analogWrite(11,255);
  delay(100); 
}
char input;
void loop()
{
  //if input comes, read it
  if(Serial.available())
  {
    input=Serial.read();
  }
  //if input is w, then move forward
  if(input=='w')
  {
    forward();
  }
  //if input is s, thenn move backward
  else if(input=='s')
  {
    reverse();
  }
  //if input is a, then move left
  else if(input=='a')
  {
    left(); 
  }
  //if input is d, then move right
  else if(input=='d')
  {
    right(); 
  }
  //if input is "space", then stop movement
  else if(input==' ')
  {
    halt(); 
  }
}



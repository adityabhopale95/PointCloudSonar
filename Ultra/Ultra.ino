
#define echo 7
#define trig 8

long duration;
int distance;
int val_read;

void setup()
{
  pinMode(echo,INPUT);
  pinMode(A0,INPUT);
  pinMode(trig,OUTPUT);
  Serial.begin(4800);
}

void loop()
{
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  
  duration = pulseIn(echo,HIGH);
//  val_read = analogRead(A0);
//  Serial.println("ADC Value: ");
//  Serial.println(val_read);
  distance = duration * 0.034/2;
//  Serial.print("Distance: ");
  Serial.write(distance);  
  delay(50);
}

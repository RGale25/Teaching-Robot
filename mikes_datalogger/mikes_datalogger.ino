const int irqPin = 3;
const int irqNum = 1;
const int numberOfEntries = 8;

volatile unsigned long microSeconds;
volatile byte index = 0;
volatile unsigned long results[ numberOfEntries ];
volatile boolean start = false;

void setup()
{
  pinMode( irqPin, INPUT );
  Serial.begin( 9600 );
  attachInterrupt( irqNum, dataLogger, CHANGE );
}

void loop()
{
  if( index >= numberOfEntries )
  {
    Serial.println( "Data" );
    for( byte i=0; i< numberOfEntries; i++ )
    {
      Serial.println( results[i] );
    }
    while(1);
  }
  delay( 1000 );
}

void dataLogger()
{
  if( start )
  {
    if( index < numberOfEntries )
    {
      results[index] = micros() - microSeconds;
      index = index + 1;
    }
  }
  else
  {
    if( digitalRead( irqPin ) == 1 )
    {
      start = true;
    }
  }
  microSeconds = micros();  
}

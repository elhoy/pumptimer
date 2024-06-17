/*/storage/DEVT/Workspace-Code/Arduino_tinkering/Blink_rand/Blink_rand.ino
Quick'n'Dirty Garden Water Pump Manager
v2
Jun 2024 EH

Description:
Revert to simpler delay function and attempt to use supply measurement to indicate day/night.
Also reverted to 5V relay and 5V Trinket, due to availablity.

Purpose:
To run a 12V (solar) pump via a relay, 
dependant on water level (float switch on/off)
and also time since last run (to allow for refil at lower tank).

One potential desirable extension would be to also detect when 
the pump is running dry (ie. lower tank empty), possibly by
monitoring 12V voltage across pump via ADC 
(assuming dry pump is less effort and therefore higher voltage).

Timing is via delay functions (simplified!), which will be good enough.

Target:
Arduino Trinket 5V (original).
Power supply via BAT+ can be up to 16V, therefore 12V solar is good.
Pinouts as below, Trinket "#pin" match ATtiny PB#.
ADC is 10bit, Vsupply input to pin #4 (Analogue2) via potential divider.
Intending to run at default 8MHz (which is plenty!).


https://learn.adafruit.com/introducing-trinket/guided-tour
https://www.microchip.com/en-us/product/ATtiny85



Modification History:
06Jun2024 - first draft
13Jun2024 - rehashing using simpler code, delay fn instead of millis and simpler loops.
14Jun2024 - refined ready for deployment! (don't forget if a==b syntax)
16Jun2024 - v2b. changed sequence - supplycheck moved into pump cycle, in case V drops during pumping. Some timing problem still exists - TODO revert to using milis().
16Jun2024 - v2c. added custom "seconds" delay function

Based initially on example analogInput Arduino project.
  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

//------------------------------------------------------//
// INITIALISATION
//------------------------------------------------------//

//Physical Pin Assignments
int supplySensePin = 2;   //Trinket pin #4 (but use A2 for analogRead call), select the input pin for the analog in (12V sense via divider)
int pumpRelayCtrlPin = 2;  //Trinket pin #2 (PB2), switched voltage sink for relay control
int ledPin = 1;            //Trinket pin #1, select the pin for the onboard LED

//initialise global holding variables
float lowVsupply = 0.95; //desired lowest ADC_IN voltage value for comparison (supplyVmax=14.5V but divided by 1:10 with resistor divider is 1.318V; supplyVlow=10.5V div 1:10 is 0.955V.)
float senseVsupply = 0;  //detected voltage, converted from analogue reading
float conversionFactor = (5.0/1023); //magic number to convert analogue pin value to voltage (10bit is 1024; VCC=VREF=VAmax.)
int timeCounter = 0;  //total count holding variable
int timeCheckWait = 30; //check state interval, in seconds (default 30sec)
int timePumpInterval = 3600; //delay interval between pumps, in seconds (default 1hr)
int timePumpGo = 300; //run pump for maximum time, in seconds (default 5mins)

void setup() {
//declare the IO pins types:
//  pinMode(supplySensePin, INPUT);  //don't declare this, as it is analogue in by default - this would set as digital in.
  pinMode(pumpRelayCtrlPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  //set IO pins initial states
  digitalWrite(pumpRelayCtrlPin, LOW);
  digitalWrite(ledPin, LOW);

  //show startup indication
  //LED five blink half sec on/off
  ledFlash(5,500);
}

//------------------------------------------------------//
// MAIN FUNCTION
//------------------------------------------------------//

void loop() {
  
  //DO STUFF (pump water up)
  //RUN IF ENOUGH POWER, BUT WAIT BETWEEN PUMP CYCLES.
  
  //start pump with solid LED
  pumpGo();

  //WAIT FOR PUMPING
  //reset time marker
  timeCounter = 0;
  while (timeCounter < timePumpGo)
  {
    delaySeconds(timeCheckWait);  // wait
    timeCounter = timeCounter + timeCheckWait;
    //check voltage level
       bool checkResult = supplyCheck();
       if (checkResult == false)
       {
         //NOT ENOUGH POWER (nighttime or low battery)
         //LED ten blinks quarter sec
         ledFlash(10,250);
         //stop early
         timeCounter = timePumpGo;
         break;
       }
//    ledFlash(1,100);
  }

  //stop pump and turn LED off
  pumpStop();

  //WAIT FOR REFILL
  //reset time marker
  timeCounter = 0;
  while (timeCounter < timePumpInterval)
  {
    delaySeconds(timeCheckWait);  // wait
    //LED 2 sec blink to show wait activity
    ledFlash(3,500);
    timeCounter = timeCounter + timeCheckWait;
  }
} 



//------------------------------------------------------//
// CUSTOM FUNCTIONS
//------------------------------------------------------//
void delaySeconds(int waitInSeconds)
{
  // https://docs.arduino.cc/built-in-examples/digital/BlinkWithoutDelay/
  // https://www.arduino.cc/en/Tutorial/BuiltInExamples/BlinkWithoutDelay
  const long msToSec = 1000; //milliseconds
  unsigned long startMillis = millis();
  unsigned long currentMillis = startMillis;
  unsigned long delayedCount = 0;

  while (delayedCount < waitInSeconds) 
  {
    //wait
    //check the time
// ?? WHY DOESN'T MILLIS WORK ON TRINKET ??         
//    currentMillis = millis();
//    delayedCount = (currentMillis - startMillis) * msToSec;

    delay(1000);
    delayedCount = delayedCount + 1;

  }
}

bool supplyCheck()
{
  // check supply voltage
  // read the value from the sensor, taking an average
  int loopcount = 0;
  int loops = 3;
  int tpause = 500; //ms
  senseVsupply = 0;
  float sensorValue = 0;

  while (loopcount < loops)
  {
    sensorValue = analogRead(supplySensePin);
    senseVsupply = senseVsupply + (sensorValue * conversionFactor);
    delay(tpause);
    loopcount++;
  }
  senseVsupply= (senseVsupply/loopcount);

  // take action
  if (senseVsupply < lowVsupply)
    {
    //LOW
    return false;
    }
  else
    {
    //HIGH
    return true;
    }
}

void pumpGo()
{
  //pump and LED on
  digitalWrite(pumpRelayCtrlPin, HIGH);
  digitalWrite(ledPin, HIGH);
  return;
}

void pumpStop()
{
  //pump and LED off
  digitalWrite(pumpRelayCtrlPin, LOW);
  digitalWrite(ledPin, LOW);
  return;
}

void ledFlash(int loops, int mspause) 
{
  //default led flash to single 1 second pulse, if no params
  if (loops < 0) {
    loops = 1;
  }
  if (mspause < 0) {
    mspause = 1000;
  }

  //go
  int count = 0;
  do {
    digitalWrite(ledPin, HIGH);
    delay(mspause); //ms
    digitalWrite(ledPin, LOW);
    delay(mspause); //ms
    count++;
  } while (count < loops);

  return;

}

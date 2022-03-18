#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   
const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore.    
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
#include <LiquidCrystal.h>
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define NOTE_DS5 622
#define NOTE_E5  659
#define REST      0
int tempo = 80;
int buzzer = 7;
const int melody[] PROGMEM = {
  NOTE_E5, 16, NOTE_DS5, 16,  
};
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / tempo;
int divider = 0, noteDuration = 0;
void setup() {   
  Serial.begin(115200);          // For Serial Monitor
  pinMode(buzzer,OUTPUT);
  // Configure the PulseSensor object, by assigning our variables to it. 
  pulseSensor.analogInput(PulseWire);   
  pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);   
 // Double-check the "pulseSensor" object was created and "began" seeing a signal. 
   if (pulseSensor.begin()) {
   Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.  
  }
  lcd.begin(16, 2);
  lcd.print("IBI:");
}
void loop() {
 int myIBI = pulseSensor.getInterBeatIntervalMs();  // Calls function on our pulseSensor object that returns BPM as an "int".
if (pulseSensor.sawStartOfBeat()) { 
 Serial.println(myIBI);                        
}
  if( myIBI < 600)
   {
    digitalWrite(buzzer,LOW);
    lcd.setCursor(4, 0);
  // print the number of seconds since reset:
  lcd.print(myIBI);
  lcd.setCursor(7,0);
  lcd.print(" ");
  //delay(20);
  lcd.setCursor(0,1);
       
   lcd.print("Just chill bro!!");
      // considered best practice in a simple sketch.

      for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = pgm_read_word_near(melody+thisNote + 1);
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzer, pgm_read_word_near(melody+thisNote), noteDuration * 0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);

    // stop the waveform generation before the next note.
    noTone(buzzer);
                                               }
   }

   else 
   {
    
    lcd.setCursor(4, 0);
  // print the number of seconds since reset:
  lcd.print(myIBI);
  if( myIBI < 1000)
   { 
    lcd.setCursor(7,0);
    lcd.print(" ");
    }
  lcd.setCursor(0,1);
        for(int n = 0; n < 16; n++) 
        {
                lcd.print(" ");
        }
  
        

  
   }
  delay(2000);  //considered best practice in a simple sketch.
}

  

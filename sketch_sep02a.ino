#include <LiquidCrystal_I2C.h>
#include <MemoryFree.h>
#include <arduinoFFT.h>
#include <string.h>
#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 2300 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.

LiquidCrystal_I2C lcd(0x27, 16, 2);
arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;

int noteFrequencies[84];
const char* noteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
char* note = "     ";
double peak = 0;
String peakPrint = "";
void setup()
{
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("freq: ");
  lcd.setCursor(0, 1);
  lcd.print("note: ");
  Serial.begin(115200); //Baud rate for the Serial Monitor
  Serial.println("LETS START");
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY)); //Period in microseconds
  
  fillnoteFrequencies();
}

void loop()
{

  double vReal[SAMPLES] = {0}; //create vector of size SAMPLES to hold real values
  double vImag[SAMPLES] = {0}; //create vector of size SAMPLES to hold imaginary values
  float ref_volt = float(readVcc()) / 1000.0;
  float dbValue;
  
  double db;
  char db_str[4], ble_dat[12];
  vReal[0] = analogRead(A0);
  dbValue = (vReal[0] / 1024.0) * ref_volt * 50.0;
  dtostrf(dbValue, 1, 2, db_str);

  if (dbValue < 120)
    return;
    
  for (int i = 1; i < SAMPLES; i++)
  {
    microSeconds = micros(); 

    vReal[i] = analogRead(A0); 
    vImag[i] = 0; //Makes imaginary term 0 always
   
    while (micros() < (microSeconds + samplingPeriod))
    {
      
    }
  }


  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);


  peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
//  lcd.setCursor(6, 0);
//  lcd.print("        ");
//  lcd.setCursor(6, 0);
//  lcd.print(peakPrint);
  delay(50);
//  lcd.print("   ");
  getClosestNote(peak);
  lcd.setCursor(6,0);
  lcd.print(peak);
  lcd.print(' ');
  lcd.print(' ');
  lcd.setCursor(6, 1);
  lcd.print(note);
  delay(1000);
}

void fillnoteFrequencies() {
  noteFrequencies[0] = 16;
  noteFrequencies[1] = 17;
  noteFrequencies[2] = 18;
  noteFrequencies[3] = 20;
  noteFrequencies[4] = 21;
  noteFrequencies[5] = 22;
  noteFrequencies[6] = 23;
  noteFrequencies[7] = 25;
  noteFrequencies[8] = 26;
  noteFrequencies[9] = 28;
  noteFrequencies[10] = 29;
  noteFrequencies[11] = 31;
  int i;
  for (i = 12 ; i < 84; ++i) {
    noteFrequencies[i] = noteFrequencies[i - 12] * 2;
  }
}

char* getClosestNote(double peak) {
    int mod = 0;
    for (int j = 0 ; j < 84; ++j) {
      if (peak >= noteFrequencies[j] - 2 and peak <= noteFrequencies[j] + 2){
          mod = j % 12;
          if (strlen(noteNames[mod]) > 1){
            note[0] = ' ';
            note[1] = ' ';
            note[2] = noteNames[mod][0];
            note[3] = noteNames[mod][1];
            note[4] = ' ';
          }else{
            note[0] = ' ';
            note[1] = ' ';
            note[2] = noteNames[mod][0];
            note[3] = ' ';
            note[4] = ' ';
          }
          break;
      }
  
      if (peak < noteFrequencies[j]) {
        mod = j % 12;
        if (peak > ((noteFrequencies[j] + noteFrequencies[j-1])/2)){
          if (strlen(noteNames[mod]) > 1){
            note[0] = ' ';
            note[1] = ' ';
            note[2] = noteNames[mod][0];
            note[3] = noteNames[mod][1];
            note[4] = '>';
          }else{
            note[0] = ' ';
            note[1] = ' ';
            note[2] = noteNames[mod][0];
            note[3] = '>';
            note[4] = ' ';
          }
          break;
        }else{
          if (strlen(noteNames[mod]) > 1){
            note[0] = '<';
            note[1] = ' ';
            note[2] = noteNames[mod][0];
            note[3] = noteNames[mod][1];
            note[4] = ' ';
          }else{
            note[0] = '<';
            note[1] = ' ';
            note[2] = noteNames[mod][0];
            note[3] = ' ';
            note[4] = ' ';
          }
          break;
        }
      }
    }
    return;
}

// read voltage to ensure ADC converts properly
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

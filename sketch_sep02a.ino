/*
  File/Sketch Name: AudioFrequencyDetector

  Version No.: v1.0 Created 12 December, 2019

  Original Author: Clyde A. Lettsome, PhD, PE, MEM

  Description:  This code/sketch makes displays the approximate frequency of the loudest sound detected by a sound detection module. For this project, the analog output from the
  sound module detector sends the analog audio signal detected to A0 of the Arduino Uno. The analog signal is sampled and quantized (digitized). A Fast Fourier Transform (FFT) is
  then performed on the digitized data. The FFT converts the digital data from the approximate discrete-time domain result. The maximum frequency of the approximate discrete-time
  domain result is then determined and displayed via the Arduino IDE Serial Monitor.

  Note: The arduinoFFT.h library needs to be added to the Arduino IDE before compiling and uploading this script/sketch to an Arduino.

  License: This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License (GPL) version 3, or any later
  version of your choice, as published by the Free Software Foundation.

  Notes: Copyright (c) 2019 by C. A. Lettsome Services, LLC
  For more information visit https://clydelettsome.com/blog/2019/12/18/my-weekend-project-audio-frequency-detector-using-an-arduino/

*/

#include "arduinoFFT.h"
#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 2300 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
//#include <LiquidCrystal_I2C.h>
//
//LiquidCrystal_I2C lcd(0x27, 16, 2);
//
//
arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;


int noteFrequencies[84];

void setup()
{
//  lcd.begin(); 
//  lcd.backlight();
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print("TUNER");
//  delay(2000);
//  lcd.clear();
//  lcd.setCursor(0, 0);
//  lcd.print("freq: ");
//  lcd.setCursor(0, 1);
//  lcd.print("note: ");
  Serial.begin(115200); //Baud rate for the Serial Monitor
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds
//  Serial.println("filling...");
  fillnoteFrequencies();
}

void loop()
{
  double vReal[SAMPLES] = {0}; //create vector of size SAMPLES to hold real values
  double vImag[SAMPLES] = {0}; //create vector of size SAMPLES to hold imaginary values
  float ref_volt = float(readVcc()) / 1000.0;
  //preallocate
  float dbValue;
  double peak = 0;
  double db;
  char db_str[4], ble_dat[12];
  vReal[0] = analogRead(A5);
  dbValue = (vReal[0] / 1024.0) * ref_volt * 50.0;
  dtostrf(dbValue, 1, 2, db_str);
 
 if (dbValue < 45)
    return;

  Serial.print("sound volume: ");
 Serial.println(db_str);
  /*Sample SAMPLES times*/
  for (int i = 1; i < SAMPLES; i++)
  {
    microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script.

    vReal[i] = analogRead(A5); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
    vImag[i] = 0; //Makes imaginary term 0 always
    /*remaining wait time between samples if necessary*/
    while (micros() < (microSeconds + samplingPeriod))
    {
      //do nothing
    }
  }

 
//  Serial.println("calculating fequency...");
  /*Perform FFT on samples*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  /*Find peak frequency and print peak*/
  peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  Serial.print("Calculated Frequency: ");
  Serial.println(peak);


//  String note = getNote(peak);
//  String deviation = getDeviation(peak);

  //String toPrint = "";
  String peakPrint = "";
  Serial.print("seeking note...:");
  peakPrint = getClosestNote(peak);
  Serial.println(peakPrint);
//  String peakString = "  " + (String)peak + "   ";
//  String noteToPrint = peakPrint;
//  lcd.setCursor(7,0);
//  lcd.print(peak);
//  delay(1000);
  //lcd.setCursor(7,0);
  //lcd.print(peak);
  //    if(deviation == "   ")
  //      toPrint = "   " + note + "   ";
  //    else if (deviation == "  <" or deviation == " <<" or deviation == "<<<")
  //      toPrint = deviation + note + "   ";
  //    else
  //      toPrint = "   " + note + deviation;
  //    peakPrint = " " + String(peak) + "  ";
  //
  //    lcd.setCursor(7,0);
  //    lcd.print(peakPrint);
  //    lcd.setCursor(6,1);
  //    lcd.print(toPrint);

  // lcd.print(getNoteAndDeviation(peak));
  /*Script stops here. Hardware reset required.*/
  //while (1); //do one time
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
  Serial.println("starting fill loop");
  int i;
  for (i = 12 ; i < 84; ++i) {
    noteFrequencies[i] = noteFrequencies[i - 12] * 2;
  }
  Serial.println("filled");
}

String getNote(double peak) {
  if (peak < 92)
    return "E";
  if (peak >= 92 and peak < 120)
    return "A";
  if (peak >= 120 and peak < 173)
    return "D";
  if (peak > 173 and peak < 220)
    return "G";
  if (peak > 220 and peak < 290)
    return "B";
  if (peak > 290)
    return "E";
}

String getClosestNote(double peak) {
  Serial.print("received frequency: ");
  Serial.println(peak);
  const String noteNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
  int j = 0;
  int mod;
  const char lesser = '<';
  const char bigger = '>';
  String returnString;
  
  for (j = 0 ; j < 84; ++j) {
    if (peak >= noteFrequencies[j] - 2 and peak <= noteFrequencies[j] + 2){
        mod = j % 12;
        return " " + noteNames[mod] + " ";
    }
      
    if (peak < noteFrequencies[j]) {
      mod = j % 12;
      if (peak > ((noteFrequencies[j] + noteFrequencies[j-1])/2)){
        returnString = " " + noteNames[mod] + bigger;
        return returnString;
      }else{
        returnString = lesser + noteNames[mod] + " ";
        return returnString;
      }
    }
  }
  return (char*) "X";
}


String getDeviation(double peak) {
  //E low
  if (peak < 82) {
    if (peak < 50)
      return ">>>";
    else if (peak < 65)
      return ">> ";
    else if (peak < 77)
      return "> ";
    else if (peak == 82)
      return "   ";
  }
  else if (peak >= 82 and peak < 92)
    return "  <";
  //A
  else if (peak >= 92 and peak < 95)
    return ">>>";
  else if (peak >= 95 and peak < 103)
    return ">> ";
  else if (peak >= 103 and peak < 109)
    return ">  ";
  else if (peak >= 109 and peak < 111)
    return "   ";
  else if (peak >= 111 and peak < 115)
    return "  <";
  else if (peak >= 115 and peak < 118)
    return " <<";
  else if (peak >= 118 and peak < 120)
    return "<<<";
  //D
  else if (peak >= 120 and peak < 130)
    return ">>>";
  else if (peak >= 130 and peak < 137)
    return ">> ";
  else if (peak >= 137 and peak < 144)
    return ">  ";
  else if (peak >= 144 and peak < 148)
    return "   ";
  else if (peak >= 148 and peak < 158)
    return "  <";
  else if (peak >= 158 and peak < 165)
    return " <<";
  else if (peak >= 165 and peak < 173)
    return "<<<";
  //G
  else if (peak >= 173 and peak < 180)
    return ">>>";
  else if (peak >= 180 and peak < 188)
    return ">> ";
  else if (peak >= 188 and peak < 194)
    return ">  ";
  else if (peak >= 194 and peak < 198)
    return "   ";
  else if (peak >= 198 and peak < 208)
    return "  <";
  else if (peak >= 208 and peak < 213)
    return " <<";
  else if (peak >= 213 and peak < 220)
    return "<<<";
  //B
  else if (peak >= 220 and peak < 230)
    return ">>>";
  else if (peak >= 230 and peak < 238)
    return ">> ";
  else if (peak >= 238 and peak < 244)
    return ">  ";
  else if (peak >= 244 and peak < 248)
    return "   ";
  else if (peak >= 248 and peak < 268)
    return "  <";
  else if (peak >= 268 and peak < 280)
    return " <<";
  else if (peak >= 280 and peak < 290)
    return "<<<";
  //E high
  else if (peak >= 290 and peak < 310)
    return ">>>";
  else if (peak >= 310 and peak < 320)
    return ">> ";
  else if (peak >= 320 and peak < 327)
    return ">  ";
  else if (peak >= 327 and peak < 331)
    return "   ";
  else if (peak >= 331 and peak < 345)
    return "  <";
  else if (peak >= 345 and peak < 360)
    return " <<";
  else if (peak > 360)
    return "<<<";
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

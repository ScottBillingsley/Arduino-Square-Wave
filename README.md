# Arduino-Square-Wave
Generate a square wave using a pin set to a bit in a counter..

  Using Timer1, calculate the time for notes C7 to C8 multiplied by 2^5...On the 
  MIDI note, assign this value to Timer1 OCR1A count down.. On the interrupt increment
  a counter...Each bit to the left in the counter will be an octave lower, so if the clock
  is on array[0], C7 * 2^5, bit 9 would be C2, bit 8 C3 and so on....

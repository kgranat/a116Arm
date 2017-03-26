         
byte names[] = {'G','A','B','c', 'd', 'e', 'f', 'g', 'a', 'b', 'cFive'};  
int tones[] = {2551,2273, 2041, 1915, 1700, 1519, 1432, 1275, 1136, 1014, 956};
byte melody[] = "3c3e3g3e3c3e3g3e3A3c3e3c3A3c3e3c2G2B2d2B2G2B2d3B2G2G2G2G2G2G2G2G2G3c3e3g3e3c3e3g3e3B3d3f3d3B3d3f3d3A3c3e3C3A3c3e3c5G"; //edited melody to play keyboard cat's song.
// count length: 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0
//                                10                  20                  30

int count = 0;
int count2 = 0;
int count3 = 0;
int MAX_COUNT = 30; // the longer the song, the longer this should be. if short, song will cut off.
int statePin = LOW;

void keyboardCat() {
   for (count = 0; count < MAX_COUNT; count++) {
    

    for (count3 = 0; count3 <= (melody[count*2] - 48) * 30; count3++) {
      for (count2=0;count2<8;count2++) {
        if (names[count2] == melody[count*2 + 1]) {       
          digitalWrite(buzzerPin,HIGH);
          delayMicroseconds(tones[count2]);
          digitalWrite(buzzerPin, LOW);
          delayMicroseconds(tones[count2]);
        }
        if (melody[count*2 + 1] == 'p') {
          // make a pause of a certain size
          digitalWrite(buzzerPin, 0);
          delayMicroseconds(500);
        }
      }
      }
   }

}


#define songLength  18
#define tempo       150

char notes[] = "cdfda ag cdfdg gf ";                  // This is the note you want to play
int  beats[] = {1,1,1,1,1,1,4,4,2,1,1,1,1,1,1,4,4,2}; // How long to play the note in array


void rickRoll(){                    // this function is what actually plays the song
  int i, duration;
  for (i = 0; i < songLength; i++){ // step through the song arrays
    duration = beats[i] * tempo;    // length of note/rest in ms
    if (notes[i] == ' '){
      delay(duration);              // then pause for a moment
    }else{
      tone(buzzerPin, frequency(notes[i]), duration);
      delay(duration);              // wait for tone to finish
    }
    delay(tempo/10);                // brief pause between notes
  }
}
//-------------frequency-----------
int frequency(char note){           // This function is called by rickRoll()
  int r;
  const int numNotes = 8;           // number of notes we're storing
  char names[] = { 'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C' };    // letter notes
  int frequencies[] = {262, 294, 330, 349, 392, 440, 494, 523}; // frequencies
  for (r = 0; r < numNotes; r++){
    if (names[r] == note){
      return(frequencies[r]);       // Yes! Return the frequency
    }
  }
  return(0);
}
//============================= you need these functions =======================================


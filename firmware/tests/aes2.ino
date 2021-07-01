#include <AESLib.h>

uint8_t key[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
//pseudo payload data
char * data = "0.0001,2.005,1.500,0.000,3.5,5,4,1.5";
#define maxWordCount 10
char *Words[maxWordCount];

void setup() {
  Serial.begin(57600);
  pretest();
}

void loop() {
  //pretest();
  //delay(2000);
}
void pretest() {
  Serial.print("Data to be encrypted: ");
  Serial.println(data);
  aes256_enc_single(key, data);
  //Serial.print("encrypted:");
  //Serial.println(data);

  Serial.print("encrypted: ");
  for (int i = 0; i < sizeof(data); i++)
  {
    Serial.print(data[i], HEX);
    Serial.print(" "); //separator
  }
  Serial.println();

  aes256_dec_single(key, data);
  Serial.print("decrypted:");
  Serial.println(data);
  Serial.println("Splitted payload data: ");
  splitPayload(data);
}

byte splitPayload(byte * str){
  byte wordCount = 0;
  char *item = strtok(str,",");//get the first word
  while(item != NULL){//loop through all the comma seperated variables
    Words[wordCount] = item;
    item = strtok (NULL, " ,"); //getting subsequence word
    Serial.println(Words[wordCount]);
    wordCount++;
  }
  return 0;
}


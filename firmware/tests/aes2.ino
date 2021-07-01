#include <AESLib.h>

uint8_t key[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
//pseudo payload
char * data = "hello there";
void setup() {
  Serial.begin(57600);



}

void loop() {
  pretest();
  delay(2000);
}
void pretest() {
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

  
}


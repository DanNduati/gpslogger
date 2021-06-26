#include <AES.h>
#include "./printf.h"

AES aes ;

byte *key = (unsigned char*)"0123456789010123";

byte plain[] = "Hello there";
int plainLength = sizeof(plain)-1;  // don't count the trailing /0 of the string !
int padedLength = plainLength + N_BLOCK - plainLength % N_BLOCK;

//real iv = iv x2 ex: 01234567 = 0123456701234567
unsigned long long int my_iv = 36753562;

void setup ()
{
  Serial.begin (9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  printf_begin();
  delay(500);
  printf("\n===testing mode\n") ;
}

void loop () 
{
  prekey_test () ;
  delay(2000);
}

void prekey (int bits)
{
  aes.iv_inc();
  byte iv [N_BLOCK] ;
  byte plain_p[padedLength];
  byte cipher [padedLength] ;
  byte check [padedLength] ;
  aes.set_IV(my_iv);
  aes.get_IV(iv);
  aes.do_aes_encrypt(plain,plainLength,cipher,key,bits,iv);
  aes.set_IV(my_iv);
  aes.get_IV(iv);
  aes.do_aes_decrypt(cipher,padedLength,check,key,bits,iv);
  printf("\n\nPLAIN :");
  aes.printArray(plain,(bool)true);
  printf("\nCIPHER:");
  aes.printArray(cipher,(bool)false);
  printf("\nCHECK :");
  aes.printArray(check,(bool)true);
  printf("\nIV    :");
  aes.printArray(iv,16);
  printf("\n============================================================\n");
}

void prekey_test ()
{
  prekey (128) ;
}

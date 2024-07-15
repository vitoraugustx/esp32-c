#include <AESLib.h>

AESLib aesLib;

uint8_t key[] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
uint8_t iv[N_BLOCK] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

void aes_init() {
  // Assuming initialization involves IV generation only.
  aesLib.gen_iv(iv);
}

String encrypt(String msg) {
  char data[256];
  msg.toCharArray(data, msg.length() + 1);
  int len = aesLib.get_cipher64_length(msg.length());
  char encrypted[len];
  // Cast data to const byte* when passing to encrypt64
  aesLib.encrypt64((const byte*)data, msg.length(), encrypted, key, 128, iv); // Assuming 128-bit AES
  return String(encrypted);
}

String decrypt(String msg) {
  char data[256];
  msg.toCharArray(data, msg.length() + 1);
  char decrypted[msg.length() + 1];
  // Cast data to const byte* when passing to decrypt64
  aesLib.decrypt64((const byte*)data, msg.length(), decrypted, key, 128, iv); // Assuming 128-bit AES
  return String(decrypted);
}


void setup() {
  Serial.begin(115200);
  aes_init();

  String plaintext = "Hello, Arduino!";
  Serial.println("Plaintext: " + plaintext);

  String encrypted = encrypt(plaintext);
  Serial.println("Encrypted: " + encrypted);

  String decrypted = decrypt(encrypted);
  Serial.println("Decrypted: " + decrypted);
}

void loop() {
  // Nothing here
}

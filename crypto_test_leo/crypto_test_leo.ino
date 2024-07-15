#include <AESLib.h>

AESLib aesLib;

char cleartext[512] = {0};
char ciphertext[1024];

byte aes_key[] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66 }; // 16-byte (128-bit) AES key

// General initialization vector (use your own IVs in production for full security!!!)
byte aes_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

String encrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char encrypted[2 * msgLen] = {0};
  aesLib.encrypt64((const byte*)msg, msgLen, encrypted, aes_key, sizeof(aes_key), iv);
  return String(encrypted);
}

String decrypt_impl(char * msg, byte iv[]) {
  int msgLen = strlen(msg);
  char decrypted[msgLen] = {0}; // Half may be enough
  aesLib.decrypt64(msg, msgLen, (byte*)decrypted, aes_key, sizeof(aes_key), iv);
  return String(decrypted);
}

void setup() {
  Serial.begin(115200);

  // JSON message to be encrypted
  sprintf(cleartext, "{\"physical_id\": \"d0368b4875\", \"heartRate\": 192, \"temperature\": 35.13, \"saturation\": 105, \"sysPressure\": 0, \"diaPressure\": 0, \"glucose\": 50, \"timestamp\": \"2021-06-01T12:00:00\", \"patient\": \"Juca\"}");
  Serial.print("Original string: ");
  Serial.println(cleartext);

  aesLib.set_paddingmode(paddingMode::CMS);

  // Send AES key once
  Serial.print("AES Key: ");
  for (int i = 0; i < sizeof(aes_key); i++) {
    Serial.print(aes_key[i], HEX);
    if (i < sizeof(aes_key) - 1) {
      Serial.print(":");
    }
  }
  Serial.println();

  // Encrypt data
  byte enc_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block is written, provide your own new copy...
  String encrypted = encrypt_impl(cleartext, enc_iv);
  sprintf(ciphertext, "%s", encrypted.c_str());
  Serial.print("Base64 encoded ciphertext: ");
  Serial.println(encrypted);

  // Decrypt data
  byte dec_iv[N_BLOCK] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // iv_block is written, provide your own new copy...
  String decrypted = decrypt_impl(ciphertext, dec_iv);
  Serial.print("Original / Base64 decoded text: ");
  Serial.println(decrypted);
}

void loop() {
  // Do nothing
}

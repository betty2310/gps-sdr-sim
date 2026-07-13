#include <assert.h>
#include <string.h>

#include "tools/sha256.h"

int main(void) {
  char hex[SHA256_HEX_SIZE];

  sha256_bytes_hex("", 0, hex);
  assert(strcmp(hex,
                "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855") ==
         0);
  sha256_bytes_hex("abc", 3, hex);
  assert(strcmp(hex,
                "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad") ==
         0);
  assert(sha256_file_hex("tests/fixtures/matched-code-trajectory.csv", hex) ==
         0);
  assert(strcmp(hex,
                "3406275aa94033eba1dc23b4471b8d0bc060860d280b82eae4ef0fb346834647") ==
         0);
  assert(sha256_file_hex("tests/fixtures/does-not-exist", hex) != 0);
  return 0;
}

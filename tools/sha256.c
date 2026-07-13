#include "sha256.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  uint8_t block[64];
  uint32_t state[8];
  uint64_t bit_count;
  size_t block_size;
} sha256_context_t;

static const uint32_t round_constants[64] = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU,
    0x59f111f1U, 0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U,
    0x243185beU, 0x550c7dc3U, 0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U,
    0xc19bf174U, 0xe49b69c1U, 0xefbe4786U, 0x0fc19dc6U, 0x240ca1ccU,
    0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU, 0x983e5152U,
    0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU,
    0x53380d13U, 0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U,
    0xa2bfe8a1U, 0xa81a664bU, 0xc24b8b70U, 0xc76c51a3U, 0xd192e819U,
    0xd6990624U, 0xf40e3585U, 0x106aa070U, 0x19a4c116U, 0x1e376c08U,
    0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU, 0x5b9cca4fU,
    0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

static uint32_t rotate_right(uint32_t value, unsigned int count) {
  return (value >> count) | (value << (32U - count));
}

static void transform(sha256_context_t *context, const uint8_t block[64]) {
  uint32_t words[64];
  uint32_t a, b, c, d, e, f, g, h;
  size_t i;

  for (i = 0; i < 16; ++i) {
    words[i] = ((uint32_t)block[i * 4] << 24) |
               ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] << 8) |
               (uint32_t)block[i * 4 + 3];
  }
  for (i = 16; i < 64; ++i) {
    uint32_t s0 = rotate_right(words[i - 15], 7) ^
                  rotate_right(words[i - 15], 18) ^ (words[i - 15] >> 3);
    uint32_t s1 = rotate_right(words[i - 2], 17) ^
                  rotate_right(words[i - 2], 19) ^ (words[i - 2] >> 10);
    words[i] = words[i - 16] + s0 + words[i - 7] + s1;
  }

  a = context->state[0];
  b = context->state[1];
  c = context->state[2];
  d = context->state[3];
  e = context->state[4];
  f = context->state[5];
  g = context->state[6];
  h = context->state[7];
  for (i = 0; i < 64; ++i) {
    uint32_t sum1 = rotate_right(e, 6) ^ rotate_right(e, 11) ^
                    rotate_right(e, 25);
    uint32_t choose = (e & f) ^ ((~e) & g);
    uint32_t temporary1 = h + sum1 + choose + round_constants[i] + words[i];
    uint32_t sum0 = rotate_right(a, 2) ^ rotate_right(a, 13) ^
                    rotate_right(a, 22);
    uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
    uint32_t temporary2 = sum0 + majority;

    h = g;
    g = f;
    f = e;
    e = d + temporary1;
    d = c;
    c = b;
    b = a;
    a = temporary1 + temporary2;
  }
  context->state[0] += a;
  context->state[1] += b;
  context->state[2] += c;
  context->state[3] += d;
  context->state[4] += e;
  context->state[5] += f;
  context->state[6] += g;
  context->state[7] += h;
}

static void initialize(sha256_context_t *context) {
  memset(context, 0, sizeof(*context));
  context->state[0] = 0x6a09e667U;
  context->state[1] = 0xbb67ae85U;
  context->state[2] = 0x3c6ef372U;
  context->state[3] = 0xa54ff53aU;
  context->state[4] = 0x510e527fU;
  context->state[5] = 0x9b05688cU;
  context->state[6] = 0x1f83d9abU;
  context->state[7] = 0x5be0cd19U;
}

static void update(sha256_context_t *context, const uint8_t *data,
                   size_t size) {
  size_t i;

  for (i = 0; i < size; ++i) {
    context->block[context->block_size++] = data[i];
    if (context->block_size == 64) {
      transform(context, context->block);
      context->bit_count += 512;
      context->block_size = 0;
    }
  }
}

static void finish(sha256_context_t *context, uint8_t digest[32]) {
  size_t i = context->block_size;
  uint64_t total_bits;

  context->block[i++] = 0x80;
  if (i > 56) {
    while (i < 64)
      context->block[i++] = 0;
    transform(context, context->block);
    i = 0;
  }
  while (i < 56)
    context->block[i++] = 0;
  total_bits = context->bit_count + (uint64_t)context->block_size * 8;
  for (i = 0; i < 8; ++i)
    context->block[63 - i] = (uint8_t)(total_bits >> (i * 8));
  transform(context, context->block);

  for (i = 0; i < 32; ++i)
    digest[i] =
        (uint8_t)(context->state[i / 4] >> (24 - (i % 4) * 8));
}

static void digest_to_hex(const uint8_t digest[32],
                          char output[SHA256_HEX_SIZE]) {
  static const char digits[] = "0123456789abcdef";
  size_t i;

  for (i = 0; i < 32; ++i) {
    output[i * 2] = digits[digest[i] >> 4];
    output[i * 2 + 1] = digits[digest[i] & 0x0f];
  }
  output[64] = '\0';
}

void sha256_bytes_hex(const void *data, size_t size,
                      char output[SHA256_HEX_SIZE]) {
  sha256_context_t context;
  uint8_t digest[32];

  initialize(&context);
  if (size > 0)
    update(&context, (const uint8_t *)data, size);
  finish(&context, digest);
  digest_to_hex(digest, output);
}

int sha256_file_hex(const char *path, char output[SHA256_HEX_SIZE]) {
  sha256_context_t context;
  uint8_t buffer[16384];
  uint8_t digest[32];
  FILE *file;
  size_t count;
  int read_failed;
  int close_failed;

  if (path == NULL || output == NULL)
    return -1;
  file = fopen(path, "rb");
  if (file == NULL)
    return -1;
  initialize(&context);
  while ((count = fread(buffer, 1, sizeof(buffer), file)) > 0)
    update(&context, buffer, count);
  read_failed = ferror(file);
  close_failed = fclose(file) != 0;
  if (read_failed || close_failed)
    return -1;
  finish(&context, digest);
  digest_to_hex(digest, output);
  return 0;
}

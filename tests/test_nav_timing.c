#include "gpssim.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

// Independent LNAV extraction: undo the previous word's D30* data inversion.
static unsigned long data_word(const channel_t *channel, int word) {
  unsigned long data = channel->dwrd[word] & 0x3FFFFFC0UL;
  if (word > 0 && (channel->dwrd[word - 1] & 1UL))
    data ^= 0x3FFFFFC0UL;
  return data;
}

static unsigned long how(const channel_t *channel, int subframe) {
  return (data_word(channel, subframe * 10 + 1) >> 13) & 0x1FFFFUL;
}

int main(void) {
  channel_t channel;
  memset(&channel, 0, sizeof(channel));
  for (int i = 0; i < 5; ++i)
    channel.sbf[i][0] = 0x8B0000UL << 6;

  // A fractional epoch just before a frame must use that frame, never the next.
  assert(generateNavMsg((gpstime_t){2400, 29.98}, &channel, 1));
  assert(channel.g0.week == 2400 && channel.g0.sec == 0);
  for (int i = 0; i <= 5; ++i)
    assert(how(&channel, i) == (unsigned long)i);

  assert(generateNavMsg((gpstime_t){2400, 604799.98}, &channel, 1));
  assert(channel.g0.week == 2400 && channel.g0.sec == 604770);
  for (int i = 0; i <= 5; ++i)
    assert(how(&channel, i) == (100795UL + i) % 100800UL);
  assert(((data_word(&channel, 12) >> 20) & 1023UL) == (2400 % 1024));
  unsigned long last_subframe[10];
  memcpy(last_subframe, channel.dwrd + 50, sizeof(last_subframe));

  // Preserve the transmitted preceding subframe across week rollover.
  assert(generateNavMsg((gpstime_t){2401, 0.08}, &channel, 0));
  assert(channel.g0.week == 2401 && channel.g0.sec == 0);
  assert(memcmp(last_subframe, channel.dwrd, sizeof(last_subframe)) == 0);
  for (int i = 0; i <= 5; ++i)
    assert(how(&channel, i) == (unsigned long)i);
  assert(((data_word(&channel, 12) >> 20) & 1023UL) == (2401 % 1024));

  assert(generateNavMsg((gpstime_t){2400, 604800.08}, &channel, 1));
  assert(channel.g0.week == 2401 && channel.g0.sec == 0);
  assert(how(&channel, 1) == 1);
  puts("LNAV fractional frame, HOW modulo-week and WN rollover tests passed");
  return 0;
}

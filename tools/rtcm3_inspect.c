/*
 * rtcm3_inspect — standalone diagnostic client for an RTCM v3 TCP stream.
 *
 * Connects to a host:port (defaults tuned for a Trimble receiver on 5018),
 * scans the byte stream for RTCM v3 frames, validates CRC-24Q, and prints
 * a summary of every frame. When a GPS broadcast ephemeris message (1019)
 * is seen, all fields are decoded and pretty-printed.
 *
 * This is a pre-integration sanity check for the Trimble RTCM ephemeris
 * plan (docs/trimble-rtcm-ephemeris-sync.md). It is intentionally kept in
 * a single translation unit with no project dependencies so it builds and
 * runs anywhere.
 *
 * Usage:
 *   rtcm3_inspect [-h <host>] [-p <port>] [-t <msg_type>] [-v]
 *                 [-s <summary_sec>] [-n <max_frames>]
 *                 [-m <mount_point>] [-u <user>[:<pass>]]
 *
 *   -h  host                   (default 127.0.0.1)
 *   -p  port                   (default 5018, or 2101 for NTRIP casters)
 *   -t  only print this msg#   (0 = all, default 0)
 *   -v  verbose periodic stats
 *   -s  summary interval sec   (default 10)
 *   -n  exit after N frames    (default 0 = forever)
 *   -m  NTRIP mount point      (enables NTRIP v1 client mode)
 *   -u  NTRIP credentials      user or user:pass (default no auth)
 *
 * Direct TCP mode (e.g. Trimble port 5018):
 *   rtcm3_inspect -h 192.168.5.245 -p 5018
 *
 * NTRIP client mode (NTRIP caster with ephemeris):
 *   rtcm3_inspect -h gcloud.navistar.io -p 2101 -m b1_hust_rtcm -u navis -t 1019
 */

#include <arpa/inet.h>
#include <errno.h>
#include <math.h>
#include <netdb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define RTCM_PREAMBLE 0xD3u
#define RTCM_MAX_PAYLOAD 1023
#define RTCM_FRAME_OVERHEAD 6 /* 3 header + 3 CRC */
#define SCANNER_BUF_SIZE (RTCM_MAX_PAYLOAD + RTCM_FRAME_OVERHEAD + 4096)
#define MSG_COUNT_TABLE 4096

/* =========================================================================
 * CRC-24Q: polynomial 0x1864CFB, init 0. RTCM v3 transport.
 * Bit-at-a-time implementation — RTCM frame rates are tiny.
 * ========================================================================= */
static uint32_t crc24q(const uint8_t *buf, size_t len) {
  uint32_t crc = 0;
  size_t i;
  int j;

  for (i = 0; i < len; i++) {
    crc ^= (uint32_t)buf[i] << 16;
    for (j = 0; j < 8; j++) {
      crc <<= 1;
      if (crc & 0x1000000u)
        crc ^= 0x1864CFBu;
    }
  }
  return (crc & 0xFFFFFFu);
}

/* =========================================================================
 * MSB-first bit-field extraction on a byte buffer.
 * ========================================================================= */
static uint64_t bits_u(const uint8_t *buf, int pos, int len) {
  uint64_t value = 0;
  int i;

  for (i = 0; i < len; i++) {
    int bit_idx = pos + i;
    int byte = bit_idx >> 3;
    int bit = 7 - (bit_idx & 7);

    value = (value << 1) | (uint64_t)((buf[byte] >> bit) & 1u);
  }
  return (value);
}

static int64_t bits_s(const uint8_t *buf, int pos, int len) {
  uint64_t u = bits_u(buf, pos, len);

  if (len <= 0 || len >= 64)
    return ((int64_t)u);

  if (u & (1ull << (len - 1)))
    u |= (~0ull) << len;

  return ((int64_t)u);
}

/* =========================================================================
 * RTCM 1019 (GPS broadcast ephemeris).
 *
 * Field encoding follows RTCM Standard 10403.3. Angular quantities are
 * carried in semi-circles; scale factors expressed as 2^-k. Total payload
 * is 488 bits (61 bytes).
 * ========================================================================= */
typedef struct {
  uint32_t msg_num;
  uint32_t prn;
  uint32_t week10;   /* 10-bit week (mod 1024) */
  uint32_t ura;
  uint32_t code_on_l2;
  double   idot_sc;  /* semi-circles/s */
  uint32_t iode;
  double   toc_s;    /* seconds into week */
  double   af2;      /* s/s^2 */
  double   af1;      /* s/s */
  double   af0;      /* s */
  uint32_t iodc;
  double   crs;      /* m */
  double   deltan_sc;/* semi-circles/s */
  double   m0_sc;    /* semi-circles */
  double   cuc;      /* rad */
  double   ecc;
  double   cus;      /* rad */
  double   sqrta;    /* sqrt(m) */
  double   toe_s;    /* seconds into week */
  double   cic;      /* rad */
  double   omg0_sc;  /* semi-circles */
  double   cis;      /* rad */
  double   inc0_sc;  /* semi-circles */
  double   crc_m;    /* m */
  double   aop_sc;   /* semi-circles */
  double   omgdot_sc;/* semi-circles/s */
  double   tgd;      /* s */
  uint32_t sv_health;
  uint32_t l2p_flag;
  uint32_t fit_flag;
} rtcm1019_t;

static int decode_1019(const uint8_t *p, size_t len, rtcm1019_t *e) {
  int pos = 0;

  if (len < 61)
    return (-1);

  e->msg_num     = (uint32_t)bits_u(p, pos, 12); pos += 12;
  e->prn         = (uint32_t)bits_u(p, pos,  6); pos +=  6;
  e->week10      = (uint32_t)bits_u(p, pos, 10); pos += 10;
  e->ura         = (uint32_t)bits_u(p, pos,  4); pos +=  4;
  e->code_on_l2  = (uint32_t)bits_u(p, pos,  2); pos +=  2;
  e->idot_sc     = (double)bits_s(p, pos, 14) * pow(2.0, -43); pos += 14;
  e->iode        = (uint32_t)bits_u(p, pos,  8); pos +=  8;
  e->toc_s       = (double)bits_u(p, pos, 16) * 16.0;           pos += 16;
  e->af2         = (double)bits_s(p, pos,  8) * pow(2.0, -55); pos +=  8;
  e->af1         = (double)bits_s(p, pos, 16) * pow(2.0, -43); pos += 16;
  e->af0         = (double)bits_s(p, pos, 22) * pow(2.0, -31); pos += 22;
  e->iodc        = (uint32_t)bits_u(p, pos, 10); pos += 10;
  e->crs         = (double)bits_s(p, pos, 16) * pow(2.0,  -5); pos += 16;
  e->deltan_sc   = (double)bits_s(p, pos, 16) * pow(2.0, -43); pos += 16;
  e->m0_sc       = (double)bits_s(p, pos, 32) * pow(2.0, -31); pos += 32;
  e->cuc         = (double)bits_s(p, pos, 16) * pow(2.0, -29); pos += 16;
  e->ecc         = (double)bits_u(p, pos, 32) * pow(2.0, -33); pos += 32;
  e->cus         = (double)bits_s(p, pos, 16) * pow(2.0, -29); pos += 16;
  e->sqrta       = (double)bits_u(p, pos, 32) * pow(2.0, -19); pos += 32;
  e->toe_s       = (double)bits_u(p, pos, 16) * 16.0;           pos += 16;
  e->cic         = (double)bits_s(p, pos, 16) * pow(2.0, -29); pos += 16;
  e->omg0_sc     = (double)bits_s(p, pos, 32) * pow(2.0, -31); pos += 32;
  e->cis         = (double)bits_s(p, pos, 16) * pow(2.0, -29); pos += 16;
  e->inc0_sc     = (double)bits_s(p, pos, 32) * pow(2.0, -31); pos += 32;
  e->crc_m       = (double)bits_s(p, pos, 16) * pow(2.0,  -5); pos += 16;
  e->aop_sc      = (double)bits_s(p, pos, 32) * pow(2.0, -31); pos += 32;
  e->omgdot_sc   = (double)bits_s(p, pos, 24) * pow(2.0, -43); pos += 24;
  e->tgd         = (double)bits_s(p, pos,  8) * pow(2.0, -31); pos +=  8;
  e->sv_health   = (uint32_t)bits_u(p, pos,  6); pos +=  6;
  e->l2p_flag    = (uint32_t)bits_u(p, pos,  1); pos +=  1;
  e->fit_flag    = (uint32_t)bits_u(p, pos,  1); pos +=  1;

  (void)pos;
  return (0);
}

static const char *msg_name(uint32_t msg) {
  switch (msg) {
  case 1001: return "GPS L1 observables (legacy)";
  case 1002: return "GPS L1 extended (legacy)";
  case 1003: return "GPS L1/L2 observables (legacy)";
  case 1004: return "GPS L1/L2 extended (legacy)";
  case 1005: return "Stationary ARP";
  case 1006: return "Stationary ARP + Height";
  case 1007: return "Antenna Descriptor";
  case 1008: return "Antenna Descriptor + Serial";
  case 1012: return "GLONASS L1/L2 extended (legacy)";
  case 1013: return "System Parameters";
  case 1019: return "GPS Ephemeris";
  case 1020: return "GLONASS Ephemeris";
  case 1033: return "Receiver/Antenna Descriptor";
  case 1042: return "BeiDou Ephemeris";
  case 1044: return "QZSS Ephemeris";
  case 1045: return "Galileo F/NAV Ephemeris";
  case 1046: return "Galileo I/NAV Ephemeris";
  case 1074: return "GPS MSM4";
  case 1075: return "GPS MSM5";
  case 1076: return "GPS MSM6";
  case 1077: return "GPS MSM7";
  case 1084: return "GLONASS MSM4";
  case 1087: return "GLONASS MSM7";
  case 1094: return "Galileo MSM4";
  case 1097: return "Galileo MSM7";
  case 1114: return "QZSS MSM4";
  case 1117: return "QZSS MSM7";
  case 1124: return "BeiDou MSM4";
  case 1127: return "BeiDou MSM7";
  case 1230: return "GLONASS Code-Phase Biases";
  default:   return "(other)";
  }
}

static void print_1019(const rtcm1019_t *e) {
  /* Effective week: RTCM 1019 carries a 10-bit (mod 1024) week number.
     The current rollover (since 2019-04-07) puts reality in weeks 2304+. */
  uint32_t week_hint = (uint32_t)((uint64_t)(time(NULL) - 315964800ll +
                                             18ll /* leap seconds */) / 604800ull);
  uint32_t week_full = e->week10;

  while (week_full + 1024 <= week_hint)
    week_full += 1024;

  printf("  PRN=%u week=%u(full=%u) URA=%u IODE=%u IODC=%u health=0x%02x code_L2=%u L2P=%u fit=%u\n",
         e->prn, e->week10, week_full, e->ura, e->iode, e->iodc,
         e->sv_health, e->code_on_l2, e->l2p_flag, e->fit_flag);
  printf("  toc=%.0fs  toe=%.0fs  af0=%.9es  af1=%.6es/s  af2=%.3es/s^2  TGD=%.3es\n",
         e->toc_s, e->toe_s, e->af0, e->af1, e->af2, e->tgd);
  printf("  sqrtA=%.9f (A=%.2f m)  ecc=%.10f  i0=%.12f*pi  Omega0=%.12f*pi\n",
         e->sqrta, e->sqrta * e->sqrta, e->ecc, e->inc0_sc, e->omg0_sc);
  printf("  M0=%.12f*pi  omega=%.12f*pi  dN=%.3e sc/s  idot=%.3e sc/s  OmegaDot=%.3e sc/s\n",
         e->m0_sc, e->aop_sc, e->deltan_sc, e->idot_sc, e->omgdot_sc);
  printf("  Cuc=%.3e  Cus=%.3e  Crc=%.3f m  Crs=%.3f m  Cic=%.3e  Cis=%.3e\n",
         e->cuc, e->cus, e->crc_m, e->crs, e->cic, e->cis);
}

/* =========================================================================
 * RTCM v3 frame scanner over a byte stream.
 *
 * On each call, scanner_extract either
 *   returns 1 and exposes a pointer to the next valid frame's payload
 *     (caller must invoke scanner_advance once processing is done),
 *   returns 0 when more bytes are required,
 *   returns -1 after consuming one byte on a framing or CRC failure.
 * ========================================================================= */
typedef struct {
  uint8_t buf[SCANNER_BUF_SIZE];
  size_t  len;
  uint64_t crc_errs;
  uint64_t bytes_dropped;
} rtcm_scanner_t;

static void scanner_drop_one(rtcm_scanner_t *s) {
  if (s->len == 0)
    return;

  memmove(s->buf, s->buf + 1, s->len - 1);
  s->len--;
  s->bytes_dropped++;
}

static int scanner_extract(rtcm_scanner_t *s, uint8_t **out_payload,
                           size_t *out_len, uint32_t *out_msg) {
  uint32_t payload_len, frame_len;
  uint32_t crc_calc, crc_recv;

  while (s->len >= 1 && s->buf[0] != RTCM_PREAMBLE)
    scanner_drop_one(s);

  if (s->len < 3)
    return (0);

  payload_len = ((uint32_t)(s->buf[1] & 0x03u) << 8) | (uint32_t)s->buf[2];
  if (payload_len > RTCM_MAX_PAYLOAD) {
    scanner_drop_one(s);
    return (-1);
  }

  frame_len = 3u + payload_len + 3u;
  if (s->len < frame_len)
    return (0);

  crc_calc = crc24q(s->buf, 3u + payload_len);
  crc_recv = ((uint32_t)s->buf[3 + payload_len    ] << 16) |
             ((uint32_t)s->buf[3 + payload_len + 1] <<  8) |
             ((uint32_t)s->buf[3 + payload_len + 2]);

  if (crc_calc != crc_recv) {
    s->crc_errs++;
    scanner_drop_one(s);
    return (-1);
  }

  *out_payload = s->buf + 3;
  *out_len     = (size_t)payload_len;
  if (payload_len >= 2)
    *out_msg = (uint32_t)bits_u(s->buf + 3, 0, 12);
  else
    *out_msg = 0;

  return (1);
}

static void scanner_advance(rtcm_scanner_t *s, size_t payload_len) {
  size_t frame_len = 3 + payload_len + 3;

  if (frame_len >= s->len) {
    s->len = 0;
    return;
  }
  memmove(s->buf, s->buf + frame_len, s->len - frame_len);
  s->len -= frame_len;
}

/* =========================================================================
 * TCP client.
 * ========================================================================= */
static int tcp_connect(const char *host, int port) {
  struct addrinfo hints;
  struct addrinfo *res = NULL;
  struct addrinfo *rp;
  char port_str[16];
  int fd = -1;
  int rc;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family   = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  snprintf(port_str, sizeof(port_str), "%d", port);

  rc = getaddrinfo(host, port_str, &hints, &res);
  if (rc != 0) {
    fprintf(stderr, "getaddrinfo(%s:%d): %s\n", host, port, gai_strerror(rc));
    return (-1);
  }

  for (rp = res; rp != NULL; rp = rp->ai_next) {
    fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (fd < 0)
      continue;
    if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
      break;
    close(fd);
    fd = -1;
  }
  freeaddrinfo(res);
  return (fd);
}

/* =========================================================================
 * NTRIP v1 client handshake.
 *
 * Sends an HTTP/1.0 GET to the caster, reads response headers until the
 * blank line, checks for a 200 status (either "ICY 200 OK" or "HTTP/1.x
 * 200"), then returns. The caller's recv() loop then reads the RTCM stream
 * that follows immediately after the headers.
 * ========================================================================= */
static void base64_encode(const char *in, char *out, size_t out_size) {
  static const char T[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t in_len = strlen(in);
  size_t o = 0;
  size_t i;

  for (i = 0; i < in_len && o + 4 < out_size; i += 3) {
    uint8_t a = (uint8_t)in[i];
    uint8_t b = (i + 1 < in_len) ? (uint8_t)in[i + 1] : 0;
    uint8_t c = (i + 2 < in_len) ? (uint8_t)in[i + 2] : 0;

    out[o++] = T[a >> 2];
    out[o++] = T[((a & 3) << 4) | (b >> 4)];
    out[o++] = (i + 1 < in_len) ? T[((b & 0xf) << 2) | (c >> 6)] : '=';
    out[o++] = (i + 2 < in_len) ? T[c & 0x3f] : '=';
  }
  out[o] = '\0';
}

/* Returns 0 on success, -1 on failure (also prints reason to stderr). */
static int ntrip_handshake(int fd, const char *mount,
                           const char *user, const char *pass) {
  char req[1024];
  char hdr[4096];
  int req_len, h;
  ssize_t n;

  if (user && user[0]) {
    char cred[256];
    char b64[512];

    snprintf(cred, sizeof(cred), "%s:%s", user, pass ? pass : "");
    base64_encode(cred, b64, sizeof(b64));
    req_len = snprintf(req, sizeof(req),
                       "GET /%s HTTP/1.0\r\n"
                       "User-Agent: NTRIP rtcm3_inspect/1.0\r\n"
                       "Authorization: Basic %s\r\n"
                       "\r\n",
                       mount, b64);
  } else {
    req_len = snprintf(req, sizeof(req),
                       "GET /%s HTTP/1.0\r\n"
                       "User-Agent: NTRIP rtcm3_inspect/1.0\r\n"
                       "\r\n",
                       mount);
  }

  if (send(fd, req, (size_t)req_len, 0) != (ssize_t)req_len) {
    perror("send NTRIP request");
    return (-1);
  }

  /* Read response byte-by-byte until \r\n\r\n. */
  h = 0;
  memset(hdr, 0, sizeof(hdr));
  while (h < (int)sizeof(hdr) - 1) {
    n = recv(fd, hdr + h, 1, 0);
    if (n <= 0) {
      fprintf(stderr, "NTRIP: connection closed during handshake\n");
      return (-1);
    }
    h++;
    if (h >= 4 && hdr[h-4] == '\r' && hdr[h-3] == '\n' &&
        hdr[h-2] == '\r' && hdr[h-1] == '\n')
      break;
  }

  fprintf(stderr, "NTRIP response:\n%.*s\n", h, hdr);

  if (strstr(hdr, "200") == NULL) {
    fprintf(stderr, "NTRIP: no 200 status in response — check mount point and credentials\n");
    return (-1);
  }
  return (0);
}

/* =========================================================================
 * main loop: read, scan, print.
 * ========================================================================= */
static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig) {
  (void)sig;
  g_stop = 1;
}

static void print_stats(FILE *out, uint64_t total_frames,
                        const uint32_t *msg_count,
                        uint64_t crc_errs, uint64_t bytes_dropped) {
  int i;

  fprintf(out, "--- %llu frames; crc_err=%llu drop=%llu; type counts:",
          (unsigned long long)total_frames,
          (unsigned long long)crc_errs,
          (unsigned long long)bytes_dropped);
  for (i = 0; i < MSG_COUNT_TABLE; i++) {
    if (msg_count[i] > 0)
      fprintf(out, " %d:%u", i, msg_count[i]);
  }
  fprintf(out, "\n");
}

static void usage(const char *prog) {
  fprintf(stderr,
          "Usage: %s [-h host] [-p port] [-t msg_type]"
          " [-v] [-s summary_sec] [-n max_frames]"
          " [-m mount_point] [-u user[:pass]]\n"
          "  Direct TCP:  %s -h 192.168.5.245 -p 5018\n"
          "  NTRIP:       %s -h gcloud.navistar.io -p 2101 -m b1_hust_rtcm -u navis\n",
          prog, prog, prog);
}

int main(int argc, char **argv) {
  const char *host = "127.0.0.1";
  int port = 5018;
  int verbose = 0;
  int filter_msg = 0;
  int summary_sec = 10;
  long max_frames = 0;
  const char *ntrip_mount = NULL;
  const char *ntrip_user  = NULL;
  const char *ntrip_pass  = NULL;
  int i;

  rtcm_scanner_t scanner;
  uint32_t msg_count[MSG_COUNT_TABLE];
  uint64_t total_frames = 0;
  time_t last_summary;
  int fd;

  memset(&scanner, 0, sizeof(scanner));
  memset(msg_count, 0, sizeof(msg_count));

  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-h") == 0 && i + 1 < argc) {
      host = argv[++i];
    } else if (strcmp(argv[i], "-p") == 0 && i + 1 < argc) {
      port = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-t") == 0 && i + 1 < argc) {
      filter_msg = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-v") == 0) {
      verbose = 1;
    } else if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
      summary_sec = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
      max_frames = atol(argv[++i]);
    } else if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
      ntrip_mount = argv[++i];
    } else if (strcmp(argv[i], "-u") == 0 && i + 1 < argc) {
      /* Accept "user" or "user:pass" */
      char *colon;

      ntrip_user = argv[++i];
      colon = strchr(argv[i], ':');
      if (colon) {
        *colon = '\0';
        ntrip_pass = colon + 1;
      }
    } else if (strcmp(argv[i], "--help") == 0) {
      usage(argv[0]);
      return (0);
    } else {
      fprintf(stderr, "unknown argument: %s\n", argv[i]);
      usage(argv[0]);
      return (2);
    }
  }

  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  fd = tcp_connect(host, port);
  if (fd < 0) {
    fprintf(stderr, "failed to connect to %s:%d\n", host, port);
    return (1);
  }
  fprintf(stderr, "connected to %s:%d\n", host, port);

  if (ntrip_mount) {
    fprintf(stderr, "NTRIP mount: %s  user: %s\n",
            ntrip_mount, ntrip_user ? ntrip_user : "(none)");
    if (ntrip_handshake(fd, ntrip_mount, ntrip_user, ntrip_pass) != 0) {
      close(fd);
      return (1);
    }
    fprintf(stderr, "NTRIP stream open; waiting for RTCM v3 frames...\n");
  } else {
    fprintf(stderr, "waiting for RTCM v3 frames...\n");
  }

  last_summary = time(NULL);

  while (!g_stop) {
    ssize_t n;

    n = recv(fd, scanner.buf + scanner.len,
             sizeof(scanner.buf) - scanner.len, 0);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      perror("recv");
      break;
    }
    if (n == 0) {
      fprintf(stderr, "connection closed by peer\n");
      break;
    }
    scanner.len += (size_t)n;

    for (;;) {
      uint8_t *payload;
      size_t plen;
      uint32_t msg_num;
      int rc;

      rc = scanner_extract(&scanner, &payload, &plen, &msg_num);
      if (rc == 0)
        break;
      if (rc < 0)
        continue;

      total_frames++;
      if (msg_num < MSG_COUNT_TABLE)
        msg_count[msg_num]++;

      if (filter_msg == 0 || (int)msg_num == filter_msg) {
        time_t now = time(NULL);
        struct tm tm_buf;
        char ts[32];

        localtime_r(&now, &tm_buf);
        strftime(ts, sizeof(ts), "%H:%M:%S", &tm_buf);
        printf("[%s] msg %u (%s)  payload=%zu B\n",
               ts, msg_num, msg_name(msg_num), plen);

        if (msg_num == 1019) {
          rtcm1019_t e;

          if (decode_1019(payload, plen, &e) == 0)
            print_1019(&e);
          else
            printf("  (1019 payload too short: %zu bytes)\n", plen);
        }
        fflush(stdout);
      }

      scanner_advance(&scanner, plen);

      if (max_frames > 0 && (long)total_frames >= max_frames) {
        g_stop = 1;
        break;
      }
    }

    if (verbose) {
      time_t now = time(NULL);

      if (now - last_summary >= summary_sec) {
        print_stats(stderr, total_frames, msg_count,
                    scanner.crc_errs, scanner.bytes_dropped);
        last_summary = now;
      }
    }
  }

  close(fd);
  fprintf(stderr, "stopping.\n");
  print_stats(stderr, total_frames, msg_count,
              scanner.crc_errs, scanner.bytes_dropped);

  return (0);
}

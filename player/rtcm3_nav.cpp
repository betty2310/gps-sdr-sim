#include "rtcm3_nav.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace {

constexpr std::uint8_t RTCM_PREAMBLE = 0xD3u;
constexpr std::size_t RTCM_MAX_PAYLOAD = 1023u;

typedef struct {
  std::uint32_t prn;
  std::uint32_t week10;
  std::uint32_t code_on_l2;
  double idot_sc;
  std::uint32_t iode;
  double toc_s;
  double af2;
  double af1;
  double af0;
  std::uint32_t iodc;
  double crs;
  double deltan_sc;
  double m0_sc;
  double cuc;
  double ecc;
  double cus;
  double sqrta;
  double toe_s;
  double cic;
  double omg0_sc;
  double cis;
  double inc0_sc;
  double crc_m;
  double aop_sc;
  double omgdot_sc;
  double tgd;
  std::uint32_t sv_health;
} rtcm1019_t;

static void setError(char *err, std::size_t err_len, const char *fmt, ...) {
  va_list ap;

  if (err == NULL || err_len == 0)
    return;

  va_start(ap, fmt);
  vsnprintf(err, err_len, fmt, ap);
  va_end(ap);
}

static std::uint32_t crc24q(const std::uint8_t *buf, std::size_t len) {
  std::uint32_t crc = 0;

  for (std::size_t i = 0; i < len; i++) {
    crc ^= (std::uint32_t)buf[i] << 16;
    for (int j = 0; j < 8; j++) {
      crc <<= 1;
      if (crc & 0x1000000u)
        crc ^= 0x1864CFBu;
    }
  }

  return (crc & 0xFFFFFFu);
}

static std::uint64_t bits_u(const std::uint8_t *buf, int pos, int len) {
  std::uint64_t value = 0;

  for (int i = 0; i < len; i++) {
    int bit_idx = pos + i;
    int byte = bit_idx >> 3;
    int bit = 7 - (bit_idx & 7);

    value = (value << 1) | (std::uint64_t)((buf[byte] >> bit) & 1u);
  }

  return (value);
}

static std::int64_t bits_s(const std::uint8_t *buf, int pos, int len) {
  std::uint64_t value = bits_u(buf, pos, len);

  if (len <= 0 || len >= 64)
    return ((std::int64_t)value);

  if (value & (1ull << (len - 1)))
    value |= (~0ull) << len;

  return ((std::int64_t)value);
}

static int fullWeekFromWeek10(std::uint32_t week10) {
  std::uint32_t week_hint =
      (std::uint32_t)((std::uint64_t)(time(NULL) - 315964800ll + 18ll) /
                      604800ull);
  std::uint32_t week_full = week10;

  while (week_full + 1024u <= week_hint)
    week_full += 1024u;

  return ((int)week_full);
}

static int decode_1019(const std::uint8_t *payload, std::size_t len,
                       rtcm1019_t *e) {
  int pos = 0;

  if (len < 61 || e == NULL)
    return (-1);

  (void)bits_u(payload, pos, 12);
  pos += 12;
  e->prn = (std::uint32_t)bits_u(payload, pos, 6);
  pos += 6;
  e->week10 = (std::uint32_t)bits_u(payload, pos, 10);
  pos += 10;
  pos += 4; /* URA */
  e->code_on_l2 = (std::uint32_t)bits_u(payload, pos, 2);
  pos += 2;
  e->idot_sc = (double)bits_s(payload, pos, 14) * std::pow(2.0, -43);
  pos += 14;
  e->iode = (std::uint32_t)bits_u(payload, pos, 8);
  pos += 8;
  e->toc_s = (double)bits_u(payload, pos, 16) * 16.0;
  pos += 16;
  e->af2 = (double)bits_s(payload, pos, 8) * std::pow(2.0, -55);
  pos += 8;
  e->af1 = (double)bits_s(payload, pos, 16) * std::pow(2.0, -43);
  pos += 16;
  e->af0 = (double)bits_s(payload, pos, 22) * std::pow(2.0, -31);
  pos += 22;
  e->iodc = (std::uint32_t)bits_u(payload, pos, 10);
  pos += 10;
  e->crs = (double)bits_s(payload, pos, 16) * std::pow(2.0, -5);
  pos += 16;
  e->deltan_sc = (double)bits_s(payload, pos, 16) * std::pow(2.0, -43);
  pos += 16;
  e->m0_sc = (double)bits_s(payload, pos, 32) * std::pow(2.0, -31);
  pos += 32;
  e->cuc = (double)bits_s(payload, pos, 16) * std::pow(2.0, -29);
  pos += 16;
  e->ecc = (double)bits_u(payload, pos, 32) * std::pow(2.0, -33);
  pos += 32;
  e->cus = (double)bits_s(payload, pos, 16) * std::pow(2.0, -29);
  pos += 16;
  e->sqrta = (double)bits_u(payload, pos, 32) * std::pow(2.0, -19);
  pos += 32;
  e->toe_s = (double)bits_u(payload, pos, 16) * 16.0;
  pos += 16;
  e->cic = (double)bits_s(payload, pos, 16) * std::pow(2.0, -29);
  pos += 16;
  e->omg0_sc = (double)bits_s(payload, pos, 32) * std::pow(2.0, -31);
  pos += 32;
  e->cis = (double)bits_s(payload, pos, 16) * std::pow(2.0, -29);
  pos += 16;
  e->inc0_sc = (double)bits_s(payload, pos, 32) * std::pow(2.0, -31);
  pos += 32;
  e->crc_m = (double)bits_s(payload, pos, 16) * std::pow(2.0, -5);
  pos += 16;
  e->aop_sc = (double)bits_s(payload, pos, 32) * std::pow(2.0, -31);
  pos += 32;
  e->omgdot_sc = (double)bits_s(payload, pos, 24) * std::pow(2.0, -43);
  pos += 24;
  e->tgd = (double)bits_s(payload, pos, 8) * std::pow(2.0, -31);
  pos += 8;
  e->sv_health = (std::uint32_t)bits_u(payload, pos, 6);

  return (0);
}

static void ephemFrom1019(const rtcm1019_t *src, ephem_t *dst) {
  gpstime_t toc;

  memset(dst, 0, sizeof(*dst));

  toc.week = fullWeekFromWeek10(src->week10);
  toc.sec = src->toc_s;

  dst->vflg = 1;
  dst->toc = toc;
  dst->toe.week = toc.week;
  dst->toe.sec = src->toe_s;
  gps2date(&toc, &dst->t);

  dst->iodc = (int)src->iodc;
  dst->iode = (int)src->iode;
  dst->deltan = src->deltan_sc * PI;
  dst->cuc = src->cuc;
  dst->cus = src->cus;
  dst->cic = src->cic;
  dst->cis = src->cis;
  dst->crc = src->crc_m;
  dst->crs = src->crs;
  dst->ecc = src->ecc;
  dst->sqrta = src->sqrta;
  dst->m0 = src->m0_sc * PI;
  dst->omg0 = src->omg0_sc * PI;
  dst->inc0 = src->inc0_sc * PI;
  dst->aop = src->aop_sc * PI;
  dst->omgdot = src->omgdot_sc * PI;
  dst->idot = src->idot_sc * PI;
  dst->af0 = src->af0;
  dst->af1 = src->af1;
  dst->af2 = src->af2;
  dst->tgd = src->tgd;
  dst->svhlth = (int)src->sv_health;
  if ((dst->svhlth > 0) && (dst->svhlth < 32))
    dst->svhlth += 32;
  dst->codeL2 = (int)src->code_on_l2;

  dst->A = dst->sqrta * dst->sqrta;
  dst->n = sqrt(GM_EARTH / (dst->A * dst->A * dst->A)) + dst->deltan;
  dst->sq1e2 = sqrt(1.0 - dst->ecc * dst->ecc);
  dst->omgkdot = dst->omgdot - OMEGA_EARTH;
}

static void scannerDropOne(rtcm3_nav_stream_t *stream) {
  if (stream->scanner_len == 0)
    return;

  memmove(stream->scanner_buf, stream->scanner_buf + 1, stream->scanner_len - 1);
  stream->scanner_len--;
  stream->bytes_dropped++;
}

static int scannerExtract(rtcm3_nav_stream_t *stream, std::uint8_t **payload,
                          std::size_t *payload_len, std::uint32_t *msg_type) {
  std::uint32_t payload_bytes;
  std::uint32_t frame_len;
  std::uint32_t crc_calc;
  std::uint32_t crc_recv;

  while (stream->scanner_len >= 1 && stream->scanner_buf[0] != RTCM_PREAMBLE)
    scannerDropOne(stream);

  if (stream->scanner_len < 3)
    return (0);

  payload_bytes = ((std::uint32_t)(stream->scanner_buf[1] & 0x03u) << 8) |
                  (std::uint32_t)stream->scanner_buf[2];
  if (payload_bytes > RTCM_MAX_PAYLOAD) {
    scannerDropOne(stream);
    return (-1);
  }

  frame_len = 3u + payload_bytes + 3u;
  if (stream->scanner_len < frame_len)
    return (0);

  crc_calc = crc24q(stream->scanner_buf, 3u + payload_bytes);
  crc_recv = ((std::uint32_t)stream->scanner_buf[3 + payload_bytes] << 16) |
             ((std::uint32_t)stream->scanner_buf[3 + payload_bytes + 1] << 8) |
             ((std::uint32_t)stream->scanner_buf[3 + payload_bytes + 2]);
  if (crc_calc != crc_recv) {
    stream->crc_errs++;
    scannerDropOne(stream);
    return (-1);
  }

  *payload = stream->scanner_buf + 3;
  *payload_len = payload_bytes;
  *msg_type =
      payload_bytes >= 2 ? (std::uint32_t)bits_u(stream->scanner_buf + 3, 0, 12)
                         : 0u;

  return (1);
}

static void scannerAdvance(rtcm3_nav_stream_t *stream, std::size_t payload_len) {
  std::size_t frame_len = 3u + payload_len + 3u;

  if (frame_len >= stream->scanner_len) {
    stream->scanner_len = 0;
    return;
  }

  memmove(stream->scanner_buf, stream->scanner_buf + frame_len,
          stream->scanner_len - frame_len);
  stream->scanner_len -= frame_len;
}

static void appendBytes(rtcm3_nav_stream_t *stream, const std::uint8_t *data,
                        std::size_t len) {
  if (len > sizeof(stream->scanner_buf)) {
    data += len - sizeof(stream->scanner_buf);
    len = sizeof(stream->scanner_buf);
  }

  if (stream->scanner_len + len > sizeof(stream->scanner_buf)) {
    std::size_t drop = stream->scanner_len + len - sizeof(stream->scanner_buf);

    memmove(stream->scanner_buf, stream->scanner_buf + drop,
            stream->scanner_len - drop);
    stream->scanner_len -= drop;
    stream->bytes_dropped += drop;
  }

  memcpy(stream->scanner_buf + stream->scanner_len, data, len);
  stream->scanner_len += len;
}

static int waitSocket(int fd, short events, int timeout_ms, char *err,
                      std::size_t err_len) {
  struct pollfd pfd = {fd, events, 0};
  int rc;

  do {
    rc = poll(&pfd, 1, timeout_ms);
  } while (rc < 0 && errno == EINTR);

  if (rc < 0) {
    setError(err, err_len, "poll() failed: %s", strerror(errno));
    return (FALSE);
  }
  if (rc == 0) {
    setError(err, err_len, "socket timed out");
    return (FALSE);
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    setError(err, err_len, "socket closed during I/O");
    return (FALSE);
  }

  return (TRUE);
}

static int tcpConnect(const char *host, int port, int timeout_ms, char *err,
                      std::size_t err_len) {
  struct addrinfo hints;
  struct addrinfo *res = NULL;
  struct addrinfo *rp;
  char port_str[16];
  int fd = -1;
  int rc;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  snprintf(port_str, sizeof(port_str), "%d", port);

  rc = getaddrinfo(host, port_str, &hints, &res);
  if (rc != 0 || res == NULL) {
    setError(err, err_len, "Cannot resolve host %s", host);
    return (-1);
  }

  for (rp = res; rp != NULL; rp = rp->ai_next) {
    int flags;

    fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (fd < 0)
      continue;

    flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0)
      fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    rc = connect(fd, rp->ai_addr, rp->ai_addrlen);
    if (rc < 0 && errno != EINPROGRESS) {
      close(fd);
      fd = -1;
      continue;
    }

    if (rc < 0) {
      if (waitSocket(fd, POLLOUT, timeout_ms, err, err_len) == FALSE) {
        close(fd);
        fd = -1;
        continue;
      }

      int so_err = 0;
      socklen_t so_len = sizeof(so_err);
      getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_err, &so_len);
      if (so_err != 0) {
        setError(err, err_len, "connect() to %s:%d failed: %s", host, port,
                 strerror(so_err));
        close(fd);
        fd = -1;
        continue;
      }
    }

    if (flags >= 0)
      fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

    break;
  }

  freeaddrinfo(res);

  if (fd < 0 && (err == NULL || err[0] == '\0'))
    setError(err, err_len, "connect() to %s:%d failed", host, port);

  return (fd);
}

static int setNonBlocking(int fd, int nonblocking, char *err,
                          std::size_t err_len) {
  int flags = fcntl(fd, F_GETFL, 0);

  if (flags < 0) {
    setError(err, err_len, "fcntl(F_GETFL) failed: %s", strerror(errno));
    return (FALSE);
  }

  if (nonblocking)
    flags |= O_NONBLOCK;
  else
    flags &= ~O_NONBLOCK;

  if (fcntl(fd, F_SETFL, flags) < 0) {
    setError(err, err_len, "fcntl(F_SETFL) failed: %s", strerror(errno));
    return (FALSE);
  }

  return (TRUE);
}

static void base64Encode(const char *in, char *out, std::size_t out_size) {
  static const char T[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::size_t in_len = strlen(in);
  std::size_t o = 0;

  for (std::size_t i = 0; i < in_len && o + 4 < out_size; i += 3) {
    std::uint8_t a = (std::uint8_t)in[i];
    std::uint8_t b = (i + 1 < in_len) ? (std::uint8_t)in[i + 1] : 0;
    std::uint8_t c = (i + 2 < in_len) ? (std::uint8_t)in[i + 2] : 0;

    out[o++] = T[a >> 2];
    out[o++] = T[((a & 0x3u) << 4) | (b >> 4)];
    out[o++] = (i + 1 < in_len) ? T[((b & 0xFu) << 2) | (c >> 6)] : '=';
    out[o++] = (i + 2 < in_len) ? T[c & 0x3Fu] : '=';
  }

  out[o] = '\0';
}

static int sendAll(int fd, const char *buf, std::size_t len, int timeout_ms,
                   char *err, std::size_t err_len) {
  std::size_t off = 0;

  while (off < len) {
    ssize_t n;

    if (waitSocket(fd, POLLOUT, timeout_ms, err, err_len) == FALSE)
      return (FALSE);

    n = send(fd, buf + off, len - off, 0);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      setError(err, err_len, "send() failed: %s", strerror(errno));
      return (FALSE);
    }

    off += (std::size_t)n;
  }

  return (TRUE);
}

static int ntripHandshake(int fd, const char *mount_point,
                          const char *credentials, int timeout_ms, char *err,
                          std::size_t err_len) {
  char req[1024];
  char hdr[4096];
  int req_len;
  int h = 0;

  if (credentials != NULL && credentials[0] != '\0') {
    char auth_b64[512];

    base64Encode(credentials, auth_b64, sizeof(auth_b64));
    req_len = snprintf(req, sizeof(req),
                       "GET /%s HTTP/1.0\r\n"
                       "User-Agent: NTRIP bladetx/1.0\r\n"
                       "Authorization: Basic %s\r\n"
                       "\r\n",
                       mount_point, auth_b64);
  } else {
    req_len = snprintf(req, sizeof(req),
                       "GET /%s HTTP/1.0\r\n"
                       "User-Agent: NTRIP bladetx/1.0\r\n"
                       "\r\n",
                       mount_point);
  }

  if (req_len <= 0 || (std::size_t)req_len >= sizeof(req)) {
    setError(err, err_len, "NTRIP request too long");
    return (FALSE);
  }

  if (sendAll(fd, req, (std::size_t)req_len, timeout_ms, err, err_len) == FALSE)
    return (FALSE);

  memset(hdr, 0, sizeof(hdr));
  while (h < (int)sizeof(hdr) - 1) {
    ssize_t n;

    if (waitSocket(fd, POLLIN, timeout_ms, err, err_len) == FALSE)
      return (FALSE);

    n = recv(fd, hdr + h, 1, 0);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      setError(err, err_len, "recv() failed during NTRIP handshake: %s",
               strerror(errno));
      return (FALSE);
    }
    if (n == 0) {
      setError(err, err_len, "NTRIP connection closed during handshake");
      return (FALSE);
    }

    h++;
    if (h >= 4 && hdr[h - 4] == '\r' && hdr[h - 3] == '\n' &&
        hdr[h - 2] == '\r' && hdr[h - 1] == '\n')
      break;
  }

  if (strstr(hdr, "200") == NULL) {
    setError(err, err_len,
             "NTRIP handshake failed: check mount point and credentials");
    return (FALSE);
  }

  return (TRUE);
}

} // namespace

void rtcm3_nav_init(rtcm3_nav_stream_t *stream) {
  memset(stream, 0, sizeof(*stream));
  stream->fd = -1;
}

void rtcm3_nav_close(rtcm3_nav_stream_t *stream) {
  if (stream == NULL)
    return;

  if (stream->fd >= 0)
    close(stream->fd);

  stream->fd = -1;
}

int rtcm3_nav_open(rtcm3_nav_stream_t *stream, const rtcm3_nav_options_t *opt,
                   char *err, std::size_t err_len) {
  rtcm3_nav_init(stream);

  if (opt == NULL || opt->host == NULL || opt->host[0] == '\0') {
    setError(err, err_len, "RTCM host is required");
    return (FALSE);
  }

  stream->fd = tcpConnect(opt->host, opt->port, opt->timeout_ms, err, err_len);
  if (stream->fd < 0)
    return (FALSE);

  if (opt->mount_point != NULL && opt->mount_point[0] != '\0') {
    if (ntripHandshake(stream->fd, opt->mount_point, opt->credentials,
                       opt->timeout_ms, err, err_len) == FALSE) {
      rtcm3_nav_close(stream);
      return (FALSE);
    }
  }

  if (setNonBlocking(stream->fd, TRUE, err, err_len) == FALSE) {
    rtcm3_nav_close(stream);
    return (FALSE);
  }

  return (TRUE);
}

int rtcm3_nav_pump(rtcm3_nav_stream_t *stream, int timeout_ms, int *updated,
                   char *err, std::size_t err_len) {
  struct pollfd pfd;
  std::uint8_t recv_buf[4096];
  int any_update = FALSE;

  if (updated != NULL)
    *updated = FALSE;

  if (stream == NULL || stream->fd < 0) {
    setError(err, err_len, "RTCM stream is not connected");
    return (FALSE);
  }

  pfd.fd = stream->fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  {
    int rc;

    do {
      rc = poll(&pfd, 1, timeout_ms);
    } while (rc < 0 && errno == EINTR);

    if (rc < 0) {
      setError(err, err_len, "poll() failed: %s", strerror(errno));
      return (FALSE);
    }
    if (rc == 0)
      return (TRUE);
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      setError(err, err_len, "RTCM socket closed");
      return (FALSE);
    }
  }

  for (;;) {
    ssize_t n = recv(stream->fd, recv_buf, sizeof(recv_buf), 0);

    if (n < 0) {
      if (errno == EINTR)
        continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK)
        break;

      setError(err, err_len, "recv() failed: %s", strerror(errno));
      return (FALSE);
    }

    if (n == 0) {
      setError(err, err_len, "RTCM socket closed");
      return (FALSE);
    }

    appendBytes(stream, recv_buf, (std::size_t)n);

    for (;;) {
      std::uint8_t *payload = NULL;
      std::size_t payload_len = 0;
      std::uint32_t msg_type = 0;
      int rc = scannerExtract(stream, &payload, &payload_len, &msg_type);

      if (rc == 0)
        break;
      if (rc < 0)
        continue;

      if (msg_type == 1019u) {
        rtcm1019_t decoded;

        if (decode_1019(payload, payload_len, &decoded) == 0 &&
            decoded.prn >= 1u && decoded.prn <= (std::uint32_t)MAX_SAT) {
          ephem_t eph;
          int sv = (int)decoded.prn - 1;

          ephemFrom1019(&decoded, &eph);
          if (stream->valid[sv] != TRUE ||
              memcmp(&stream->gps_rtcm_eph[sv], &eph, sizeof(ephem_t)) != 0)
            any_update = TRUE;

          if (stream->valid[sv] != TRUE)
            stream->valid_count++;
          stream->valid[sv] = TRUE;
          stream->gps_rtcm_eph[sv] = eph;
        }
      }

      scannerAdvance(stream, payload_len);
    }
  }

  if (updated != NULL)
    *updated = any_update;

  return (TRUE);
}

void rtcm3_nav_copy_ephemeris(const rtcm3_nav_stream_t *stream, ephem_t *dst) {
  memcpy(dst, stream->gps_rtcm_eph, sizeof(ephem_t) * MAX_SAT);
}

int rtcm3_nav_has_prn(const rtcm3_nav_stream_t *stream, int prn) {
  if (stream == NULL || prn < 1 || prn > MAX_SAT)
    return (FALSE);

  return (stream->valid[prn - 1] == TRUE);
}

int rtcm3_nav_valid_prns(const rtcm3_nav_stream_t *stream) {
  if (stream == NULL)
    return (0);

  return (stream->valid_count);
}

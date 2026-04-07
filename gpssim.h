#ifndef GPSSIM_H
#define GPSSIM_H

#include <signal.h>

/* Ensure sig_atomic_t is available before extern "C" block */
#ifdef __cplusplus
extern "C" {
#endif

//#define FLOAT_CARR_PHASE // For RKT simulation. Higher computational load, but smoother carrier phase.

#define TRUE	(1)
#define FALSE	(0)

/*! \brief Maximum length of a line in a text file (RINEX, motion) */
#define MAX_CHAR (100)

/*! \brief Maximum number of satellites in RINEX file */
#define MAX_SAT (32)

/*! \brief Maximum number of channels we simulate */
#define MAX_CHAN (16)

/*! \brief Maximum number of user motion points */
#ifndef USER_MOTION_SIZE
#define USER_MOTION_SIZE (3000) // max duration at 10Hz
#endif

/*! \brief Maximum duration for static mode*/
#define STATIC_MAX_DURATION (86400) // second

/*! \brief Number of subframes */
#define N_SBF (5) // 5 subframes per frame

/*! \brief Number of words per subframe */
#define N_DWRD_SBF (10) // 10 word per subframe

/*! \brief Number of words */
#define N_DWRD ((N_SBF+1)*N_DWRD_SBF) // Subframe word buffer size

/*! \brief C/A code sequence length */
#define CA_SEQ_LEN (1023)

#define SECONDS_IN_WEEK 604800.0
#define SECONDS_IN_HALF_WEEK 302400.0
#define SECONDS_IN_DAY 86400.0
#define SECONDS_IN_HOUR 3600.0
#define SECONDS_IN_MINUTE 60.0

#define POW2_M5  0.03125
#define POW2_M19 1.907348632812500e-6
#define POW2_M29 1.862645149230957e-9
#define POW2_M31 4.656612873077393e-10
#define POW2_M33 1.164153218269348e-10
#define POW2_M43 1.136868377216160e-13
#define POW2_M55 2.775557561562891e-17

#define POW2_M50 8.881784197001252e-016
#define POW2_M30 9.313225746154785e-010
#define POW2_M27 7.450580596923828e-009
#define POW2_M24 5.960464477539063e-008

// Conventional values employed in GPS ephemeris model (ICD-GPS-200)
#define GM_EARTH 3.986005e14
#define OMEGA_EARTH 7.2921151467e-5
#define PI 3.1415926535898

#define WGS84_RADIUS	6378137.0
#define WGS84_ECCENTRICITY 0.0818191908426

#define R2D 57.2957795131

#define SPEED_OF_LIGHT 2.99792458e8
#define LAMBDA_L1 0.190293672798365

/*! \brief GPS L1 Carrier frequency */
#define CARR_FREQ (1575.42e6)
/*! \brief C/A code frequency */
#define CODE_FREQ (1.023e6)
#define CARR_TO_CODE (1.0/1540.0)

// Sampling data format
#define SC01 (1)
#define SC08 (8)
#define SC16 (16)

#define EPHEM_ARRAY_SIZE (15) // for daily GPS broadcast ephemers file (brdc)

// Synthetic satellite constants
#define GPS_ORBIT_RADIUS 26559700.0  // GPS semi-major axis (meters)
#define GPS_INCLINATION 0.9599310886 // ~55 degrees in radians
#define D2R (PI / 180.0)

// Jammer noise normalization constant
#define JAM_NOISE_RMS_SCALE 72.31

/*! \brief Structure representing GPS time */
typedef struct
{
	int week;	/*!< GPS week number (since January 1980) */
	double sec; 	/*!< second inside the GPS \a week */
} gpstime_t;

/*! \brief Structure repreenting UTC time */
typedef struct
{
	int y; 		/*!< Calendar year */
	int m;		/*!< Calendar month */
	int d;		/*!< Calendar day */
	int hh;		/*!< Calendar hour */
	int mm;		/*!< Calendar minutes */
	double sec;	/*!< Calendar seconds */
} datetime_t;

/*! \brief Structure representing ephemeris of a single satellite */
typedef struct
{
	int vflg;	/*!< Valid Flag */
	datetime_t t;
	gpstime_t toc;	/*!< Time of Clock */
	gpstime_t toe;	/*!< Time of Ephemeris */
	int iodc;	/*!< Issue of Data, Clock */
	int iode;	/*!< Isuse of Data, Ephemeris */
	double deltan;	/*!< Delta-N (radians/sec) */
	double cuc;	/*!< Cuc (radians) */
	double cus;	/*!< Cus (radians) */
	double cic;	/*!< Correction to inclination cos (radians) */
	double cis;	/*!< Correction to inclination sin (radians) */
	double crc;	/*!< Correction to radius cos (meters) */
	double crs;	/*!< Correction to radius sin (meters) */
	double ecc;	/*!< e Eccentricity */
	double sqrta;	/*!< sqrt(A) (sqrt(m)) */
	double m0;	/*!< Mean anamoly (radians) */
	double omg0;	/*!< Longitude of the ascending node (radians) */
	double inc0;	/*!< Inclination (radians) */
	double aop;
	double omgdot;	/*!< Omega dot (radians/s) */
	double idot;	/*!< IDOT (radians/s) */
	double af0;	/*!< Clock offset (seconds) */
	double af1;	/*!< rate (sec/sec) */
	double af2;	/*!< acceleration (sec/sec^2) */
	double tgd;	/*!< Group delay L2 bias */
	int svhlth;
	int codeL2;
	// Working variables follow
	double n; 	/*!< Mean motion (Average angular velocity) */
	double sq1e2;	/*!< sqrt(1-e^2) */
	double A;	/*!< Semi-major axis */
	double omgkdot; /*!< OmegaDot-OmegaEdot */
} ephem_t;

typedef struct
{
	int enable;
	int vflg;
	double alpha0,alpha1,alpha2,alpha3;
	double beta0,beta1,beta2,beta3;
	double A0,A1;
	int dtls,tot,wnt;
	int dtlsf,dn,wnlsf;
	// enable custom leap event
	int leapen;
} ionoutc_t;

typedef struct
{
	gpstime_t g;
	double range; // pseudorange
	double rate;
	double d; // geometric distance
	double azel[2];
	double iono_delay;
} range_t;

/*! \brief Structure representing a Channel */
typedef struct
{
	int prn;	/*< PRN Number */
	int ca[CA_SEQ_LEN]; /*< C/A Sequence */
	double f_carr;	/*< Carrier frequency */
	double f_code;	/*< Code frequency */
#ifdef FLOAT_CARR_PHASE
	double carr_phase;
#else
	unsigned int carr_phase; /*< Carrier phase */
	int carr_phasestep;	/*< Carrier phasestep */
#endif
	double code_phase; /*< Code phase */
	gpstime_t g0;	/*!< GPS time at start */
	unsigned long sbf[5][N_DWRD_SBF]; /*!< current subframe */
	unsigned long dwrd[N_DWRD]; /*!< Data words of sub-frame */
	int iword;	/*!< initial word */
	int ibit;	/*!< initial bit */
	int icode;	/*!< initial code */
	int dataBit;	/*!< current data bit */
	int codeCA;	/*!< current C/A code */
	double azel[2];
	range_t rho0;
} channel_t;

/*! \brief Attack method */
typedef enum {
	ATTACK_METHOD_NONE = 0,
	ATTACK_METHOD_JAM_DROP,
	ATTACK_METHOD_JAM_NOISE,
	ATTACK_METHOD_SPOOF_DELAY,
	ATTACK_METHOD_SPOOF_NAV
} attack_method_t;

/*! \brief Attack configuration */
typedef struct {
	attack_method_t method[MAX_SAT];
	double jam_js_db;        /*!< Jammer-to-signal ratio in dB for jam_noise */
	int prn_select[MAX_SAT]; /*!< 1 = render this PRN, 0 = suppress */
	int partial_mode;        /*!< 1 = only render selected PRNs */
	double gain_boost_db;    /*!< Power boost in dB for partial-mode PRNs */
} attack_config_t;

/*! \brief Synthetic satellite mode */
typedef enum {
	SYNTH_NONE = 0,
	SYNTH_FORCE,      /*!< Bypass elevation check (requires ephemeris in RINEX) */
	SYNTH_OVERHEAD,   /*!< Synthesize circular orbit placing satellite at zenith */
	SYNTH_AZEL        /*!< Synthesize circular orbit at specified azimuth/elevation */
} synth_mode_t;

/*! \brief Configuration for synthetic satellite generation */
typedef struct {
	synth_mode_t mode[MAX_SAT];
	double azimuth[MAX_SAT];    /*!< Target azimuth in radians (SYNTH_AZEL) */
	double elevation[MAX_SAT];  /*!< Target elevation in radians (SYNTH_AZEL) */
	int enabled;                /*!< 1 if any synthetic satellite is configured */
} synth_config_t;

/*! \brief Synthesized ephemerides kept separate from real RINEX sets */
typedef struct {
	int valid[MAX_SAT];
	ephem_t eph[MAX_SAT];
} synth_ephem_store_t;

////////////////////////////////////////////////////////////
// Global data (defined in gpssim.c)
////////////////////////////////////////////////////////////

extern int sinTable512[];
extern int cosTable512[];
extern double ant_pat_db[];
extern int allocatedSat[];
extern double xyz[][3];
extern volatile sig_atomic_t stop_requested;

////////////////////////////////////////////////////////////
// Function declarations
////////////////////////////////////////////////////////////

/* Clipping */
short clipInt16(int x);
int clipInt32FromDouble(double x);

/* Attack / partial constellation */
void initAttackConfig(attack_config_t *cfg);
void initAttackNoiseState(unsigned int noise_state[MAX_SAT]);
int nextNoiseValue(unsigned int *state);
int nextGaussianNoise(unsigned int *state);
const char *attackMethodName(attack_method_t method);
int parseAttackMethod(const char *name, attack_method_t *method);
int parseAttackConfig(attack_config_t *cfg, const char *spec);
int parsePartialPrns(attack_config_t *cfg, const char *spec);
int hasUnimplementedAttack(const attack_config_t *cfg);
attack_method_t getAttackMethod(const attack_config_t *cfg, int prn);
void applyGainAttack(const attack_config_t *cfg, int prn, int *gain);

/* Angle utilities */
double wrapToPi(double angle);

/* Synthetic satellites */
void initSynthConfig(synth_config_t *cfg);
void initSynthEphemStore(synth_ephem_store_t *store);
int getSetReferenceToc(const ephem_t *eph_set, gpstime_t *toc);
void overlaySyntheticEphemerisSet(ephem_t *dst, const ephem_t *real_set,
    const synth_config_t *cfg, const synth_ephem_store_t *store);
int parseSynthConfig(synth_config_t *cfg, const char *spec);
void azel2satpos(const double *rx_xyz, double az, double el, double *sat_ecef);
void synthEphemeris(ephem_t *eph, const double *rx_xyz, double az, double el,
    gpstime_t toe, gpstime_t toc);

/* Vector math */
void subVect(double *y, const double *x1, const double *x2);
double normVect(const double *x);
double dotProd(const double *x1, const double *x2);

/* C/A code generation */
void codegen(int *ca, int prn);

/* Time conversions */
void date2gps(const datetime_t *t, gpstime_t *g);
void gps2date(const gpstime_t *g, datetime_t *t);
double subGpsTime(gpstime_t g1, gpstime_t g0);
gpstime_t incGpsTime(gpstime_t g0, double dt);
int shouldAdvanceEphSet(gpstime_t next_toc, gpstime_t grx);

/* Coordinate transforms */
void xyz2llh(const double *xyz, double *llh);
void llh2xyz(const double *llh, double *xyz);
void ltcmat(const double *llh, double t[3][3]);
void ecef2neu(const double *xyz, double t[3][3], double *neu);
void neu2azel(double *azel, const double *neu);

/* Ephemeris */
int replaceExpDesignator(char *str, int len);
int readRinexNavAll(ephem_t eph[][MAX_SAT], ionoutc_t *ionoutc, const char *fname);
void satpos(ephem_t eph, gpstime_t g, double *pos, double *vel, double *clk);
void eph2sbf(const ephem_t eph, const ionoutc_t ionoutc,
    unsigned long sbf[5][N_DWRD_SBF]);
unsigned long countBits(unsigned long v);
unsigned long computeChecksum(unsigned long source, int nib);

/* Ionospheric delay */
double ionosphericDelay(const ionoutc_t *ionoutc, gpstime_t g, double *llh,
    double *azel);

/* Range computation */
void computeRange(range_t *rho, ephem_t eph, ionoutc_t *ionoutc, gpstime_t g,
    double xyz[]);
void computeCodePhase(channel_t *chan, range_t rho1, double dt);

/* User motion */
int readUserMotion(double xyz[USER_MOTION_SIZE][3], const char *filename);
int readUserMotionLLH(double xyz[USER_MOTION_SIZE][3], const char *filename);
int readNmeaGGA(double xyz[USER_MOTION_SIZE][3], const char *filename);

/* Navigation message */
int generateNavMsg(gpstime_t g, channel_t *chan, int init);

/* Satellite visibility and channel allocation */
int checkSatVisibility(ephem_t eph, gpstime_t g, double *xyz, double elvMask,
    double *azel);
int allocateChannel(channel_t *chan, ephem_t *eph, ionoutc_t ionoutc,
    gpstime_t grx, double *xyz, double elvMask,
    const attack_config_t *acfg, const synth_config_t *scfg);

/* Usage */
void usage(void);

#ifdef __cplusplus
}
#endif

#endif

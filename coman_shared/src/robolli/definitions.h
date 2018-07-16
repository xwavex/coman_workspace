/*
	defienitions.h

	Copyright (C) 2012 Italian Institute of Technology

	Developer:
        Alessio Margan (2012-, alessio.margan@iit.it)

*/

#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <stdint.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <broadcast_data.h>

#define MAX_DSP_BOARDS  40//32
#define MAX_MC_BOARDS   34//25
#define MAX_FT_BOARDS   4//2

#define BCAST_MC_DATA_PACKETS 0xBB // BCAST_DATA_PACKETS MotorController
#define BCAST_FT_DATA_PACKETS 0xBC // BCAST_DATA_PACKETS ForceTorqueSens
#define BCAST_MA_DATA_PACKETS 0xBD // BCAST_DATA_PACKETS MultiAxis ?!?

#define _1_SEC          1000
// 10 mins at 1Khz loop
//#define LOG_SIZE    _1_SEC * 60 * 10
// 10 secs at 1Khz loop
#define LOG_SIZE    _1_SEC * 60

/**
 * @enum GainSet
 * @ingroup RoboMotorController
 * Set of gains of different controller types.
 */
#ifdef C_WRAPPER
    typedef char GainSet;
    #define	VELOCITY_GAINS 0
    #define	POSITION_GAINS 1
    #define	TORQUE_GAINS   2
#else
    enum GainSet
    {
    	VELOCITY_GAINS = 0,
    	POSITION_GAINS = 1,
    	TORQUE_GAINS   = 2
    };
#endif

#define MV_POS 0x0001
#define MV_VEL 0x0002
#define MV_TOR 0x0004
#define MV_STF 0x0008


#define FAULT_CURRENT_HIGH      0x00000001
#define FAULT_TEMPERATURE_HIGH  0x00000002
#define FAULT_VBUS_HIGH         0x00000004
#define FAULT_WATCHDOG          0x00000008
#define FAULT_EMERGENCY_STOP    0x00000010
#define FAULT_TORQUE_DATA       0x00000020


/**
 * @defgroup DataStructures Data Structures
 * @ingroup RoboLLI
 * @brief Here are the data structures with brief descriptions:
 *
 * @{
 */

/**
 */
typedef struct {
    uint64_t    ts;
    uint16_t    mc_bc_policy;
    uint16_t    mc_bc_extra_policy;
    uint16_t    ft_bc_policy;
} info_data_t;


/**
 * common udp header tx by DSP boards
 */
typedef struct {
    unsigned char _header;
	unsigned char _n_bytes;
	unsigned char _command;
	unsigned char _board_id;
} bc_header_t;

/**
 * convenient way to access broadcast data
 */
typedef	union {
    bc_header_t     bc_header;
    mc_bc_data_t    mc_bc_data;
    ft_bc_data_t    ft_bc_data;
} bc_data_t;

/**
 * timestamped DSP broadcast data
 */
typedef struct {
    uint64_t    ts_rx;
    bc_data_t   raw_bc_data;
} ts_bc_data_t;

/**
 * ...
 */
struct __pid_gains_t {
    char    gain_set;
    int     p;
    int     i;
    int     d;
}  __attribute__((__packed__));
typedef __pid_gains_t pid_gains_t;

/**
 * ...
 */
typedef struct {
    int     multiplier;
    short   offset;
} __attribute__((__packed__)) torque_factor_t;


/**
 */
typedef std::map<uint8_t, int> group_ref_t;
/**
 */
typedef std::map<uint8_t, std::pair<int,int> > group_ref_comp_t;


#if __XENO__
    static const std::string pipe_prefix("/proc/xenomai/registry/rtipc/xddp/");
#else
    static const std::string pipe_prefix("/tmp/");
#endif

/** @}
 */

#endif



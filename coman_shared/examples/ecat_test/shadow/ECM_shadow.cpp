#include <assert.h>
#include <signals.h>

#include "ECM_shadow.h"
#include "ec_slaves_struct.h"

/* 
ethercat slaves
0  49152:0  PREOP  +  0x00000530:0x00000007
1  49152:1  PREOP  E  0x00000000:0x00000000 
|  |     |  |      |  |
|  |     |  |      |  \- Name from the SII if avaliable ,
|  |     |  |      |   otherwise vendor ID and product
|  |     |  |      |   code ( both hexadecimal ).
|  |     |  |      \- Error flag . '+' means no error ,
|  |     |  |      'E' means that scan or
|  |     |  |       configuration failed .
|  |     |  \- Current application - layer state .
|  |     \- Decimal relative position to the last
|  |      slave with an alias address set.
|  \- Decimal alias address of this slave (if set),
|   otherwise of the last slave with an alias set ,
|   or zero , if no alias was encountered up to this
|   position .
\- Absolute ring position in the bus. 
 
*/

// alias, position
#define BusCoupler_Pos  49152, 0
#define Slave01_Pos     49152, 1
#define Slave02_Pos     49152, 2
#define Slave03_Pos     49152, 3

// vendor_id, product_code
#define Shadow_Ronex_Bridge 0x00000530, 0x00000007
#define Shadow_Ronex_Slave  0x00000530, 0X00000424
//#define Shadow_Ronex_Slave  0x00000000, 0x00000000

typedef union {

    unsigned int command_as_uint[8];
    struct {
        unsigned short  dig_out;        //  2
        unsigned int    pwm[4];         // 16
        unsigned short  ana_out[2];     //  4
    };
} ronex_command_t;

typedef union {

    unsigned int status_as_uint[3];
    struct {
        unsigned char  dig_in;        //  2
        unsigned short ana_in[4];    // 8
    };
} ronex_status_t;


static ronex_command_t  ronex_command;
static ronex_status_t   ronex_status;

/* List record type for PDO entry mass-registration.
 *
 * This type is used for the array parameter of the
 * ecrt_domain_reg_pdo_entry_list()
 *
typedef struct {
   uint16_t alias;      < Slave alias address.
   uint16_t position;   < Slave position.
   uint32_t vendor_id;  < Slave vendor ID.
   uint32_t product_code; < Slave product code.
   uint16_t index;      < PDO entry index.
   uint8_t subindex;    < PDO entry subindex.
   unsigned int offset; < Pointer to a variable to store the PDO entry's (byte-)offset in the process data.
   unsigned int bit_position; < Pointer to a variable to store a bit position (0-7) within the offset. Can be NULL, in which case an error does not byte-align.
} ec_pdo_entry_reg_t;
*/

static unsigned int command_offset;
static unsigned int status_offset;

static const ec_pdo_entry_reg_t regs[] = {

    {Slave01_Pos, Shadow_Ronex_Slave, 0x1000, 0x1, &command_offset},
    {Slave01_Pos, Shadow_Ronex_Slave, 0x1016, 0x1, &status_offset},

    //{Slave02_Pos, Shadow_Ronex_Slave, 0x1000, 0x1, &command_offset},
    //{Slave02_Pos, Shadow_Ronex_Slave, 0x1016, 0x1, &status_offset},

    //{Slave03_Pos, Shadow_Ronex_Slave, 0x1000, 0x1, &command_offset},
    //{Slave03_Pos, Shadow_Ronex_Slave, 0x1016, 0x1, &status_offset},

    {}
};


ECat_master_shadow::ECat_master_shadow() : ECat_master(regs, 100) {

    name = "ecat_master";
    period.period = {0,1000}; 

    schedpolicy = SCHED_FIFO;
    priority = sched_get_priority_max(schedpolicy);
    stacksize = PTHREAD_STACK_MIN;
}

ECat_master_shadow::~ECat_master_shadow() {

    std::cout << "Count loop time : " << (uint64_t)count(loop_time) << std::endl;
    std::cout << "Mean loop time : " << (uint64_t)mean(loop_time) << std::endl;
    std::cout << "Min loop time  : " << min(loop_time) << std::endl;
    std::cout << "Max loop time  : " << max(loop_time) << std::endl;
    std::cout << "Var loop time  : " << variance(loop_time) << std::endl;
    std::cout << "Mean error loop time  : " << boost::accumulators::error_of<tag::mean>(loop_time) << std::endl;

}

void ECat_master_shadow::th_init(void *) { 

    init();
}

void ECat_master_shadow::th_loop(void *) { 

    static uint64_t tPrev;
    uint64_t tNow;

    tNow = get_time_ns();
    if (tPrev) {
        loop_time(tNow-tPrev);
    }
    tPrev = tNow;

}

/** Obtains a slave configuration.
 *
 * Creates a slave configuration object for the given \a alias and \a position
 * tuple and returns it. If a configuration with the same \a alias and \a
 * position already exists, it will be re-used. In the latter case, the given
 * vendor ID and product code are compared to the stored ones. On mismatch, an
 * error message is raised and the function returns \a NULL.
 *
 * Slaves are addressed with the \a alias and \a position parameters.
 * - If \a alias is zero, \a position is interpreted as the desired slave's
 *   ring position.
 * - If \a alias is non-zero, it matches a slave with the given alias. In this
 *   case, \a position is interpreted as ring offset, starting from the
 *   aliased slave, so a position of zero means the aliased slave itself and a
 *   positive value matches the n-th slave behind the aliased one.
 *
 * If the slave with the given address is found during the bus configuration,
 * its vendor ID and product code are matched against the given value. On
 * mismatch, the slave is not configured and an error message is raised.
 *
 * If different slave configurations are pointing to the same slave during bus
 * configuration, a warning is raised and only the first configuration is
 * applied.
 *
 * \retval >0 Pointer to the slave configuration structure.
 * \retval NULL in the error case.
 */
/*
ec_slave_config_t *ecrt_master_slave_config(                
        ec_master_t *master, /**< EtherCAT master         
        uint16_t alias, /**< Slave alias.                
        uint16_t position, /**< Slave position.           
        uint32_t vendor_id, /**< Expected vendor ID.      
        uint32_t product_code /**< Expected product code. 
        );                                                  
*/



int ECat_master_shadow::config_PDOs(void) {

    printf("Configuring shadow...\n");

    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, Slave01_Pos, Shadow_Ronex_Slave),
                               EC_END, slave_shadow_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    //if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, Slave02_Pos, Shadow_Ronex_Slave),
    //                           EC_END, slave_shadow_syncs)) {
    //    fprintf(stderr, "Failed to configure PDOs.\n");
    //    return -1;
    //}

    // Create configuration for bus coupler
    if ( ! ecrt_master_slave_config(master, BusCoupler_Pos, Shadow_Ronex_Bridge))
        return -1;

    return 0;
}


void ECat_master_shadow::ec_hook_loop(void * arg) {

    static int cycle_counter = 0;
    static int blink;
    uint16_t ain0;

    cycle_counter++;

    if (!(cycle_counter % 1000)) {
        blink = !blink;
    }

    if (blink) {
        EC_WRITE_U16(domain_pd + command_offset, (uint16_t)0x55);
    } else {
        EC_WRITE_U16(domain_pd + command_offset, (uint16_t)0x00);

    }


}


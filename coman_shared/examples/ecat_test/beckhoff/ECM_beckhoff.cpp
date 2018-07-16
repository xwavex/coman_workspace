#include <assert.h>
#include <signals.h>

#include "ECM_beckhoff.h"
#include "ec_slaves_struct.h"

#define BusCoupler01_Pos    0, 0
#define AnaInpSlave01_Pos   1, 0
#define AnaOutSlave01_Pos   1, 1
#define EncIncSlave01_Pos   1, 2
#define EncSsiSlave01_Pos   1, 3

#define BusCoupler02_Pos    1, 4
#define AnaInpSlave02_Pos   1, 5
#define AnaOutSlave02_Pos   1, 6
#define EncIncSlave02_Pos   1, 7
#define EncSsiSlave02_Pos   1, 8

#define Beckhoff_EK1100 0x00000002, 0x044c2c52
#define Beckhoff_EL3104 0x00000002, 0x0c203052
#define Beckhoff_EL4134 0x00000002, 0x10263052
#define Beckhoff_EL5101 0x00000002, 0x13ed3052
#define Beckhoff_EL5001 0x00000002, 0x13893052


// offsets for PDO entries
static unsigned int off_ana_in0_value;
static unsigned int off_ana_in1_value;
static unsigned int off_ana_in2_value;
static unsigned int off_ana_in3_value;
static unsigned int off_ana_out0     ;
static unsigned int off_ana_out1     ;
static unsigned int off_ana_out2     ;
static unsigned int off_ana_out3     ;
static unsigned int off_enc_inc1_out_ctrl; 
static unsigned int off_enc_inc1_out_value;
static unsigned int off_enc_inc1_in_ctrl;  
static unsigned int off_enc_inc1_in_value; 
static unsigned int off_enc_inc1_in_latch; 
static unsigned int off_enc_ssi1_status;
static unsigned int off_enc_ssi1_value;



static unsigned int off_ana_in4_value;
static unsigned int off_ana_in5_value;
static unsigned int off_ana_in6_value;
static unsigned int off_ana_in7_value;
static unsigned int off_ana_out4     ;
static unsigned int off_ana_out5     ;
static unsigned int off_ana_out6     ;
static unsigned int off_ana_out7     ;
static unsigned int off_enc_inc2_out_ctrl; 
static unsigned int off_enc_inc2_out_value;
static unsigned int off_enc_inc2_in_ctrl;  
static unsigned int off_enc_inc2_in_value; 
static unsigned int off_enc_inc2_in_latch; 
static unsigned int off_enc_ssi2_status;
static unsigned int off_enc_ssi2_value;


static const ec_pdo_entry_reg_t regs[] = {

    {AnaInpSlave01_Pos, Beckhoff_EL3104, 0x6000, 0x11, &off_ana_in0_value},
    {AnaInpSlave01_Pos, Beckhoff_EL3104, 0x6010, 0x11, &off_ana_in1_value},
    {AnaInpSlave01_Pos, Beckhoff_EL3104, 0x6020, 0x11, &off_ana_in2_value},
    {AnaInpSlave01_Pos, Beckhoff_EL3104, 0x6030, 0x11, &off_ana_in3_value},

    {AnaOutSlave01_Pos, Beckhoff_EL4134, 0x7000, 0x01, &off_ana_out0},
    {AnaOutSlave01_Pos, Beckhoff_EL4134, 0x7010, 0x01, &off_ana_out1},
    {AnaOutSlave01_Pos, Beckhoff_EL4134, 0x7020, 0x01, &off_ana_out2},
    {AnaOutSlave01_Pos, Beckhoff_EL4134, 0x7030, 0x01, &off_ana_out3},

    {EncIncSlave01_Pos, Beckhoff_EL5101, 0x7000, 0x01, &off_enc_inc1_out_ctrl},
    {EncIncSlave01_Pos, Beckhoff_EL5101, 0x7000, 0x02, &off_enc_inc1_out_value},
    {EncIncSlave01_Pos, Beckhoff_EL5101, 0x6000, 0x01, &off_enc_inc1_in_ctrl},
    {EncIncSlave01_Pos, Beckhoff_EL5101, 0x6000, 0x02, &off_enc_inc1_in_value},
    {EncIncSlave01_Pos, Beckhoff_EL5101, 0x6000, 0x03, &off_enc_inc1_in_latch},

    {EncSsiSlave01_Pos, Beckhoff_EL5001, 0x3101, 0x01, &off_enc_ssi1_status},
    {EncSsiSlave01_Pos, Beckhoff_EL5001, 0x3101, 0x02, &off_enc_ssi1_value},


    {AnaInpSlave02_Pos, Beckhoff_EL3104, 0x6000, 0x11, &off_ana_in4_value},
    {AnaInpSlave02_Pos, Beckhoff_EL3104, 0x6010, 0x11, &off_ana_in5_value},
    {AnaInpSlave02_Pos, Beckhoff_EL3104, 0x6020, 0x11, &off_ana_in6_value},
    {AnaInpSlave02_Pos, Beckhoff_EL3104, 0x6030, 0x11, &off_ana_in7_value},

    {AnaOutSlave02_Pos, Beckhoff_EL4134, 0x7000, 0x01, &off_ana_out4},
    {AnaOutSlave02_Pos, Beckhoff_EL4134, 0x7010, 0x01, &off_ana_out5},
    {AnaOutSlave02_Pos, Beckhoff_EL4134, 0x7020, 0x01, &off_ana_out6},
    {AnaOutSlave02_Pos, Beckhoff_EL4134, 0x7030, 0x01, &off_ana_out7},

    {EncIncSlave02_Pos, Beckhoff_EL5101, 0x7000, 0x01, &off_enc_inc2_out_ctrl},
    {EncIncSlave02_Pos, Beckhoff_EL5101, 0x7000, 0x02, &off_enc_inc2_out_value},
    {EncIncSlave02_Pos, Beckhoff_EL5101, 0x6000, 0x01, &off_enc_inc2_in_ctrl},
    {EncIncSlave02_Pos, Beckhoff_EL5101, 0x6000, 0x02, &off_enc_inc2_in_value},
    {EncIncSlave02_Pos, Beckhoff_EL5101, 0x6000, 0x03, &off_enc_inc2_in_latch},

    {EncSsiSlave01_Pos, Beckhoff_EL5001, 0x3101, 0x01, &off_enc_ssi2_status},
    {EncSsiSlave01_Pos, Beckhoff_EL5001, 0x3101, 0x02, &off_enc_ssi2_value},

    {}
};


ECat_master_beckhoff::ECat_master_beckhoff() : ECat_master(regs, 100) {

    name = "ecat_master";
    period.period = {0,1000}; 

    schedpolicy = SCHED_FIFO;
    priority = sched_get_priority_max(schedpolicy);
    stacksize = PTHREAD_STACK_MIN;
}

ECat_master_beckhoff::~ECat_master_beckhoff() {

    std::cout << "Count loop time : " << (uint64_t)count(loop_time) << std::endl;
    std::cout << "Mean loop time : " << (uint64_t)mean(loop_time) << std::endl;
    std::cout << "Min loop time  : " << min(loop_time) << std::endl;
    std::cout << "Max loop time  : " << max(loop_time) << std::endl;
    std::cout << "Var loop time  : " << variance(loop_time) << std::endl;
    std::cout << "Mean error loop time  : " << boost::accumulators::error_of<tag::mean>(loop_time) << std::endl;

}

void ECat_master_beckhoff::th_init(void *) { 

    init();
}

void ECat_master_beckhoff::th_loop(void *) { 

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


int ECat_master_beckhoff::config_PDOs(void) {

    printf("Configuring EL3104...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, AnaInpSlave01_Pos, Beckhoff_EL3104),
                               EC_END, slave_1_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring EL4134...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, AnaOutSlave01_Pos, Beckhoff_EL4134),
                               EC_END, slave_2_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring EL5101...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, EncIncSlave01_Pos, Beckhoff_EL5101),
                               EC_END, slave_3_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring EL5001...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, EncSsiSlave01_Pos, Beckhoff_EL5001),
                               EC_END, slave_4_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for bus coupler
    if ( ! ecrt_master_slave_config(master, BusCoupler01_Pos, Beckhoff_EK1100))
        return -1;


    printf("Configuring EL3104...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, AnaInpSlave02_Pos, Beckhoff_EL3104),
                               EC_END, slave_6_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring EL4134...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, AnaOutSlave02_Pos, Beckhoff_EL4134),
                               EC_END, slave_7_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring EL5101...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, EncIncSlave02_Pos, Beckhoff_EL5101),
                               EC_END, slave_8_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    printf("Configuring EL5001...\n");
    if (ecrt_slave_config_pdos(ecrt_master_slave_config(master, EncSsiSlave02_Pos, Beckhoff_EL5001),
                               EC_END, slave_9_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    // Create configuration for bus coupler
    if ( ! ecrt_master_slave_config(master, BusCoupler02_Pos, Beckhoff_EK1100))
        return -1;

    
    return 0;
}


void ECat_master_beckhoff::ec_hook_loop(void * arg) {

    static int cycle_counter = 0;
    static int blink;
    uint16_t ain0;

    cycle_counter++;

    if (!(cycle_counter % 1000)) {
        blink = !blink;
    }

    ain0 = EC_READ_U16(domain_pd + off_ana_in0_value);

    //EC_WRITE_U16(domain_pd + off_ana_out0, blink ? 0x0: 0x3fff); // 5V
    // 10 Hz +- 5V sin
    EC_WRITE_U16(domain_pd + off_ana_out0, (uint16_t)sine_wave(10,0x3fff,0)); // 5V
    EC_WRITE_U16(domain_pd + off_ana_out1, ain0); 


}



// generate with
// ethercat cstruct

/* Master 0, Slave 1, "EL3104"
 * Vendor ID:       0x00000002
 * Product code:    0x0c203052
 * Revision number: 0x00100000
 */

ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Underrange */
    {0x6000, 0x02, 1}, /* Overrange */
    {0x6000, 0x03, 2}, /* Limit 1 */
    {0x6000, 0x05, 2}, /* Limit 2 */
    {0x6000, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1800, 0x07, 1}, /* TxPDO State */
    {0x1800, 0x09, 1}, /* TxPDO Toggle */
    {0x6000, 0x11, 16}, /* Value */
    {0x6010, 0x01, 1}, /* Underrange */
    {0x6010, 0x02, 1}, /* Overrange */
    {0x6010, 0x03, 2}, /* Limit 1 */
    {0x6010, 0x05, 2}, /* Limit 2 */
    {0x6010, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1802, 0x07, 1}, /* TxPDO State */
    {0x1802, 0x09, 1}, /* TxPDO Toggle */
    {0x6010, 0x11, 16}, /* Value */
    {0x6020, 0x01, 1}, /* Underrange */
    {0x6020, 0x02, 1}, /* Overrange */
    {0x6020, 0x03, 2}, /* Limit 1 */
    {0x6020, 0x05, 2}, /* Limit 2 */
    {0x6020, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1804, 0x07, 1}, /* TxPDO State */
    {0x1804, 0x09, 1}, /* TxPDO Toggle */
    {0x6020, 0x11, 16}, /* Value */
    {0x6030, 0x01, 1}, /* Underrange */
    {0x6030, 0x02, 1}, /* Overrange */
    {0x6030, 0x03, 2}, /* Limit 1 */
    {0x6030, 0x05, 2}, /* Limit 2 */
    {0x6030, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1806, 0x07, 1}, /* TxPDO State */
    {0x1806, 0x09, 1}, /* TxPDO Toggle */
    {0x6030, 0x11, 16}, /* Value */
};

ec_pdo_info_t slave_1_pdos[] = {
    {0x1a00, 11, slave_1_pdo_entries + 0}, /* AI TxPDO-Map Standard Ch.1 */
    {0x1a02, 11, slave_1_pdo_entries + 11}, /* AI TxPDO-Map Standard Ch.2 */
    {0x1a04, 11, slave_1_pdo_entries + 22}, /* AI TxPDO-Map Standard Ch.3 */
    {0x1a06, 11, slave_1_pdo_entries + 33}, /* AI TxPDO-Map Standard Ch.4 */
};

ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_1_pdos + 0, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 2, "EL4134"
 * Vendor ID:       0x00000002
 * Product code:    0x10263052
 * Revision number: 0x03f90000
 */

ec_pdo_entry_info_t slave_2_pdo_entries[] = {
    {0x7000, 0x01, 16}, /* Analog output */
    {0x7010, 0x01, 16}, /* Analog output */
    {0x7020, 0x01, 16}, /* Analog output */
    {0x7030, 0x01, 16}, /* Analog output */
};

ec_pdo_info_t slave_2_pdos[] = {
    {0x1600, 1, slave_2_pdo_entries + 0}, /* AO RxPDO-Map OutputsCh.1 */
    {0x1601, 1, slave_2_pdo_entries + 1}, /* AO RxPDO-Map OutputsCh.2 */
    {0x1602, 1, slave_2_pdo_entries + 2}, /* AO RxPDO-Map OutputsCh.3 */
    {0x1603, 1, slave_2_pdo_entries + 3}, /* AO RxPDO-Map OutputsCh.4 */
};

ec_sync_info_t slave_2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_2_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 3, "EL5101"
 * Vendor ID:       0x00000002
 * Product code:    0x13ed3052
 * Revision number: 0x03fd0000
 */

ec_pdo_entry_info_t slave_3_pdo_entries[] = {
    {0x7000, 0x01, 8}, /* Ctrl */
    {0x7000, 0x02, 16}, /* Value */
    {0x6000, 0x01, 8}, /* Status */
    {0x6000, 0x02, 16}, /* Value */
    {0x6000, 0x03, 16}, /* Latch */
};

ec_pdo_info_t slave_3_pdos[] = {
    {0x1600, 2, slave_3_pdo_entries + 0}, /* RxPDO-Map Outputs */
    {0x1a00, 3, slave_3_pdo_entries + 2}, /* TxPDO-Map Inputs */
};

ec_sync_info_t slave_3_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_3_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_3_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 4, "EL5001"
 * Vendor ID:       0x00000002
 * Product code:    0x13893052
 * Revision number: 0x03fa0000
 */

ec_pdo_entry_info_t slave_4_pdo_entries[] = {
    {0x3101, 0x01, 8}, /* Status */
    {0x3101, 0x02, 32}, /* Value */
};

ec_pdo_info_t slave_4_pdos[] = {
    {0x1a00, 2, slave_4_pdo_entries + 0}, /* TxPDO 001 mapping */
};

ec_sync_info_t slave_4_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_4_pdos + 0, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 6, "EL3104"
 * Vendor ID:       0x00000002
 * Product code:    0x0c203052
 * Revision number: 0x00100000
 */

ec_pdo_entry_info_t slave_6_pdo_entries[] = {
    {0x6000, 0x01, 1}, /* Underrange */
    {0x6000, 0x02, 1}, /* Overrange */
    {0x6000, 0x03, 2}, /* Limit 1 */
    {0x6000, 0x05, 2}, /* Limit 2 */
    {0x6000, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1800, 0x07, 1}, /* TxPDO State */
    {0x1800, 0x09, 1}, /* TxPDO Toggle */
    {0x6000, 0x11, 16}, /* Value */
    {0x6010, 0x01, 1}, /* Underrange */
    {0x6010, 0x02, 1}, /* Overrange */
    {0x6010, 0x03, 2}, /* Limit 1 */
    {0x6010, 0x05, 2}, /* Limit 2 */
    {0x6010, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1802, 0x07, 1}, /* TxPDO State */
    {0x1802, 0x09, 1}, /* TxPDO Toggle */
    {0x6010, 0x11, 16}, /* Value */
    {0x6020, 0x01, 1}, /* Underrange */
    {0x6020, 0x02, 1}, /* Overrange */
    {0x6020, 0x03, 2}, /* Limit 1 */
    {0x6020, 0x05, 2}, /* Limit 2 */
    {0x6020, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1804, 0x07, 1}, /* TxPDO State */
    {0x1804, 0x09, 1}, /* TxPDO Toggle */
    {0x6020, 0x11, 16}, /* Value */
    {0x6030, 0x01, 1}, /* Underrange */
    {0x6030, 0x02, 1}, /* Overrange */
    {0x6030, 0x03, 2}, /* Limit 1 */
    {0x6030, 0x05, 2}, /* Limit 2 */
    {0x6030, 0x07, 1}, /* Error */
    {0x0000, 0x00, 1}, /* Gap */
    {0x0000, 0x00, 5}, /* Gap */
    {0x1c32, 0x20, 1}, /* Sync error */
    {0x1806, 0x07, 1}, /* TxPDO State */
    {0x1806, 0x09, 1}, /* TxPDO Toggle */
    {0x6030, 0x11, 16}, /* Value */
};

ec_pdo_info_t slave_6_pdos[] = {
    {0x1a00, 11, slave_6_pdo_entries + 0}, /* AI TxPDO-Map Standard Ch.1 */
    {0x1a02, 11, slave_6_pdo_entries + 11}, /* AI TxPDO-Map Standard Ch.2 */
    {0x1a04, 11, slave_6_pdo_entries + 22}, /* AI TxPDO-Map Standard Ch.3 */
    {0x1a06, 11, slave_6_pdo_entries + 33}, /* AI TxPDO-Map Standard Ch.4 */
};

ec_sync_info_t slave_6_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 4, slave_6_pdos + 0, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 7, "EL4134"
 * Vendor ID:       0x00000002
 * Product code:    0x10263052
 * Revision number: 0x03f90000
 */

ec_pdo_entry_info_t slave_7_pdo_entries[] = {
    {0x7000, 0x01, 16}, /* Analog output */
    {0x7010, 0x01, 16}, /* Analog output */
    {0x7020, 0x01, 16}, /* Analog output */
    {0x7030, 0x01, 16}, /* Analog output */
};

ec_pdo_info_t slave_7_pdos[] = {
    {0x1600, 1, slave_7_pdo_entries + 0}, /* AO RxPDO-Map OutputsCh.1 */
    {0x1601, 1, slave_7_pdo_entries + 1}, /* AO RxPDO-Map OutputsCh.2 */
    {0x1602, 1, slave_7_pdo_entries + 2}, /* AO RxPDO-Map OutputsCh.3 */
    {0x1603, 1, slave_7_pdo_entries + 3}, /* AO RxPDO-Map OutputsCh.4 */
};

ec_sync_info_t slave_7_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 4, slave_7_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 8, "EL5101"
 * Vendor ID:       0x00000002
 * Product code:    0x13ed3052
 * Revision number: 0x03fd0000
 */

ec_pdo_entry_info_t slave_8_pdo_entries[] = {
    {0x7000, 0x01, 8}, /* Ctrl */
    {0x7000, 0x02, 16}, /* Value */
    {0x6000, 0x01, 8}, /* Status */
    {0x6000, 0x02, 16}, /* Value */
    {0x6000, 0x03, 16}, /* Latch */
};

ec_pdo_info_t slave_8_pdos[] = {
    {0x1600, 2, slave_8_pdo_entries + 0}, /* RxPDO-Map Outputs */
    {0x1a00, 3, slave_8_pdo_entries + 2}, /* TxPDO-Map Inputs */
};

ec_sync_info_t slave_8_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_8_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_8_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 9, "EL5001"
 * Vendor ID:       0x00000002
 * Product code:    0x13893052
 * Revision number: 0x03fa0000
 */

ec_pdo_entry_info_t slave_9_pdo_entries[] = {
    {0x3101, 0x01, 8}, /* Status */
    {0x3101, 0x02, 32}, /* Value */
};

ec_pdo_info_t slave_9_pdos[] = {
    {0x1a00, 2, slave_9_pdo_entries + 0}, /* TxPDO 001 mapping */
};

ec_sync_info_t slave_9_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_9_pdos + 0, EC_WD_DISABLE},
    {0xff}
};




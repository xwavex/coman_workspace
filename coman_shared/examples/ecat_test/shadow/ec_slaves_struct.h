

ec_pdo_entry_info_t slave_shadow_pdo_entries[] = {
    {0x1000, 0x01, 22*8}, /* command : used to set OUTPUT */
    {0x1016, 0x01, 10*8}, /* status : used to read INPUT */
};

ec_pdo_info_t slave_shadow_pdos[] = {
    {0x1600, 1, slave_shadow_pdo_entries + 0}, /* RxPDO map Outputs */
    {0x1a00, 1, slave_shadow_pdo_entries + 1}, /* TxPDO map Inputs */
};

ec_sync_info_t slave_shadow_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_shadow_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 1, slave_shadow_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


const char *exception_names[] =
{
    "IllegalInstructionCause",     /* 0 */
    "SyscallCause",                /* 1 */
    "InstructionFetchErrorCause",  /* 2 */
    "LoadStoreErrorCause",         /* 3 */
    "Level1InterruptCause",        /* 4 */
    "AllocaCause",                 /* 5 */
    "IntegerDivideByZeroCause",    /* 6 */
    "",                            /* 7 - reserved */
    "PrivilegedCause",             /* 8 */
    "LoadStoreAlignmentCause",     /* 9 */
    "",                            /* 10 - reserved */
    "",                            /* 11 - reserved */
    "InstrPIFDataErrorCause",      /* 12 */
    "LoadStorePIFDataErrorCause",  /* 13 */
    "InstrPIFAddrErrorCause",      /* 14 */
    "LoadStorePIFAddrErrorCause",  /* 15 */
    "InstTLBMissCause",            /* 16 */
    "InstTLBMultiHitCause",        /* 17 */
    "InstFetchPrivilegeCause",     /* 18 */
    "",                            /* 19 - reserved */
    "InstFetchProhibitedCause",    /* 20 */
    "",                            /* 21 - reserved */
    "",                            /* 22 - reserved */
    "",                            /* 23 - reserved */
    "LoadStoreTLBMissCause",       /* 24 */
    "LoadStoreTLBMultiHitCause",   /* 25 */
    "LoadStorePrivilegeCause",     /* 26 */
    "",                            /* 27 - reserved */
    "LoadProhibitedCause",         /* 28 */
    "StoreProhibitedCause",        /* 29 */
    "",                            /* 30 - reserved */
    "",                            /* 31 - reserved */
    "Coprocessor0Disabled",        /* 32 */
    "Coprocessor1Disabled",        /* 33 */
};

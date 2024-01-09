

#ifndef STORAGE_H_
#define STORAGE_H_

#include <stdint.h>
#include <stdbool.h>
#include "datatypes.h"

// Global variables
config_data m_config;

// Functions
void storage_init(void);
void storage_save_config(void);

#endif /* STORAGE_H_ */

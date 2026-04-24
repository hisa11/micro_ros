import os
import re

def process_file(filepath):
    with open(filepath, 'r') as f:
        content = f.read()

    # mbed.h replacement
    content = content.replace('#include "mbed.h"', '#include "main.h"\n#include "can.h"\n#include "can_utils.hpp"\n#include <algorithm>')
    content = content.replace('#include <mbed.h>', '#include "main.h"\n#include "can.h"\n#include "can_utils.hpp"\n#include <algorithm>')
    
    # CAN class replacement
    content = content.replace('CAN &can', 'CAN_HandleTypeDef *can')
    content = content.replace('CAN &can_ref', 'CAN_HandleTypeDef *can_ref')
    content = content.replace('can(can)', 'can(can)')
    content = content.replace('can(can_ref)', 'can(can_ref)')
    
    # wait_us and ThisThread::sleep_for replacement
    content = content.replace('wait_us(500);', 'HAL_Delay(1);')
    content = content.replace('ThisThread::sleep_for(1ms);', 'HAL_Delay(1);')

    # CAN write replacement
    content = re.sub(r'can\.write\((.*?)\)', r'can_write(can, \1)', content)
    
    # CAN read replacement
    # In VESC.hpp: can.read(rxMsg) -> read is tricky since in STM32 usually we read in interrupt.
    # We will simulate a read function for VESC, or just let users know CAN read needs interrupt setup.
    # Actually, can we just replace can.read with a pop from a software queue or HAL_CAN_GetRxMessage?
    # Let's write a simple can_read wrapper in can_utils.hpp if it's called.
    content = content.replace('can.read(', 'can_read(can, ')

    # can.reset()
    content = content.replace('can.reset()', '/* HAL_CAN_Reset? Not typical, skip or reinteg */')

    with open(filepath, 'w') as f:
        f.write(content)

for root, dirs, files in os.walk('UserLibs'):
    for file in files:
        if file.endswith(('.cpp', '.hpp')) and file != 'can_utils.hpp':
            process_file(os.path.join(root, file))


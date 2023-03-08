import os
import logging


def initSystem():
    # set the time
    os.system('hwclock -s -f /dev/rtc1')

    # set wifi
    os.system('nmcli r wifi on')
    os.system(
        'nmcli dev wifi connect TECNOPLUG password vene2522F ifname wlan0')

    logging.info('System initialized')

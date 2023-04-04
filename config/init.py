import os
import logging


def initSystem():
    # set the time
    os.system('hwclock -s -f /dev/rtc1')

    os.system('killall haveged')

    # set wifi
    # os.system('nmcli r wifi on')
    # os.system('nmcli dev wifi connect GiroVision password tecnoplug08 ifname wlan0')
    # os.system('nmcli dev wifi connect TECNOPLUG password vene2522 ifname wlan0')
    # create_ap wlan0 lo MyAccessPoint MyPassPhrase
    # os.system('iwconfig wlan0 power off')

    logging.info('System initialized')

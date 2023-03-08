# read te time
hwclock -r -f /dev/rtc1

# set the time from RTC
hwclock -s -f /dev/rtc1

# set the time to RTC
hwclock -w -f /dev/rtc1

# set the wifi
nmcli r wifi on
nmcli dev wifi connect CTNET-FERNANDEZ password vene2522 ifname wlan0

# get wifi list
nmcli dev wifi
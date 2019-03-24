from smbus import SMBus

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
bus.write_byte(addr, 0x1) # switch it on
input("Press return to exit")
bus.write_byte(addr, 0x0) # switch it on
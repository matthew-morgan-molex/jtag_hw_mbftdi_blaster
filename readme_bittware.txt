1) install pyftdi

$ pip install pyftdi

2) Program the FTDI EEPROM
Use the tool eeprom/ftdi_config.py to program the FTDI EEPROM

- First show the available interfaces. For example:

$ python eeprom/ftdi_config.py
Available interfaces:
  ftdi://ftdi:4232:5:2/1  (Quad RS232-HS)
  ftdi://ftdi:4232:5:2/2  (Quad RS232-HS)
  ftdi://ftdi:4232:5:2/3  (Quad RS232-HS)
  ftdi://ftdi:4232:5:2/4  (Quad RS232-HS)

- Next, program the eeprom.  Use the serial number of your card and
  the first interface of the listed interfaces found
$ python eeprom/ftdi_config.py --card-type IA-860M --serial 12345 ftdi://ftdi:4232:5:2/1

3) copy udev file

$ sudo cp 99-bw-blaster.rules /etc/udev/rules.d

4) build the Quartus Blaster lib

$ cd jtag_hw_mbftdi_blaster_src
$ make
$ cp libjtag_hw_mbftdi-blaster.so /quartus/linux64/

NOTE: /quaruts/linux64/ is an example. Copy the file to the linux64
dir in your quartus install area.

- After copying make the .so permissions the same as all the other
  libjtag*.so library files.

$ chmod 755 /quartus/linux64/libjtag_hw_mbftdi-blaster.so

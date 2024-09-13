1) install pyftdi
   $ pip install pyftdi
2) If not running as root, add user to the `dialout` group.  This is
   to allow access to the FTDI device
   $ usermod -a -G dialout USERNAME

3) Program the FTDI EEPROM
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

  NOTE: After programming the EEPROM you may have to power cycle the
        card for the EEPROM setting to take effect.

4) copy udev file

  $ sudo cp 99-bw-blaster.rules /etc/udev/rules.d

5) build the Quartus Blaster lib

  $ cd jtag_hw_mbftdi_blaster_src
  $ make
  $ cp libjtag_hw_mbftdi-blaster.so /quartus/linux64/

  NOTE: /quaruts/linux64/ is an example. Copy the file to the linux64
        dir in your quartus install area.

  - After copying make the .so permissions the same as all the other
    libjtag*.so library files.

  $ chmod 755 /quartus/linux64/libjtag_hw_mbftdi-blaster.so

6) verify operation

  $ jtagconfig
  1) MBFTDI-Blaster v2.1b (64) [MBUSB-0]        
    6BA00477   S10HPS/AGILEX_HPS/N5X_HPS
    434BD0DD   AGIB023R18A(.|B|R0)
 
  2) MBFTDI-Blaster v2.1b (64) [MBUSB-1]
    434BD0DD   AGIB023R18A(.|B|R0)
 
  3) MBFTDI-Blaster v2.1b (64) [MBUSB-2]
    434BD0DD   AGIB023R18A(.|B|R0)
 
  4) MBFTDI-Blaster v2.1b (64) [MBUSB-3]
    434BD0DD   AGIB023R18A(.|B|R0)

  - To see the serial number of a particular cable

  $ jtagconfig --getparam <Cable Number> SerialNumber

  Where Cable Number is 1,2,3 etc.

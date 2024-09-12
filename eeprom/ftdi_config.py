import argparse
import sys
import hexdump

from pyftdi.eeprom import FtdiEeprom
from pyftdi.usbtools import UsbTools

def handle_program(arguments):
    """handle_program
    Program FTDI EEPROM

    Args:
        arguments: arguments from the argparser
    """

    eeprom = FtdiEeprom()

    # Select the FTDI device to access
    eeprom.open(arguments.device)
    eeprom.initialize()
    eeprom.set_product_name(arguments.card_type)
    eeprom.set_manufacturer_name('BittWare')

    eeprom.set_serial_number('BW' + arguments.serial)
    eeprom.set_property('channel_a_driver', 'D2XX')
    eeprom.set_property('channel_a_type', 'UART')
    eeprom.set_property('self_powered', 'true')
    # Commit the change to the EEPROM
    eeprom.commit(dry_run=False)
    eeprom.dump_config()
    print(eeprom.data[8])
    hexdump.hexdump(eeprom.data)

def dump_devices():
    devdescs = UsbTools.list_devices('ftdi:///?',
                                     {'ftdi': 0x0403},
                                     { 0x0403: {'4232': 0x6011, '4232h': 0x6011}},
                                     0x0403)
    #print(devdescs)
    UsbTools.show_devices('ftdi', {'ftdi': 0x0403},
                          { 0x0403: {'4232': 0x6011, '4232h': 0x6011}},
                          devdescs)

def main():
    """main
    main function
    """
    program_description = "BittWare FTDI EEPROM programmer utility\n"

    parser = argparse.ArgumentParser(description=program_description,
                                     formatter_class=argparse.RawTextHelpFormatter)

    parser.add_argument("--debug", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument('--card-type', help=f'Card type', required=True)
    parser.add_argument('--serial', help='Serial Number as decimal value.', required=True)
    parser.add_argument( 'device', help=f'FTDI Device')

    if len(sys.argv) > 1:
        args = parser.parse_args()
        handle_program(args)
    else:
        dump_devices()
        
if __name__ == "__main__":
    main()

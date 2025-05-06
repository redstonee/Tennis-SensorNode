from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from argparse import ArgumentParser


parser = ArgumentParser(description="Patch and flash the Tennis-Dongle firmware.")
parser.add_argument(
    "--netid", type=int, required=True, help="The netID to patch. (0-80)"
)
parser.add_argument(
    "--sensorid", type=int, required=True, help="The sensorID to patch. (0-15)"
)
from subprocess import Popen, STDOUT

parser.add_argument(
    "--file",
    action="store",
    type=str,
    default=".pio/build/genericSTM32G030C8/firmware.elf",
    help="The path to the ELF file to patch. (default: .pio/build/genericSTM32G030C8/firmware.elf)",
)
args = parser.parse_args()

DATA_SECTION_NAME = ".rodata"
SENSOR_ID_NAME = "sensorID"
NET_ID_NAME = "netID"


def fuckOff(code):
    input("Press Enter to continue...")
    exit(code)


if args.netid < 0 or args.netid > 80:
    print("netID must be between 0 and 80")
    fuckOff(1)

if args.sensorid < 0 or args.sensorid > 15:
    print("sensorID must be between 0 and 15")
    fuckOff(1)


try:
    with open(args.file, "r+b") as fuckingFile:
        elf = ELFFile(fuckingFile)
        dataSection = elf.get_section_by_name(DATA_SECTION_NAME)
        symtabSection = elf.get_section_by_name(".symtab")
        if dataSection is None:
            print("Section {} not found".format(DATA_SECTION_NAME))
            fuckOff(1)
        if symtabSection is None:
            print("Symbol table section not found")
            fuckOff(1)

        sectionAddr = dataSection["sh_addr"]
        sectionOffset = dataSection["sh_offset"]

        print("Section {} found".format(DATA_SECTION_NAME))
        print("Section size: {}".format(dataSection.data_size))
        print("Section address: {}".format(hex(sectionAddr)))
        print()

        netID_Found = False
        sensorID_Found = False
        netID_Offset = 0
        sensorID_Offset = 0

        for sym in symtabSection.iter_symbols():
            if netID_Found and sensorID_Found:
                break
            if sym.name == SENSOR_ID_NAME:
                print("Symbol {} found".format(SENSOR_ID_NAME))
                addr = sym["st_value"]
                sensorID_Offset = addr + sectionOffset - sectionAddr
                sensorID_Found = True
                continue

            if sym.name == NET_ID_NAME:
                print("Symbol {} found".format(NET_ID_NAME))
                addr = sym["st_value"]
                netID_Offset = addr + sectionOffset - sectionAddr
                netID_Found = True
                continue

        if not netID_Found:
            print("Symbol {} not found".format(NET_ID_NAME))
            fuckOff(1)
        if not sensorID_Found:
            print("Symbol {} not found".format(SENSOR_ID_NAME))
            fuckOff(1)

        fuckingFile.seek(netID_Offset)
        fuckingFile.write(args.netid.to_bytes(1, byteorder="little"))
        print("netID patched to {}".format(args.netid))
        fuckingFile.seek(sensorID_Offset)
        fuckingFile.write(args.sensorid.to_bytes(1, byteorder="little"))
        print("sensorID patched to {}".format(args.sensorid))

        fuckingFile.close()


except Exception as e:
    print("Failed to open the ELF file:", e)
    fuckOff(1)

try:
    with ConnectHelper.session_with_chosen_probe(
        options={"target_override": "stm32g030c8tx"}
    ) as session:
        FileProgrammer(session).program(args.file)
        session.target.reset()

except Exception as e:
    print("Failed to flash the firmware:", e)
    fuckOff(1)

fuckOff(0)

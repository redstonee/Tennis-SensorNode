from elftools.elf.elffile import ELFFile
from subprocess import Popen, STDOUT

FILENAME = ".pio/build/genericSTM32G030C8/firmware.elf"
DATA_SECTION_NAME = ".rodata"
SENSOR_ID_NAME = "sensorID"
NET_ID_NAME = "netID"


def fuckOff(code):
    input("Press Enter to continue...")
    exit(code)


netID = 0
sensorID = 0
try:
    netID = int(input("Enter the netID: "))
    if netID < 0 or netID > 80:
        print("netID must be between 0 and 80")
        fuckOff(1)

    sensorID = int(input("Enter the sensorID: "))
    if sensorID < 0 or sensorID > 16:
        print("sensorID must be between 0 and 16")
        fuckOff(1)

except ValueError:
    print("Invalid input. Please enter a valid integer.")
    fuckOff(1)

print()

try:
    with open(FILENAME, "r+b") as fuckingFile:
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
        fuckingFile.write(netID.to_bytes(1, byteorder="little"))
        print("netID patched to {}".format(netID))
        fuckingFile.seek(sensorID_Offset)
        fuckingFile.write(sensorID.to_bytes(1, byteorder="little"))
        print("sensorID patched to {}".format(sensorID))

        fuckingFile.close()


except Exception as e:
    print("Failed to open the ELF file:", e)
    fuckOff(1)

try:
    flashCommand = [
        "openocd",
        "-f",
        "interface/cmsis-dap.cfg",
        "-f",
        "target/stm32g0x.cfg",
        "-c",
        "program {} verify reset exit".format(FILENAME),
    ]

    flashProcess = Popen(flashCommand, stderr=STDOUT)
    flashProcess.wait()
    if flashProcess.returncode != 0:
        print("Failed to flash the firmware")
    else:
        print("Firmware flashed successfully")

except Exception as e:
    print("Failed to flash the firmware:", e)
    fuckOff(1)

fuckOff(0)

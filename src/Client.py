import networktables as nt, time
from networktables import NetworkTables
from colorama import Fore, Back, Style
import pygame as pg, os

os.system('cls')

SERVER = "10.29.90.1"

NetworkTables.initialize(SERVER)

IS_CONNECTED = False

def connectionListener(connected, info):
    global IS_CONNECTED
    print(Fore.GREEN + f"Connected to {SERVER}")
    IS_CONNECTED = True

NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

tables = {
    "maps": NetworkTables.getTable("Maps"),
}

loading_symbols = ['|', '/', '—', '\\']
index = 0

while True:
    if IS_CONNECTED:
        print("Arm Position: ", str(tables["maps"].getNumber("Arm Position", 0)) + "r")
        time.sleep(0.5)
    else:
        os.system('cls')
        print(Fore.YELLOW + f"Connecting to {SERVER} " + loading_symbols[index % 4])
        time.sleep(0.2)
        index += 1
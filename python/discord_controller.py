import pyautogui
import serial
import argparse
import time
import logging
from ctypes import POINTER, cast
from comtypes import CLSCTX_ALL
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume

devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))
volume.SetMasterVolumeLevel(-10, None)

class MyControllerMap:
    def __init__(self):
        self.button = {'A': 'L', # Mute
                       'B': 'A', # Deafen
                       'C': 'B', # decline call
                       'D': 'C', # ansewer call
                       'E': 'D' # Trocar de aba
                       }

#pyautogui.hotkey('ctrl', 'shift', 'M')

class SerialControllerInterface:
    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.incoming = '0'
        pyautogui.PAUSE = 0  ## remove delay
    
    def update(self):
        ## Sync protocol
        print("update")
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))
            print("lendo")

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))

        if data == b'1': # Mute
            logging.info("Sending press")
            pyautogui.hotkey('ctrl', 'shift', 'M')

        elif data == b'2': # Deafen
            logging.info("Sending press")
            pyautogui.hotkey('ctrl', 'shift', 'D')

        elif data == b'3': # decline call
            logging.info("Sending press")
            pyautogui.hotkey('ctrl', 'shift', 'N')

        elif data == b'4':
            logging.info("Sending press") # ansewer call
            pyautogui.hotkey('ctrl', 'enter')

        elif data == b'5':
            logging.info("Sending press")
            pyautogui.keyDown(self.mapping.button['E']) # Não faz nada e nem existe o botão E na interface do discord

        elif data == b'6':
            logging.info("Sending press")
            # aumentar volume
            self.currentVolumeDb = volume.GetMasterVolumeLevel()
            print(self.currentVolumeDb)
            if self.currentVolumeDb >= -5:
                self.currentVolumeDb = -6
            volume.SetMasterVolumeLevel(self.currentVolumeDb + 5, None)

        elif data == b'7':
            logging.info("Sending press")
            # diminuir volume
            self.currentVolumeDb = volume.GetMasterVolumeLevel()
            print(self.currentVolumeDb)
            if self.currentVolumeDb <= -45:
                self.currentVolumeDb = -44
            volume.SetMasterVolumeLevel(self.currentVolumeDb - 5, None)
        elif data == b'0':
            logging.info("Sending release")

        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()

    def update(self):
        pyautogui.keyDown(self.mapping.button['A'])
        time.sleep(0.1)
        pyautogui.keyUp(self.mapping.button['A'])
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)

if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=115200)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()

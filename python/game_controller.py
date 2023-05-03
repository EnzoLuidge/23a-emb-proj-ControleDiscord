from ctypes import POINTER, cast
from comtypes import CLSCTX_ALL
import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas
import pyautogui
from pycaw.pycaw import AudioUtilities, IAudioEndpointVolume

devices = AudioUtilities.GetSpeakers()
interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
volume = cast(interface, POINTER(IAudioEndpointVolume))

class MyControllerMap:
    def __init__(self):
        self.button = {'A': pyautogui.hotkey('ctrl', 'shift', 'M'), # Mute
                       'B': pyautogui.hotkey('ctrl', 'shift', 'D'), # Deafen
                       'C': pyautogui.hotkey('escape'), # decline call
                       'D': pyautogui.hotkey('ctrl', 'enter'), # ansewer call
                       'E': pyautogui.hotkey('alt', 'tab') # Trocar de aba
                       }


class SerialControllerInterface:

    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)
        self.incoming = '0'
        self.devices = AudioUtilities.GetSpeakers()
        self.interface = devices.Activate(IAudioEndpointVolume._iid_, CLSCTX_ALL, None)
        self.volume = cast(interface, POINTER(IAudioEndpointVolume))
        self.currentVolumeDb = self.volume.GetMasterVolumeLevel()

    def update(self):
        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))

        if data == b'1':
            logging.info("Sending press")
            pyautogui.keyDown(self.mapping.button['A'])

        elif data == b'2':
            logging.info("Sending press")
            pyautogui.keyDown(self.mapping.button['B'])

        elif data == b'3':
            logging.info("Sending press")
            pyautogui.keyDown(self.mapping.button['C'])

        elif data == b'4':
            logging.info("Sending press")
            pyautogui.keyDown(self.mapping.button['D'])

        elif data == b'5':
            logging.info("Sending press")
            pyautogui.keyDown(self.mapping.button['E'])

        elif data == b'6':
            logging.info("Sending press")
            # aumentar volume
            self.currentVolumeDb = self.volume.GetMasterVolumeLevel()
            self.volume.SetMasterVolumeLevel(self.currentVolumeDb + 1.0, None)

        elif data == b'7':
            logging.info("Sending press")
            # diminuir volume
            self.currentVolumeDb = self.volume.GetMasterVolumeLevel()
            self.volume.SetMasterVolumeLevel(self.currentVolumeDb - 1.0, None)

        elif data == b'0':
            logging.info("Sending release")
            pyautogui.keyUp(self.mapping.button['A'])
            pyautogui.keyUp(self.mapping.button['B'])
            pyautogui.keyUp(self.mapping.button['C'])
            pyautogui.keyUp(self.mapping.button['D'])
            pyautogui.keyUp(self.mapping.button['E'])
            pyautogui.keyUp(self.mapping.button['F'])
            pyautogui.keyUp(self.mapping.button['G'])




        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)

    def update(self):
        self.j.set_button(self.mapping.button['A'], 1)
        time.sleep(0.1)
        self.j.set_button(self.mapping.button['A'], 0)
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
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

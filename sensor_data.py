#!/usr/bin/env python3

import os
import board
import serial
import adafruit_gps
import pickle
import asyncio
from typing import List
from subprocess import Popen
from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from pygeodesy.latlonBase import LatLonBase

KNOTS_MPH_CONVERSION = 1.150779
#SNAP_DATA = os.getenv('SNAP_DATA')
SNAP_DATA = "."

class SensorReads:
    def __init__(self):
        self.data_file = "{}/data_write.txt".format(SNAP_DATA)
        self.ffmpeg_file = "{}/data_read.txt".format(SNAP_DATA)
        self.sensors_file = "{}/sensors.txt".format(SNAP_DATA)
        self.twitch_id_file = "{}/twitch-id.txt".format(SNAP_DATA)
        self.sensors = {}
        self.sensor_data = {
            "speed": 0.0,
            "distance": 0.0,
        }
        self.heart_rate = None
        self.power = None

        # set up the gps
        uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1)
        self.gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
        self.gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        self.gps.send_command(b"PMTK220,1000")

        # get initial values of latitute, longitute, and altitude
        self.gps.update()
        if self.gps.has_fix:
            self.OldLatLonInfo = LatLonBase(self.gps.latitude, self.gps.longitude, height=self.gps.altitude_m)
            self.sensor_data["distance"] = 0

        # connect to sensors and run the main loop
        self.loop = asyncio.get_event_loop()
        asyncio.gather(
            self.sensor_load(),
            self.run()
        )
        self.loop.run_forever()

    # sensor_load is the entry point for connecting to the BLE sensors
    # 1. load the sensors file from disk
    # 2. convert the sensors to a list of BLEDevice class instances
    # 3. subscribe to the notifications from the sensors
    async def sensor_load(self):
        try:
            with open(self.sensors_file, "rb") as sensor_file:
                self.sensors = pickle.load(sensor_file)
        except (FileNotFoundError, TypeError):
            # if the file doesn't exist, just keep going without sensor data
            pass

        addresses = [sensor["address"] for sensor in self.sensors]
        characteristic_uuids = [sensor["characteristic_uuid"] for sensor in self.sensors]
        notification_handlers = [
            self.get_hr if sensor["name"] == "heart_rate" else self.get_power
            for sensor in self.sensors
        ]
        await self.scan_for_devices(addresses)
        asyncio.gather(
            *(
                self.connect_to_device(device,
                    char_uuid,
                    notif
                )
                for device, char_uuid, notif in zip(self.devices,
                    characteristic_uuids,
                    notification_handlers
                )
            )
        )
    
    # in order to subscribe to notifications from multiple devices in parallel,
    # bleak needs them to be in the form of BLEDevice class instances
    async def scan_for_devices(self, addresses: List[str]) -> List[BLEDevice]:
        addresses = [a.lower() for a in addresses]
        s = BleakScanner()
        print("Detecting devices...")
        self.devices = [await s.find_device_by_address(address) for address in addresses]
        for d in self.devices:
            if d:
                print(f"Detected {d}...")
        if None in self.devices:
            # We did not find all desired devices...
            undetected_devices = list(
                set(addresses).difference(
                    list(
                        filter(
                            lambda x: x in [d.address.lower() for d in self.devices if d],
                            addresses,
                        )
                    )
                )
            )
            raise ValueError(
                f"Could not find the following device(s): {undetected_devices}..."
            )
    
    # subscribe to notifications from a device and sleep indefinitely
    async def connect_to_device(self, address: BLEDevice, notification_uuid: str, callback):
        print(f"Starting {address} loop...")
        async with BleakClient(address, timeout=20.0) as client:
            print(f"Connected to {address}...")
            try:
                await client.start_notify(notification_uuid, callback)
                while True:
                    # TODO: handle disconnects
                    await asyncio.sleep(1.0)
            except Exception as e:
                print(e)

    # format the data in a nice way for the video overlay
    def format_data(self):
        try:
            data = f'{self.sensor_data["distance"]:.1f} miles  {self.sensor_data["speed"]:.1f} m/h'
            if "heart_rate" in self.sensor_data:
                data = data + f'  Heart Rate: {self.sensor_data["heart_rate"]} bpm'
            if "power" in self.sensor_data:
                data = data + f'  Power: {self.sensor_data["power"]} watts'
            return data
        except Exception as e:
            print(e)

    # ble heart rate service is done via a notify/subscribe model
    # this function will serve as the notification handler and set the
    # heart rate in self.sensor_data
    def get_hr(self, sender, data):
        byte0 = data[0]
        hrv_uint8 = (byte0 & 1) == 0

        if hrv_uint8:
            hr = data[1]
        else:
            hr = (data[2] << 8) | data[1]
        self.sensor_data["heart_rate"] = str(hr)

    # ble cycling power service is done via a notify/subscribe model
    # this function will serve as the notification handler and set the
    # heart rate in self.sensor_data
    def get_power(self, sender, data):
        lsb = data[2]
        msb = data[3]
        power = (msb<<8) | lsb
        print("power: {} watts".format(power))
        self.sensor_data["power"] = power

    async def update_data(self):
        # First get the speed
        try:
            self.gps.update()
        except OSError:
            # this happens when the GPS is queried too often, ignore it
            pass

        self.get_mph()
        self.calculate_distance()

        with open(self.data_file, "w") as data_file:
            data = self.format_data()
            print(data)
            data_file.write(data)
        # atomically update the file
        os.rename(self.data_file, self.ffmpeg_file)

    # Calculate the distance travelled since the last gps update, and add it
    # to the total distance travelled
    def calculate_distance(self):
        if self.gps.has_fix:
            if not hasattr(self, "OldLatLonInfo"):
                self.OldLatLonInfo = LatLonBase(self.gps.latitude, self.gps.longitude, height=self.gps.altitude_m)
            else:
                self.LatLonInfo = LatLonBase(self.gps.latitude, self.gps.longitude, height=self.gps.altitude_m)
                self.sensor_data["distance"] += self.LatLonInfo.haversineTo(self.OldLatLonInfo)
                self.OldLatLonInfo = self.LatLonInfo

    # get the speed in mph if the GPS has a signal
    def get_mph(self):
        if self.gps.has_fix and self.gps.speed_knots is not None:
            self.sensor_data["speed"] = self.gps.speed_knots * KNOTS_MPH_CONVERSION
        else:
            self.sensor_data["speed"] = 0.0
    
    async def run(self):
        # create a default subtitle
        with open(self.data_file,"w") as data_file:
            data_file.write("Loading Sensor Data...")

        # load the user configured twitch stream ID
        with open(self.twitch_id_file, "rb") as twitch_file:
            twitch_id = twitch_file.readlines()
    
        ffmpeg = Popen(["bash", "twitch-stream", twitch_id])
        try:
            while True:
                await self.update_data()
                 make sure ffmpeg is still running
                if ffmpeg.poll() is not None:
                    raise Exception("ffmpeg has stopped running")
                await asyncio.sleep(0.5)
        except Exception as e:
            print(e)


if __name__ == "__main__":
    SensorReads()

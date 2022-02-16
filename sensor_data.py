#!/usr/bin/env python3

import os
import board
import serial
import adafruit_gps
import pickle
import asyncio
import threading
from subprocess import Popen
from bleak import discover
from bleak import BleakClient
from pygeodesy.latlonBase import LatLonBase

KNOTS_MPH_CONVERSION = 1.150779
HEART_RATE_SERVICE_UUID = "0000180d"
POWER_SERVICE_UUID = "00001818"
HEART_RATE_CHARACTERISTIC_UUID = "00002a37-0000-1000-8000-00805f9b34fb"
POWER_CHARACTERISTIC_UUID = "00002a63-0000-1000-8000-00805f9b34fb"
#SNAP_DATA = os.getenv('SNAP_DATA')
SNAP_DATA = "."

file_mutex = threading.Lock()

class SensorReads:

    def __init__(self):
        self.loop = asyncio.get_event_loop()
        self.data_file = "{}/data_write.txt".format(SNAP_DATA)
        self.ffmpeg_file = "{}/data_read.txt".format(SNAP_DATA)
        self.sensors_file = "{}/sensors.txt".format(SNAP_DATA)
        self.speed = "No GPS Signal"
        self.sensors = {}
        self.sensor_data = {}
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

        # load the sensors
        asyncio.ensure_future(self.sensor_load())
        
        # run the main loop
        asyncio.ensure_future(self.run())

        loop = asyncio.get_event_loop()
        loop.run_forever()


    async def sensor_load(self):
        # first check if there is a MAC address saved in sensors.txt
        try:
            with open(self.sensors_file, "rb") as sensor_file:
                self.sensors = pickle.load(sensor_file)
            if "heart_rate" in self.sensors:
                self.hr_mac = self.sensors["heart_rate"]
                # subscribe to the hr service
                await self.subscribe_notify(self.hr_mac, HEART_RATE_CHARACTERISTIC_UUID, self.get_hr)
            else:
                # search for a heart rate monitor in a new thread
                await self.sensor_search(HEART_RATE_SERVICE_UUID, "heart_rate", HEART_RATE_CHARACTERISTIC_UUID, self.get_hr)
            if "power" in self.sensors:
                self.power_mac = self.sensors["power"]
                await self.subscribe_notify(self.hr_mac, POWER_CHARACTERISTIC_UUID, self.get_power)
            else:
                # search for a power meter in a new thread
                await self.sensor_search(POWER_SERVICE_UUID, "power", POWER_CHARACTERISTIC_UUID, self.get_power)
        except (FileNotFoundError, TypeError):
            # if the file doesn't exist, go ahead and search for all sensor types
            await self.sensor_search(HEART_RATE_SERVICE_UUID, "heart_rate", HEART_RATE_CHARACTERISTIC_UUID, self.get_hr)
            await self.sensor_search(POWER_SERVICE_UUID, "power", POWER_CHARACTERISTIC_UUID, self.get_power)

    # search for a sensor with the given svc uuid, and if found, subscribe to notifications
    # with the provided characteristic and notification handler
    async def sensor_search(self, svc_uuid, name, char_uuid, notification_handler):
        found = False
        while not found:
            devices = await discover()
            for d in devices:
                try:
                    async with BleakClient(str(d.address)) as client:
                        svcs = await client.get_services()
                        service_iterator = svcs.__iter__()
                        for service in service_iterator:
                            if service.uuid.lower().startswith(svc_uuid):
                                print("Found sensor {} with address {}".format(name, d.address))
                                found = True
                                self.sensors[name] = d.address
                                self.write_sensors_file()
                                break

                except Exception as e:
                    pass
        await self.subscribe_notify(self.sensors[name], char_uuid, notification_handler)

    def write_sensors_file(self):
        with file_mutex:
            with open(self.sensors_file, "wb") as sensor_file:
                pickle.dump(self.sensors, sensor_file, protocol=pickle.HIGHEST_PROTOCOL)

    def format_data(self):
        data = str(self.speed) + " MPH "
        data = str(self.sensor_data["distance"]) + " Miles"
        if "heart_rate" in self.sensor_data:
            data = data + " Heart Rate: " + self.sensor_data["heart_rate"] + " BPM"
        if "power" in self.sensor_data:
            data = data + " Power: " + self.sensor_data["power"] + " Watts"
        return data

    # ble heart rate service is done via a notify rather than a read.
    # this function will serve as the notification handler and set the
    # heart reate in self.sensor_data
    async def get_hr(self, sender, data):
        #TODO
        """Simple notification handler which prints the data received."""
        byte0 = data[0]
        hrv_uint8 = (byte0 & 1) == 0

        if hrv_uint8:
            hr = data[1]
        else:
            hr = (data[2] << 8) | data[1]
        self.sensor_data["heart_rate"] = str(hr)

    async def get_power(self, sender, data):
        #TODO
        self.sensor_data["power"] = "0"

    async def subscribe_notify(self, address, char_uuid, notification_handler):
        async with BleakClient(address) as client:
            print(f"Connected: {client.is_connected}")
            await client.start_notify(char_uuid, notification_handler)

    async def update_data(self):
        # First get the speed
        try:
            self.gps.update()
        except OSError:
            # this happens when the GPS is queried too often, ignore it
            pass

        self.get_mph()

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
            if not self.OldLatLonInfo:
                self.OldLatLonInfo = LatLongBase(self.gps.latitude, self.gps.longitude, height=self.gps.altitude_m)
            else:
                self.LatLonInfo = LatLongBase(self.gps.latitude, self.gps.longitude, height=self.gps.altitude_m)
                self.sensor_data["distance"] += self.LatLonInfo.haversineTo(self.OldLatLonInfo)
                self.OldLatLonInfo = self.LatLonInfo

    def get_mph(self):
        if self.gps.has_fix:
            self.speed = self.gps.speed_knots * KNOTS_MPH_CONVERSION
        else:
            self.speed = "No GPS Signal"
    
    async def run(self):
        # create a default subtitle
        with open(self.data_file,"w") as data_file:
            data_file.write("Loading Sensor Data...")
    
        #ffmpeg = Popen(["bash", "twitch-stream"])
        try:
            while True:
                await self.update_data()
                # make sure ffmpeg is still running
                #if ffmpeg.poll() is not None:
                #    raise Exception("ffmpeg has stopped running")
                await asyncio.sleep(0.5)
        finally:
            self.write_sensors_file()


if __name__ == "__main__":
    SensorReads()

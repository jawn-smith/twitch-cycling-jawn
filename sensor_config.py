#!/usr/bin/env python3

import os
import pickle
import asyncio
import argparse
from bleak import discover
from bleak import BleakClient

HEART_RATE_BLE = {
    "name": "heart_rate",
    "service_uuid": "0000180d",
    "characteristic_uuid": "00002a37-0000-1000-8000-00805f9b34fb",
}
POWER_BLE = {
    "name": "power",
    "service_uuid": "00001818",
    "characteristic_uuid": "00002a63-0000-1000-8000-00805f9b34fb",
}

SNAP_DATA = os.getenv('SNAP_DATA')

class SensorSetup:
    def __init__(self, args):
        self.loop = asyncio.get_event_loop()
        self.sensors_file = "{}/sensors.txt".format(SNAP_DATA)
        self.twitch_id_file = "{}/twitch-id.txt".format(SNAP_DATA)
        self.sensors = []
        self.sensors_needed = []
        self.twitch_id = args.twitch_id

        # if the address was given, simply add it to the dict
        if args.hr_mac is not None:
            HEART_RATE_BLE["address"] = args.hr_mac
            self.sensors.append(HEART_RATE_BLE)
        if args.power_mac is not None:
            POWER_BLE["address"] = args.power_mac
            self.sensors.append(POWER_BLE)

        # If the scan flags were provided, scan for the sensors
        if args.scan_hr:
            self.sensors_needed.append(HEART_RATE_BLE)
        if args.scan_power:
            self.sensors_needed.append(POWER_BLE)

        self.loop.run_until_complete(
            asyncio.gather(
                *(
                    self.sensor_search(sensor)
                    for sensor in self.sensors_needed
                )
            )
        )

        self.write_configs()

    # search for a sensor with the given svc uuid, and if found, subscribe to notifications
    # with the provided characteristic and notification handler
    async def sensor_search(self, sensor):
        found = False
        while not found:
            devices = await discover()
            for d in devices:
                if not found:
                    print("searching device {} for service {}".format(d.address, sensor["service_uuid"]))
                    try:
                        async with BleakClient(str(d.address)) as client:
                            svcs = await client.get_services()
                            service_iterator = svcs.__iter__()
                            for service in service_iterator:
                                if service.uuid.lower().startswith(sensor["service_uuid"]):
                                    print("Found sensor {} with address {}".format(sensor["name"], d.address))
                                    found = True
                                    sensor["address"] = d.address
                                    self.sensors.append(sensor)
                    except Exception as e:
                        print(e)
                        pass

    # write the config info out to disk for the daemon to load
    def write_configs(self):
        if self.sensors != []:
            with open(self.sensors_file, "wb") as sensor_file:
                pickle.dump(self.sensors, sensor_file, protocol=pickle.HIGHEST_PROTOCOL)
        if self.twitch_id is not None:
            with open(self.twitch_id_file, "wb") as twitch_file:
                pickle.dump(self.twitch_id, twitch_file, protocol=pickle.HIGHEST_PROTOCOL)


def main():
    parser = argparse.ArgumentParser(description='''Configure the sensors for use
        by the cycling twitch stream daemon.''')
    hr_group = parser.add_mutually_exclusive_group()
    power_group = parser.add_mutually_exclusive_group()
    hr_group.add_argument('--scan-hr', action='store_true',
        help='Scan for a device advertising the BLE heart rate profile')
    power_group.add_argument('--scan-power', action='store_true',
        help='Scan for a device advertising the BLE cycling power profile')
    hr_group.add_argument('--hr-mac', type=str,
        help='Set the MAC address for the heart rate monitor')
    power_group.add_argument('--power-mac', type=str,
        help='Set the MAC address for the power meter')
    parser.add_argument('--twitch-id', type=str,
        help='Set the Twitch Stream ID used to authenticate the account')
    args = parser.parse_args()
    SensorSetup(args)

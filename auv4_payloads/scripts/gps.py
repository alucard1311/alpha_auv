#!/usr/bin/env python3

from serial import Serial
from ublox_gps import UbloxGps

port = Serial("/dev/ttyTHS2", baudrate=38400, timeout=1)
gps = UbloxGps(port)

if __name__ == "__main__":
    coords = gps.geo_coords()
    print(coords.lon, coords.lat)
    port.close()

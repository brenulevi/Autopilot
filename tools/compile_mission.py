"""Compile latitude/longitude missions to APM3 (typed waypoints) or legacy APM2."""

import argparse
import json
import math
import struct
import zlib
from pathlib import Path


def finite_number(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{name} must be a finite number")
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
    return value


def compile_mission(source):
    if not isinstance(source, dict) or type(source.get("version")) is not int or source["version"] not in (2, 3):
        raise ValueError("Mission version must be 2 or 3")
    version = source["version"]
    waypoints = source.get("waypoints")
    if not isinstance(waypoints, list) or not 2 <= len(waypoints) <= 32:
        raise ValueError("Mission needs 2..32 waypoints")
    payload = bytearray(struct.pack("<4sHHI", b"APM3" if version == 3 else b"APM2", version, len(waypoints), 0))
    origin = None
    previous = None
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            raise ValueError(f"Waypoint {index} must be an object")
        if version == 2 and "type" in waypoint:
            raise ValueError("Waypoint types require mission version 3")
        kind = waypoint.get("type", "fly_by")
        if not isinstance(kind, str) or kind not in ("fly_by", "fly_over"):
            raise ValueError(f"Waypoint {index} type must be fly_by or fly_over")
        latitude = finite_number(waypoint.get("lat_deg"), f"waypoint {index} latitude", -90, 90)
        longitude = finite_number(waypoint.get("lon_deg"), f"waypoint {index} longitude", -180, 180)
        altitude = finite_number(waypoint.get("altitude_m"), f"waypoint {index} altitude", -500, 10000)
        speed = finite_number(waypoint.get("airspeed_m_s"), f"waypoint {index} airspeed", 5, 100)
        lat_e7 = round(latitude * 1e7)
        lon_e7 = round(longitude * 1e7)
        if origin is None:
            origin = (lat_e7, lon_e7)
        lon_delta_e7 = lon_e7 - origin[1]
        if lon_delta_e7 > 1800000000:
            lon_delta_e7 -= 3600000000
        if lon_delta_e7 < -1800000000:
            lon_delta_e7 += 3600000000
        scale = 6371000.0 * math.pi / 180.0 * 1e-7
        north = (lat_e7 - origin[0]) * scale
        east = lon_delta_e7 * scale * math.cos(origin[0] * 1e-7 * math.pi / 180.0)
        if abs(north) > 20000 or abs(east) > 20000:
            raise ValueError(f"Waypoint {index} is more than 20 km from the first waypoint")
        if previous is not None and math.hypot(north - previous[0], east - previous[1]) < 1:
            raise ValueError(f"Waypoint {index} is less than 1 m from its predecessor")
        previous = (north, east)
        payload.extend(struct.pack("<iiiHH", lat_e7, lon_e7,
                                   round(altitude * 100), round(speed * 100), int(kind == "fly_over")))
    payload.extend(struct.pack("<I", zlib.crc32(payload)))
    return bytes(payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="JSON mission source")
    parser.add_argument("output", type=Path, help="Compiled .apm file")
    args = parser.parse_args()
    try:
        data = compile_mission(json.loads(args.source.read_text(encoding="utf-8")))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_bytes(data)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.exit(1, f"compile_mission: {error}\n")
    print(f"Compiled {len(data)} bytes: {args.output.resolve()}")


if __name__ == "__main__":
    main()

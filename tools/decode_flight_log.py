"""Decode FLG1 flash dumps to CSV using only the Python standard library."""
from __future__ import annotations

import argparse
import csv
import mmap
from pathlib import Path
import struct
import sys
import zlib

HEADER = struct.Struct("<4sBBHHHIIQI")
SOURCES = {1: "h723", 2: "f405", 3: "sim"}
TYPES = {1: "boot", 2: "controls", 3: "state", 4: "io", 5: "event", 6: "clock_pair"}
SIZES = {1: 12, 2: 20, 3: 40, 4: 32, 5: 12, 6: 20}
STATE_FIELDS = ["roll_rad", "pitch_rad", "yaw_rad", "p_rad_s", "q_rad_s", "r_rad_s",
                "airspeed_m_s", "altitude_m", "climb_rate_m_s", "valid_fields"]
FIELDS = ["input_file", "offset", "source", "session_id", "sequence", "timestamp_us",
          "dropped_total", "type", "firmware_id", "config_id", "reset_reason",
          "aileron", "elevator", "rudder", "throttle", "stage", "valid", *STATE_FIELDS,
          "authority", "reason", "pulse_count", "armed", "rc_age_ms", "autopilot_age_ms",
          "saturated_mask", *[f"pulse_{i}_us" for i in range(8)], "code", "severity",
          "argument0", "argument1", "peer_source", "peer_session_id", "peer_timestamp_us",
          "uncertainty_us", "unknown_payload_hex"]


def payload_fields(kind: int, payload: bytes) -> dict:
    """Validate known v1 payloads; preserve unknown record types as hex."""
    if kind not in SIZES:
        return {"unknown_payload_hex": payload.hex()}
    if len(payload) != SIZES[kind]:
        raise ValueError("payload length")
    if kind == 1:
        return dict(zip(["firmware_id", "config_id", "reset_reason"], struct.unpack("<III", payload)))
    if kind == 2:
        a, e, r, t, stage, valid, reserved = struct.unpack("<4fBBH", payload)
        if stage not in (1, 2, 3) or valid > 1 or reserved:
            raise ValueError("control metadata")
        return dict(aileron=a, elevator=e, rudder=r, throttle=t,
                    stage={1: "computed", 2: "pilot", 3: "selected"}[stage], valid=valid)
    if kind == 3:
        values = struct.unpack("<9fI", payload)
        if values[-1] & ~0x1FF:
            raise ValueError("state validity mask")
        return dict(zip(STATE_FIELDS, values))
    if kind == 4:
        authority, reason, count, armed, rc_age, ap_age, saturated, *pulses = struct.unpack("<4B3I8H", payload)
        if authority > 3 or reason > 5 or count > 8 or armed > 1 or saturated >> count or any(pulses[count:]):
            raise ValueError("I/O metadata")
        fields = dict(authority=authority, reason=reason, pulse_count=count, armed=armed,
                      rc_age_ms=rc_age, autopilot_age_ms=ap_age, saturated_mask=saturated)
        fields.update({f"pulse_{i}_us": p for i, p in enumerate(pulses[:count])})
        return fields
    if kind == 5:
        code, severity, reserved, a0, a1 = struct.unpack("<HBBII", payload)
        if severity > 2 or reserved:
            raise ValueError("event metadata")
        return dict(code=code, severity=severity, argument0=a0, argument1=a1)
    peer, session, timestamp, uncertainty = struct.unpack("<B3xIQI", payload)
    if peer not in SOURCES or not session or any(payload[1:4]):
        raise ValueError("clock pair metadata")
    return dict(peer_source=SOURCES[peer], peer_session_id=session,
                peer_timestamp_us=timestamp, uncertainty_us=uncertainty)


def records(data, stats: dict):
    """Scan bytes/mmap, recovering after bad CRCs, erased gaps, and torn writes.

    Keeps source-local order; it does not synchronize or sort independent clocks.
    stats is populated even for dumps containing no complete records.
    """
    offset = 0
    stats.update(records=0, skipped_bytes=0, invalid_candidates=0)
    while offset < len(data):
        start = data.find(b"FLG1", offset)
        if start < 0:
            stats["skipped_bytes"] += len(data) - offset
            break
        stats["skipped_bytes"] += start - offset
        try:
            if len(data) - start < HEADER.size + 4:
                raise ValueError("torn header")
            _, version, source, kind, size, reserved, sequence, session, timestamp, dropped = HEADER.unpack_from(data, start)
            end = start + HEADER.size + size + 4
            if version != 1 or source not in SOURCES or not session or reserved or size > 64 or end > len(data):
                raise ValueError("header")
            expected, = struct.unpack_from("<I", data, end - 4)
            if zlib.crc32(data[start:end - 4]) != expected:
                raise ValueError("CRC")
            fields = payload_fields(kind, data[start + HEADER.size:end - 4])
        except ValueError:
            stats["invalid_candidates"] += 1
            stats["skipped_bytes"] += 1
            offset = start + 1
            continue
        yield dict(offset=start, source=SOURCES[source], session_id=session,
                   sequence=sequence, timestamp_us=timestamp, dropped_total=dropped,
                   type=TYPES.get(kind, f"unknown_{kind}"), **fields)
        stats["records"] += 1
        offset = end


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path, nargs="+", help="Raw flash dump(s) or extracted FLG1 streams")
    parser.add_argument("--output", type=Path, required=True, help="New CSV path; existing files are never overwritten")
    args = parser.parse_args(argv)
    total = 0
    try:
        # Check all inputs before creating an output file.
        for path in args.inputs:
            with path.open("rb"):
                pass
        with args.output.open("x", newline="", encoding="utf-8") as out:
            writer = csv.DictWriter(out, fieldnames=FIELDS)
            writer.writeheader()
            for path in args.inputs:
                stats = {}
                with path.open("rb") as stream:
                    if path.stat().st_size:
                        with mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
                            for row in records(data, stats):
                                writer.writerow(dict(input_file=str(path), **row))
                    else:
                        list(records(b"", stats))
                total += stats["records"]
                print(f"{path}: {stats['records']} records, {stats['skipped_bytes']} skipped bytes "
                      f"(including erased space), {stats['invalid_candidates']} invalid/torn candidates", file=sys.stderr)
    except OSError as error:
        print(f"Flight-log decoding failed: {error}", file=sys.stderr)
        return 1
    if not total:
        print("No valid records recovered.", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Protokół X4F-LOG (docs/PROTOCOL.md) – klient dla USB Serial i BLE (bleak).

Strumień Xiao → host:
  * BLE: każda notyfikacja zaczyna się bajtem typu: 0x01 tekst, 0x02 ramka binarna.
  * USB: linie tekstu zakończone '\\n'; ramka binarna poprzedzona bajtem 0x02.
Ramka binarna (odpowiedź GET): BlockHeader (magic, seq, offset, len, crc32) + dane.
"""
from __future__ import annotations

import asyncio
import json
import queue
import struct
import threading
import time
import zlib
from dataclasses import dataclass

FRAME_TEXT = 0x01
FRAME_BINARY = 0x02
BLOCK_MAGIC = 0x42463458
BLOCK_HDR = struct.Struct("<IIIII")
GET_BLOCK_MAX = 4096

NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # zapis (host → Xiao)
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notyfikacje (Xiao → host)


class ProtocolError(Exception):
    pass


class DeviceBusy(ProtocolError):
    pass


@dataclass
class Block:
    seq: int
    offset: int
    data: bytes


class _Demux:
    """Składa tekst i ramki binarne; wynik do kolejek."""

    def __init__(self):
        self.lines: "queue.Queue[str]" = queue.Queue()
        self.frames: "queue.Queue[bytes]" = queue.Queue()
        self.live: "queue.Queue[dict]" = queue.Queue(maxsize=1000)
        self._text = bytearray()
        self._bin = bytearray()

    def _line(self, line: str):
        if line.startswith('{"live"'):
            try:
                if self.live.full():
                    self.live.get_nowait()
                self.live.put_nowait(json.loads(line))
            except Exception:
                pass
            return
        self.lines.put(line)

    def text(self, data: bytes):
        self._text += data
        while b"\n" in self._text:
            line, _, rest = self._text.partition(b"\n")
            self._text = bytearray(rest)
            self._line(line.decode(errors="replace").strip())

    def binary(self, data: bytes):
        self._bin += data
        while len(self._bin) >= BLOCK_HDR.size:
            magic, seq, off, ln, crc = BLOCK_HDR.unpack_from(self._bin)
            if magic != BLOCK_MAGIC:
                self._bin.clear()
                return
            total = BLOCK_HDR.size + ln
            if len(self._bin) < total:
                return
            self.frames.put(bytes(self._bin[:total]))
            del self._bin[:total]

    def reset_binary(self):
        self._bin.clear()


def find_port() -> str | None:
    """Port USB Xiao (VID Seeed 0x2886); numer ttyACM zmienia się po resecie."""
    from serial.tools import list_ports

    for p in list_ports.comports():
        if p.vid == 0x2886:
            return p.device
    return None


class SerialLink:
    """USB CDC: tekst + 0x02 przed ramką binarną."""

    def __init__(self, port: str | None = None):
        import serial

        port = port or find_port()
        if not port:
            raise ProtocolError("Nie znaleziono Xiao na USB")
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.demux = _Demux()
        self._stop = False
        self._buf = bytearray()
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self):
        state_bin_need = 0
        while not self._stop:
            try:
                d = self.ser.read(8192)
            except Exception:
                break
            if not d:
                continue
            self._buf += d
            while self._buf:
                if state_bin_need:
                    take = min(state_bin_need, len(self._buf))
                    self.demux.binary(bytes(self._buf[:take]))
                    del self._buf[:take]
                    state_bin_need -= take
                    continue
                if self._buf[0] == FRAME_BINARY:
                    if len(self._buf) < 1 + BLOCK_HDR.size:
                        break
                    ln = BLOCK_HDR.unpack_from(self._buf, 1)[3]
                    del self._buf[:1]
                    self.demux.reset_binary()
                    state_bin_need = BLOCK_HDR.size + ln
                    continue
                nl = self._buf.find(b"\n")
                if nl < 0:
                    break
                self.demux.text(bytes(self._buf[: nl + 1]))
                del self._buf[: nl + 1]

    def write(self, data: bytes):
        self.ser.write(data)

    def close(self):
        self._stop = True
        self._t.join(timeout=1)
        self.ser.close()


class BleLink:
    """BLE NUS przez bleak (wątek z pętlą asyncio)."""

    def __init__(self, name_prefix: str = "X4F", address: str | None = None, timeout: float = 10):
        self.demux = _Demux()
        self._loop = asyncio.new_event_loop()
        self._t = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._t.start()
        self.client = None
        self.mtu = 23
        fut = asyncio.run_coroutine_threadsafe(self._connect(name_prefix, address, timeout), self._loop)
        fut.result(timeout + 15)

    async def _connect(self, prefix, address, timeout):
        from bleak import BleakClient, BleakScanner

        if address is None:
            dev = await BleakScanner.find_device_by_filter(
                lambda d, ad: (d.name or ad.local_name or "").startswith(prefix), timeout=timeout
            )
            if dev is None:
                raise ProtocolError(f"Nie znaleziono urządzenia {prefix}*")
            address = dev
        self.client = BleakClient(address)
        await self.client.connect()
        try:
            self.mtu = self.client.mtu_size
        except Exception:
            pass
        await self.client.start_notify(NUS_TX, self._on_notify)

    def _on_notify(self, _h, data: bytearray):
        if not data:
            return
        if data[0] == FRAME_TEXT:
            self.demux.text(bytes(data[1:]))
        elif data[0] == FRAME_BINARY:
            self.demux.binary(bytes(data[1:]))

    def write(self, data: bytes):
        async def w():
            chunk = max(20, self.mtu - 3)
            for i in range(0, len(data), chunk):
                await self.client.write_gatt_char(NUS_RX, data[i : i + chunk], response=False)

        asyncio.run_coroutine_threadsafe(w(), self._loop).result(10)

    def close(self):
        async def c():
            await self.client.disconnect()

        try:
            asyncio.run_coroutine_threadsafe(c(), self._loop).result(5)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)


class Device:
    def __init__(self, link):
        self.link = link

    def _drain(self):
        for q in (self.link.demux.lines, self.link.demux.frames):
            while not q.empty():
                q.get_nowait()

    def command(self, cmd: str, timeout: float = 3.0):
        """Wysyła komendę, zwraca odpowiedź (dict dla JSON, str dla OK...)."""
        self._drain()
        self.link.write((cmd + "\n").encode())
        try:
            line = self.link.demux.lines.get(timeout=timeout)
        except queue.Empty:
            raise ProtocolError(f"Brak odpowiedzi na {cmd!r}")
        if line.startswith("{"):
            obj = json.loads(line)
            if "err" in obj:
                if obj["err"] == "BUSY":
                    raise DeviceBusy(cmd)
                raise ProtocolError(f"{cmd}: {obj}")
            return obj
        return line

    def info(self):
        return self.command("INFO")

    def status(self):
        return self.command("STATUS")

    def list_runs(self):
        return self.command("LIST")["runs"]

    def set_time(self, unix_s: int | None = None):
        return self.command(f"TIME {int(unix_s if unix_s is not None else time.time())}")

    def drift_reset(self):
        """Zerowanie punktów driftu (protokół ≥ 3)."""
        return self.command("DRIFT RESET")

    def mount(self, imu_axes: dict | None = None, yaw_offset_deg: float = 0.0, steer_sign: int | None = None,
              name: str | None = None):
        """Montaż płytki (protokół ≥ 3; znak skrętu 1/−1/0 – ≥ 4; nazwa profilu i zapis na stałe – ≥ 5).
        Bez argumentów – odczyt."""
        if imu_axes is None:
            return self.command("MOUNT")
        st = "" if steer_sign is None else f" {int(steer_sign)}"
        if name is not None and steer_sign is not None:
            st += " " + ("_".join(name.split()).replace('"', "").replace("\\", "") or "auto")
        return self.command(f"MOUNT {imu_axes['forward']} {imu_axes['left']} {imu_axes['up']} "
                            f"{yaw_offset_deg:.1f}{st}")

    def list_races(self):
        """Wyścigi (protokół ≥ 2): lista i stan obszaru. Protokół ≥ 9: lista w częściach ("next");
        starszy firmware ucinał ją bez znaku."""
        r = self.command("RLIST")
        nxt = r.pop("next", None)
        while nxt is not None:
            q = self.command(f"RLIST {nxt}")
            if not q.get("races"):
                break
            r["races"] += q["races"]
            nxt = q.get("next")
        return r

    def drift_dec(self, coast_g: float, brake_g: float, auto: bool):
        """Opóźnienie bokiem w drifcie [g] – puszczony gaz, hamulec – i nauka w jeździe (protokół ≥ 9)."""
        return self.command(f"DRIFT DEC {coast_g:.3f} {brake_g:.3f} {1 if auto else 0}")

    def get_block(self, seq: int, offset: int, length: int, timeout: float = 5.0,
                  cmd: str = "GET") -> bytes:
        self._drain()
        self.link.write(f"{cmd} {seq} {offset} {length}\n".encode())
        t_end = time.time() + timeout
        while time.time() < t_end:
            try:
                fr = self.link.demux.frames.get(timeout=0.05)
            except queue.Empty:
                try:
                    line = self.link.demux.lines.get_nowait()
                    obj = json.loads(line) if line.startswith("{") else {"err": line}
                    if obj.get("err") == "BUSY":
                        raise DeviceBusy("GET")
                    raise ProtocolError(f"GET: {obj}")
                except queue.Empty:
                    continue
            magic, s, off, ln, crc = BLOCK_HDR.unpack_from(fr)
            data = fr[BLOCK_HDR.size :]
            if s != seq or off != offset:
                raise ProtocolError("Niezgodny blok")
            if zlib.crc32(data) & 0xFFFFFFFF != crc:
                raise ProtocolError("CRC bloku")
            return data
        raise ProtocolError("Timeout GET")

    def download(self, run: dict, block: int = GET_BLOCK_MAX, retries: int = 3, progress=None,
                 resume: bytes = b"", race: bool = False) -> bytes:
        """Pobiera start (albo wyścig: race=True) blokami; wznawia od len(resume).
        Weryfikuje CRC całości."""
        seq, total = run["seq"], run["len"]
        buf = bytearray(resume)
        while len(buf) < total:
            n = min(block, total - len(buf))
            for attempt in range(retries + 1):
                try:
                    buf += self.get_block(seq, len(buf), n, cmd="RGET" if race else "GET")
                    break
                except DeviceBusy:
                    time.sleep(0.5)
                except ProtocolError:
                    if attempt == retries:
                        raise
            if progress:
                progress(len(buf), total)
        crc = (zlib.crc32(bytes(buf[:-4])) & 0xFFFFFFFF) if race else run_crc(bytes(buf))
        if crc != run["crc"]:
            raise ProtocolError("CRC wyścigu niezgodne" if race else "CRC startu niezgodne")
        return bytes(buf)

    def delete(self, seq: int):
        r = self.command(f"DEL {seq}")
        if r != f"OK DEL {seq}":
            raise ProtocolError(f"DEL: {r}")


    def delete_race(self, seq: int):
        """Oznacza wyścig jako pobrany (miejsce zwalnia dopiero erase_races)."""
        r = self.command(f"RDEL {seq}")
        if r != f"OK RDEL {seq}":
            raise ProtocolError(f"RDEL: {r}")

    def erase_races(self, force: bool = False):
        r = self.command("RERASE FORCE" if force else "RERASE")
        if r != "OK":
            raise ProtocolError(f"RERASE: {r}")


def run_crc(data: bytes) -> int:
    """CRC32 startu – z polem RunHeader.crc32 (bajty 124..127) liczonym jako 0."""
    if len(data) < 128:
        return zlib.crc32(data) & 0xFFFFFFFF
    c = zlib.crc32(data[:124])
    c = zlib.crc32(b"\0\0\0\0", c)
    c = zlib.crc32(data[128:], c)
    return c & 0xFFFFFFFF

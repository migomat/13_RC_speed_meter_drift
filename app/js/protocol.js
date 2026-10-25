// Protokół X4F-LOG nad dowolnym Transportem (docs/PROTOCOL.md).
// Notyfikacja = bajt typu (0x01 tekst, 0x02 ramka binarna) + dane.
// Ramka binarna (GET): BlockHeader {magic, seq, offset, len, crc32} (20 B) + dane.

import { crc32, runCrc, raceCrc } from './format.js';

export const FRAME_TEXT = 0x01;
export const FRAME_BINARY = 0x02;
export const BLOCK_MAGIC = 0x42463458;
export const GET_BLOCK_MAX = 4096;

export class ProtocolError extends Error {}
export class BusyError extends ProtocolError {}

export class Device {
  constructor(transport) {
    this.t = transport;
    this.textBuf = '';
    this.bin = null;          // {need, parts, got}
    this.lineWaiters = [];
    this.lines = [];
    this.frameWaiters = [];
    this.frames = [];
    this.liveCb = null;
    this.chain = Promise.resolve();
    this.decoder = new TextDecoder();
    transport.onFrame((u8) => this._onFrame(u8));
  }

  onLive(cb) { this.liveCb = cb; }

  _onFrame(u8) {
    if (!u8 || !u8.length) return;
    const type = u8[0];
    const data = u8.subarray(1);
    if (type === FRAME_TEXT) {
      this.textBuf += this.decoder.decode(data, { stream: true });
      let nl;
      while ((nl = this.textBuf.indexOf('\n')) >= 0) {
        const line = this.textBuf.slice(0, nl).trim();
        this.textBuf = this.textBuf.slice(nl + 1);
        this._line(line);
      }
    } else if (type === FRAME_BINARY) {
      this._binary(data);
    }
  }

  _line(line) {
    if (line.startsWith('{"live"')) {
      if (this.liveCb) { try { this.liveCb(JSON.parse(line)); } catch (e) { /* uszkodzona linia */ } }
      return;
    }
    const w = this.lineWaiters.shift();
    if (w) w(line); else this.lines.push(line);
  }

  _binary(data) {
    let off = 0;
    while (off < data.length) {
      if (!this.bin) {
        // nagłówek bloku (20 B) może być podzielony między notyfikacje (MTU 23)
        const head = this.binHead || new Uint8Array(0);
        const take = Math.min(20 - head.length, data.length - off);
        const h2 = new Uint8Array(head.length + take);
        h2.set(head);
        h2.set(data.subarray(off, off + take), head.length);
        off += take;
        if (h2.length < 20) { this.binHead = h2; return; }
        this.binHead = null;
        const dv = new DataView(h2.buffer, 0, 20);
        if (dv.getUint32(0, true) !== BLOCK_MAGIC) return;
        const len = dv.getUint32(12, true);
        this.bin = { need: 20 + len, buf: new Uint8Array(20 + len), got: 20 };
        this.bin.buf.set(h2);
        if (len === 0) { this._frameDone(); continue; }
        continue;
      }
      const take = Math.min(this.bin.need - this.bin.got, data.length - off);
      this.bin.buf.set(data.subarray(off, off + take), this.bin.got);
      this.bin.got += take;
      off += take;
      if (this.bin.got === this.bin.need) this._frameDone();
    }
  }

  _frameDone() {
    const fr = this.bin.buf;
    this.bin = null;
    const w = this.frameWaiters.shift();
    if (w) w(fr); else this.frames.push(fr);
  }

  _nextLine(timeoutMs) {
    if (this.lines.length) return Promise.resolve(this.lines.shift());
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        const i = this.lineWaiters.indexOf(fn);
        if (i >= 0) this.lineWaiters.splice(i, 1);
        reject(new ProtocolError('Brak odpowiedzi z urządzenia'));
      }, timeoutMs);
      const fn = (l) => { clearTimeout(timer); resolve(l); };
      this.lineWaiters.push(fn);
    });
  }

  _serial(fn) {
    const p = this.chain.then(fn, fn);
    this.chain = p.catch(() => {});
    return p;
  }

  _reset() {
    this.lines = [];
    this.frames = [];
    this.bin = null;
    this.binHead = null;
  }

  /** Komenda tekstowa → obiekt (JSON) albo tekst ("OK", "OK DEL n"). */
  command(cmd, timeoutMs = 4000) {
    return this._serial(async () => {
      this._reset();
      await this.t.send(cmd + '\n');
      const line = await this._nextLine(timeoutMs);
      if (line.startsWith('{')) {
        const obj = JSON.parse(line);
        if (obj.err) {
          if (obj.err === 'BUSY') throw new BusyError('Urządzenie zajęte (nagrywanie/zapis)');
          throw new ProtocolError(`${cmd}: ${obj.err}`);
        }
        return obj;
      }
      return line;
    });
  }

  info() { return this.command('INFO'); }
  status() { return this.command('STATUS'); }
  async list() { return (await this.command('LIST')).runs; }
  setTime(unixS = Math.floor(Date.now() / 1000)) { return this.command(`TIME ${unixS}`); }
  live(on) { return this.command(`LIVE ${on ? 1 : 0}`); }
  vmaxReset() { return this.command('VMAX RESET'); }
  /** Punkty driftu i montaż (protokół ≥ 3). */
  driftReset() { return this.command('DRIFT RESET'); }
  mount(axes, yawDeg) {
    if (!axes) return this.command('MOUNT');
    return this.command(`MOUNT ${axes.forward} ${axes.left} ${axes.up} ${(+yawDeg || 0).toFixed(1)}`);
  }
  arm() { return this.command('ARM'); }

  /** Wyścigi (protokół ≥ 2): {races, state, used, size}. */
  rlist() { return this.command('RLIST'); }

  getBlock(seq, offset, len, timeoutMs = 6000, cmd = 'GET') {
    return this._serial(async () => {
      this._reset();
      await this.t.send(`${cmd} ${seq} ${offset} ${len}\n`);
      const fr = await new Promise((resolve, reject) => {
        const timer = setTimeout(() => cleanup(new ProtocolError('Timeout pobierania bloku')), timeoutMs);
        const onFrame = (f) => cleanup(null, f);
        const onLine = (l) => {
          let e;
          try { e = JSON.parse(l); } catch (x) { e = { err: l }; }
          cleanup(e.err === 'BUSY' ? new BusyError('Urządzenie zajęte') : new ProtocolError(`GET: ${e.err}`));
        };
        const cleanup = (err, f) => {
          clearTimeout(timer);
          this.frameWaiters = this.frameWaiters.filter((x) => x !== onFrame);
          this.lineWaiters = this.lineWaiters.filter((x) => x !== onLine);
          if (err) reject(err); else resolve(f);
        };
        if (this.frames.length) return cleanup(null, this.frames.shift());
        this.frameWaiters.push(onFrame);
        this.lineWaiters.push(onLine);
      });
      const dv = new DataView(fr.buffer, fr.byteOffset, 20);
      const s = dv.getUint32(4, true), off = dv.getUint32(8, true), ln = dv.getUint32(12, true), crc = dv.getUint32(16, true);
      const data = fr.subarray(20, 20 + ln);
      if (s !== seq || off !== offset) throw new ProtocolError('Niezgodny blok');
      if (crc32(data) !== crc) throw new ProtocolError('Błąd CRC bloku');
      return data;
    });
  }

  /**
   * Pobiera start blokami (do 3 ponowień na blok), wznawia od partial.length.
   * Weryfikuje CRC całości z LIST. Zwraca Uint8Array.
   */
  async download(run, { partial = null, onProgress = null, blockSize = GET_BLOCK_MAX, retries = 3, race = false } = {}) {
    const buf = new Uint8Array(run.len);
    let got = 0;
    if (partial && partial.length <= run.len) { buf.set(partial); got = partial.length; }
    while (got < run.len) {
      const n = Math.min(blockSize, run.len - got);
      let data = null, lastErr = null;
      for (let a = 0; a <= retries && !data; a++) {
        try {
          data = await this.getBlock(run.seq, got, n, 6000, race ? 'RGET' : 'GET');
        } catch (e) {
          lastErr = e;
          if (e instanceof BusyError) await new Promise((r) => setTimeout(r, 700));
          if (!this.t.connected) break;
        }
      }
      if (!data) {
        const err = !this.t.connected ? new ProtocolError('Połączenie zerwane')
          : (lastErr || new ProtocolError('Pobieranie przerwane'));
        err.partial = buf.slice(0, got);
        throw err;
      }
      buf.set(data, got);
      got += data.length;
      if (onProgress) onProgress(got, run.len);
    }
    if ((race ? raceCrc(buf) : runCrc(buf)) !== run.crc) {
      const err = new ProtocolError(race ? 'CRC wyścigu niezgodne' : 'CRC startu niezgodne');
      err.partial = null;
      throw err;
    }
    return buf;
  }

  async del(seq) {
    const r = await this.command(`DEL ${seq}`);
    if (r !== `OK DEL ${seq}`) throw new ProtocolError(`DEL: ${typeof r === 'string' ? r : JSON.stringify(r)}`);
  }

  /** Wyścig oznaczony jako pobrany (miejsce zwalnia dopiero rerase). */
  async rdel(seq) {
    const r = await this.command(`RDEL ${seq}`);
    if (r !== `OK RDEL ${seq}`) throw new ProtocolError(`RDEL: ${typeof r === 'string' ? r : JSON.stringify(r)}`);
  }

  /** Kasowanie całej pamięci wyścigów; bez force tylko gdy wszystkie pobrane. */
  async rerase(force = false) {
    const r = await this.command(force ? 'RERASE FORCE' : 'RERASE');
    if (r !== 'OK') throw new ProtocolError(`RERASE: ${typeof r === 'string' ? r : JSON.stringify(r)}`);
  }
}

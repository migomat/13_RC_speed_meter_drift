// Osie płytki → osie auta wg ustawienia imu_axes ({forward, left, up}: '+x', '-y', …)
// – jak parse_axis()/map_axes() w tools/rc_launch/analyze.py.

export function parseAxis(spec) {
  spec = String(spec).trim().toLowerCase();
  return { idx: 'xyz'.indexOf(spec[spec.length - 1]), sign: spec.startsWith('-') ? -1 : 1 };
}

export function mapAxes(bx, by, bz, axes) {
  const board = [bx, by, bz];
  return ['forward', 'left', 'up'].map((name) => {
    const { idx, sign } = parseAxis(axes[name]);
    const src = board[idx];
    const out = new Float64Array(src.length);
    for (let i = 0; i < src.length; i++) out[i] = src[i] * sign;
    return out;
  });
}


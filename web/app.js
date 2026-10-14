const $ = (id) => document.getElementById(id);
const MODS = ['OFF', 'BPSK', 'QPSK', '8PSK'];
const CH = ['#b8c4b8', '#e6b45a', '#8fa7c4', '#c98f86'];
const GRID = '#232823';
const TICK = 'rgba(184,196,184,0.65)';
const CLIP = 39;  // int8 RX samples saturate here, not at 127

let ws = null;
let settings = [];
let st = null;

// ------------------------------------------------------------------ socket & tab lifecycle

const mimoClient = (window.crypto && crypto.randomUUID)
    ? crypto.randomUUID()
    : String(Date.now()) + '-' + Math.random().toString(16).slice(2);
let mimoLeaving = false;
let attachTimer = null;

function mimoAttach() {
    if (mimoLeaving) return;
    fetch('/api/apps/attach?app=mimo&client=' + encodeURIComponent(mimoClient), {
        method: 'POST',
        keepalive: true
    }).catch(() => {});
}

function mimoDetach() {
    if (mimoLeaving) return;
    mimoLeaving = true;
    if (attachTimer) {
        clearInterval(attachTimer);
        attachTimer = null;
    }
    if (ws) {
        try { ws.close(); } catch (e) {}
    }
    const url = '/api/apps/detach?app=mimo&client=' + encodeURIComponent(mimoClient);
    try {
        if (navigator.sendBeacon) navigator.sendBeacon(url);
    } catch (e) {}
    fetch(url, { method: 'POST', keepalive: true }).catch(() => {});
}

function mimoResume() {
    if (!mimoLeaving && attachTimer) return;
    mimoLeaving = false;
    if (!attachTimer)
        attachTimer = setInterval(mimoAttach, 1000);
    mimoAttach();
    if (!ws || ws.readyState > 1) connect();
}

mimoAttach();
attachTimer = setInterval(mimoAttach, 1000);

// Detach when the tab/page is closed or navigated away, but do NOT detach on
// visibilitychange (tab hidden) so multiple Quads can be tested across tabs.
window.addEventListener('pagehide', mimoDetach);
window.addEventListener('beforeunload', mimoDetach);
window.addEventListener('pageshow', mimoResume);
window.addEventListener('resume', mimoResume);

function connect() {
    const proto = (location.protocol === 'https:' ? 'wss://' : 'ws://');
    const base = location.pathname.endsWith('/') ? location.pathname : location.pathname + '/';
    const wsUrl = proto + location.host + (base === '/' ? '/ws' : base + 'ws');
    ws = new WebSocket(wsUrl);
    ws.onopen = () => {
        setStatus('CONNECTED', '');
        send({ t: 'hello' });
        send({ t: 'viz', on: true });
    };
    ws.onclose = () => {
        setStatus('CONNECTING...', 'warn');
        if (!mimoLeaving) setTimeout(connect, 1000);
    };
    ws.onmessage = (ev) => {
        let m;
        try { m = JSON.parse(ev.data); } catch (e) { return; }
        if (m.t === 'st') onStatus(m);
        else if (m.t === 'settings') onSettings(m);
        else if (m.t === 'chat') onChat(m.msgs);
        else if (m.t === 'viz') onViz(m);
    };
}
function send(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }

function setStatus(txt, cls) {
    const el = $('status');
    el.textContent = txt;
    el.className = cls || '';
}

// ------------------------------------------------------------------ tooltips

const tip = $('tip');
let tipFor = null;
function showTip(el) {
    tipFor = el;
    tip.textContent = el.dataset.tip;
    tip.style.display = 'block';
    const r = el.getBoundingClientRect(), tw = tip.offsetWidth, th = tip.offsetHeight;
    let x = Math.min(Math.max(8, r.left), window.innerWidth - tw - 8);
    let y = r.bottom + 6;
    if (y + th > window.innerHeight - 8) y = Math.max(8, r.top - th - 6);
    tip.style.left = x + 'px';
    tip.style.top = y + 'px';
}
function hideTip() { tipFor = null; tip.style.display = 'none'; }
document.addEventListener('pointerover', (e) => {
    const t = e.target.closest && e.target.closest('[data-tip]');
    if (t) { if (t !== tipFor) showTip(t); } else if (e.pointerType === 'mouse') hideTip();
});
document.addEventListener('pointerdown', (e) => {
    const t = e.target.closest && e.target.closest('[data-tip]');
    if (t) showTip(t); else hideTip();
});
document.addEventListener('scroll', hideTip, true);

// ------------------------------------------------------------------ formatting

const f1 = (v) => (v == null || !isFinite(v)) ? '-' : v.toFixed(1);
const kb = (bps) => (bps / 1e3).toFixed(1);
const khz = (hz) => (hz / 1e3).toFixed(1) + ' kHz';
function sci(v) {
    if (v == null || v < 0) return '-';
    if (v === 0) return '0';
    return v.toExponential(1);
}
const bits = (m) => [0, 1, 2, 3].filter((k) => m >> k & 1);

// ------------------------------------------------------------------ status

function onStatus(m) {
    st = m;
    $('host').textContent = m.host.toUpperCase();
    const keyed = m.tx_en && (m.tx_elems & 15);
    if (!keyed) setStatus(m.up ? 'LINK  TX OFF' : 'LISTEN', m.up ? '' : 'warn');
    else setStatus(m.up ? 'LINK' : 'NO PEER', m.up ? '' : 'warn');

    const gp = m.gp_rx + m.gp_tx;
    $('gp-rx').textContent = kb(m.gp_rx);
    $('gp-tx').textContent = kb(m.gp_tx);
    $('se').textContent = m.bw_sym > 0 ? (gp / m.bw_sym).toFixed(2) : '-';
    const scale = Math.max(gp, m.bw_sym, 1) * 1.05;
    $('bar-data').style.width = (100 * gp / scale) + '%';
    $('bar-bw').style.width = (100 * m.bw_sym / scale) + '%';
    $('bar-data-v').textContent = kb(gp) + ' kb/s';
    $('bar-bw-v').textContent = khz(m.bw_sym);
    $('bw-occ').textContent = m.bw90 > 0 ? `${(m.bw90 / 1e3).toFixed(0)} / ${(m.bw99 / 1e3).toFixed(0)} kHz` : '-';
    $('lobe').textContent = m.lobe > 0 ? (100 * m.lobe).toFixed(0) + ' %' : '-';
    $('phy').textContent = `${kb(m.phy_rx)} / ${kb(m.phy_tx)} kb/s`;
    const f = m.up ? m.f : null;
    const ndata = (mods) => mods ? mods.filter((x) => x > 0).length : 0;
    $('streams').textContent = `${f ? ndata(f.mods) : '-'} / ${keyed ? ndata(m.tx_mods) : 0}`;

    const rows = [];
    for (let k = 0; k < 4; k++) {
        const rm = f ? f.mods[k] : -1, tm = keyed ? m.tx_mods[k] : 0;
        const rs = rm > 0 ? f.snr[k] : null, ps = tm > 0 && m.peer_valid ? m.peer_snr[k] : null;
        const bar = (v, cls) => `<span class="sbar${cls || ''}"><i style="width:${v == null ? 0 : Math.max(0, Math.min(100, v / 30 * 100))}%"></i></span>${v == null ? '-' : f1(v) + ' dB'}`;
        rows.push(`<tr><td>${k}</td><td class="mod${rm > 0 ? '' : ' off'}">${rm >= 0 ? MODS[rm] : '-'}</td><td>${bar(rs)}</td>` +
            `<td class="pdiv mod${tm > 0 ? '' : ' off'}">${MODS[tm]}</td><td>${bar(ps, ' peer')}</td></tr>`);
    }
    $('el-body').innerHTML = rows.join('');
    $('cond').textContent = f ? f1(f.cond) : '-';
    $('model').textContent = `${f ? f1(f.model) : '-'} / ${m.peer_valid ? f1(m.peer_model) : '-'} dB`;

    const pct = (a, b) => b > 0 ? ` (${(100 * a / b).toFixed(1)} %)` : '';
    $('frames').textContent = `${m.fok} / ${m.fbad} / ${m.ffalse}`;
    $('blocks').textContent = `${m.bok} / ${m.btot}${pct(m.bok, m.btot)}`;
    $('pblocks').textContent = m.peer_valid ? `${m.peer_bok} / ${m.peer_btot}${pct(m.peer_bok, m.peer_btot)}` : '-';
    $('ber').textContent = m.ber >= 0 ? `${sci(m.ber)}  (${(m.ber_bits / 1e3).toFixed(0)} kbit)` : '-';
    $('cfo').textContent = m.f ? `${(m.f.cfo / 1e3).toFixed(2)} kHz` : '-';
    const rstate = m.rate_state && m.rate_state !== 'MANUAL' ? `  ${m.rate_state}` : '';
    $('tper').textContent = `${m.f ? m.f.T.toFixed(2) : '-'} / ${m.us_reg > 0 ? m.us_reg.toFixed(2) : '-'} µs${rstate}`;
    $('stalls').textContent = m.f ? `${m.f.stalls} / ${m.f.slot_max.toFixed(0)} µs` : '-';
    $('fsent').textContent = `${m.fsent}`;
    $('rxc').textContent = m.rxc ? m.rxc.join(' / ') : '-';

    const clip = m.lvl >= m.lvl_max || m.clip > 0.02;
    const pclip = m.peer_valid && m.peer_lvl >= m.lvl_max;
    $('r-call').textContent = m.call || 'NOCALL';
    $('r-pcall').textContent = m.peer_call || '-';
    $('r-tx').textContent = `${m.txg} dB / ${m.atpc}${m.txg >= m.txg_cap ? '  AT LIMIT' : ''}`;
    $('r-rx').textContent = `${m.rxg} dB`;
    $('r-lvl').textContent = `${m.lvl} / ${m.lvl_max}${clip ? '  CLIPPING' : ''}`;
    $('r-lvl').classList.toggle('warn', clip);
    $('r-lanes').textContent = m.lvl_lane ? m.lvl_lane.join(' ') : '-';
    $('r-peer').textContent = m.peer_valid ? `${m.peer_txg} / ${m.peer_rxg} dB` : '-';
    $('r-plvl').textContent = m.peer_valid ? `${m.peer_lvl} / ${m.lvl_target}${pclip ? '  CLIPPING' : ''}` : `- / ${m.lvl_target}`;
    $('r-plvl').classList.toggle('warn', !!pclip);

    drawSep(m, true);
    drawH();
}

// ------------------------------------------------------------------ spatial separation

// eigenvalues of a real symmetric matrix (cyclic Jacobi), a is modified
function eigSym(a) {
    const n = a.length;
    for (let sweep = 0; sweep < 30; sweep++) {
        let off = 0;
        for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += a[p][q] * a[p][q];
        if (off < 1e-18) break;
        for (let p = 0; p < n; p++)
            for (let q = p + 1; q < n; q++) {
                if (Math.abs(a[p][q]) < 1e-30) continue;
                const th = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
                const c = 1 / Math.sqrt(t * t + 1), s = t * c;
                for (let k = 0; k < n; k++) {
                    const akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (let k = 0; k < n; k++) {
                    const apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
            }
    }
    return a.map((r, i) => r[i]);
}

// Channel submatrix over enabled local RX channels and active peer TX elements.
// Hc is [lane][element] re/im. Singular values come from the Gram matrix H^H H (n x n
// Hermitian), embedded as the real 2n x 2n [[Re, -Im], [Im, Re]], whose eigenvalues pair up.
function sepCalc(m) {
    const f = m.f;
    if (!m.up || !f || !f.Hc) return null;
    const rows = bits(m.rx_lanes), cols = [0, 1, 2, 3].filter((k) => f.mods[k] > 0);
    const n = cols.length;
    if (!n || !rows.length) return null;
    const h = cols.map((k) => rows.map((r) => [f.Hc[2 * (4 * r + k)], f.Hc[2 * (4 * r + k) + 1]]));
    const G = [];
    for (let i = 0; i < n; i++) {
        G.push([]);
        for (let j = 0; j < n; j++) {
            let re = 0, im = 0;
            for (let r = 0; r < rows.length; r++) {
                const [a, b] = h[i][r], [c, d] = h[j][r];
                re += a * c + b * d;
                im += a * d - b * c;
            }
            G[i].push([re, im]);
        }
    }
    const M = [];
    for (let i = 0; i < 2 * n; i++) M.push(new Array(2 * n).fill(0));
    for (let i = 0; i < n; i++)
        for (let j = 0; j < n; j++) {
            const [re, im] = G[i][j];
            M[i][j] = re; M[i + n][j + n] = re;
            M[i + n][j] = im; M[i][j + n] = -im;
        }
    const ev = eigSym(M).map((v) => Math.max(v, 0)).sort((a, b) => b - a).filter((_, i) => i % 2 === 0);
    const s1 = ev.reduce((a, b) => a + b, 0), s2 = ev.reduce((a, b) => a + b * b, 0);
    const ang = [];
    let worst = null;
    for (let i = 0; i < n; i++) {
        ang.push([]);
        for (let j = 0; j < n; j++) {
            const [re, im] = G[i][j];
            const rho = Math.min(1, Math.hypot(re, im) / Math.sqrt(G[i][i][0] * G[j][j][0] + 1e-30));
            const a = Math.acos(rho) * 180 / Math.PI;
            ang[i].push(a);
            if (j > i && (!worst || a < worst.a)) worst = { a, i: cols[i], j: cols[j] };
        }
    }
    return { cols, rows, sv: ev.map(Math.sqrt), paths: s2 > 0 ? s1 * s1 / s2 : 0, ang, worst };
}

const sepHist = [];
function drawSep(m, record) {
    const S = sepCalc(m);
    const now = m.now;
    if (S && record) sepHist.push([now, S.paths, S.cols.length]);
    while (sepHist.length && sepHist[0][0] < now - 60) sepHist.shift();

    $('sep-n').textContent = S ? S.paths.toFixed(2) : '-';
    $('sep-of').textContent = S ? `OF ${S.cols.length}` : '';
    $('sep-w').textContent = S && S.worst ? S.worst.a.toFixed(0) + '°' : '-';
    $('sep-wp').textContent = S && S.worst ? `EL${S.worst.i}-EL${S.worst.j}` : '';
    let note = '';
    if (!S) note = m.up ? '' : 'NO PEER FRAMES';
    else if (S.cols.length === 1) note = 'PEER TRANSMITTING ON 1 ELEMENT (SINGLE STREAM)';
    else if (S.rows.length < S.cols.length) note = `ONLY ${S.rows.length} RX CHANNEL${S.rows.length > 1 ? 'S' : ''} ON: AT MOST ${S.rows.length} PATHS`;
    $('sep-note').textContent = note;

    // singular values, dB below the strongest
    let P = prep($('sep-sv'));
    if (P) {
        const { ctx, w, h } = P;
        ctx.font = '9px ui-monospace, Menlo, monospace';
        ctx.textBaseline = 'middle';
        const n = S ? S.sv.length : 0, rowh = Math.min(26, (h - 12) / Math.max(n, 1));
        ctx.fillStyle = TICK;
        ctx.textAlign = 'left';
        ctx.fillText('PATH STRENGTH', 0, 5);
        const L = 26, R = 46, bw = w - L - R;
        for (let i = 0; i < n; i++) {
            const db = 20 * Math.log10(S.sv[i] / S.sv[0] + 1e-9);
            const y = 14 + i * rowh;
            const frac = Math.max(0, 1 + db / 30);
            ctx.fillStyle = TICK;
            ctx.textAlign = 'left';
            ctx.fillText('σ' + (i + 1), 2, y + rowh / 2 - 1);
            ctx.strokeStyle = GRID;
            ctx.strokeRect(L + 0.5, y + 2.5, bw, rowh - 6);
            ctx.fillStyle = CH[0];
            ctx.globalAlpha = 0.75;
            ctx.fillRect(L + 1, y + 3, (bw - 1) * frac, rowh - 7);
            ctx.globalAlpha = 1;
            ctx.fillStyle = '#e8ece8';
            ctx.textAlign = 'right';
            ctx.fillText(db > -60 ? db.toFixed(1) + ' dB' : '<-60', w - 2, y + rowh / 2 - 1);
        }
    }

    // pairwise angle between element arrival patterns
    P = prep($('sep-ang'));
    if (P) {
        const { ctx, w, h } = P;
        ctx.font = '9px ui-monospace, Menlo, monospace';
        ctx.fillStyle = TICK;
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText('ANGLE, 90° = IDEAL', 0, 5);
        const n = S ? S.cols.length : 0;
        if (n > 1) {
            const L = 24, T = 24, cs = Math.min((w - L - 2) / n, (h - T - 2) / n);
            ctx.textAlign = 'center';
            for (let i = 0; i < n; i++) {
                ctx.fillStyle = TICK;
                ctx.fillText('EL' + S.cols[i], L + (i + 0.5) * cs, T - 6);
                ctx.fillText('EL' + S.cols[i], 11, T + (i + 0.5) * cs);
                for (let j = 0; j < n; j++) {
                    const x = L + j * cs, y = T + i * cs;
                    if (i === j) { ctx.strokeStyle = GRID; ctx.strokeRect(x + 1.5, y + 1.5, cs - 3, cs - 3); continue; }
                    const a = S.ang[i][j], rel = a / 90;
                    ctx.fillStyle = `rgba(184,196,184,${0.06 + 0.6 * rel})`;
                    ctx.fillRect(x + 1, y + 1, cs - 2, cs - 2);
                    ctx.fillStyle = rel > 0.6 ? '#0c110c' : '#e8ece8';
                    ctx.fillText(a.toFixed(0) + '°', x + cs / 2, y + cs / 2);
                }
            }
        }
    }

    // PATHS over the last minute
    const hist = sepHist.map((e) => e[1]);
    const top = Math.max(2, ...sepHist.map((e) => e[2]));
    plot($('sep-hist'), {
        xr: [-60, 0], yr: [0, top], xl: 's', yl: '', compact: true,
        series: [{ y: hist, x: sepHist.map((e) => e[0] - now), c: CH[0] }],
    });
}

// ------------------------------------------------------------------ settings

const TIPS = {
    tx: 'Enable local transmitter. When off, this unit only receives and the peer continues beaconing.',
    mod: 'Phase modulation on enabled local TX elements: BPSK (1 bit/chip), QPSK (2), 8PSK (3). Sent in the frame header.',
    tx_elems: 'Local antenna elements used for transmit. Off elements are powered down and marked OFF in the frame header.',
    rx_lanes: 'Local RX channels fed to the decoder. Disabled channels are zeroed after the DDC.',
    atpc: 'Adjust local TX GAIN once per burst round-trip so PEER RX LEVEL matches PEER RX LEVEL TARGET (0.7× error up to ±4 dB; −6 dB above ADC LEVEL LIMIT).',
    tx_gain: 'Local MAX2850 TX gain (0–63 dB). Fixed gain when AUTO TX POWER is off; initial gain when on.',
    tx_gain_max: 'Upper ceiling for local TX GAIN (also capped at 40 dB unless the QuadRF control page high-power notice is accepted).',
    level_target: 'Target peak ADC level (counts) at the peer receiver that local AUTO TX POWER steers toward.',
    level_max: 'ADC count threshold for receiver compression (hardware clips near 39). Above this, AUTO TX POWER backs off 6 dB immediately.',
    rx_gain: 'Local FPGA RX gain (0–63 dB, hardware AGC disabled). Typical bench value at 1–3 m is 10–16 dB.',
    auto_rx: 'Step local RX GAIN by 6 dB if local RX ADC level clips with peer TX at 0 dB, or sits >12 dB below target with peer TX at max.',
    burst: 'Frames transmitted per TDD turn. Higher values reduce turnaround overhead (~35 ms/turn) at the cost of higher latency.',
    mode: 'SDM: independent data stream per TX element (spatial multiplexing). CDM4 / CDM8: BPSK spread by 4- or 8-chip Walsh×PN codes across elements.',
    auto_rate: 'Step TX write period between 6 and 18.9 µs based on peer block delivery. Announces each burst\'s next period in the feedback block.',
    rate_us: 'TX JTAG register write period T (0 = unpaced ~19.2 µs). Fixed when AUTO LINK RATE is off; starting period when on.',
    rate_min_us: 'Shortest TX write period (highest symbol rate) allowed when AUTO LINK RATE is on.',
    rate_max_us: 'Longest TX write period (lowest symbol rate) allowed when AUTO LINK RATE is on.',
    rate_min_ok: 'Minimum peer block delivery ratio before AUTO LINK RATE steps down to a slower write period.',
    rate_up_ok: 'Peer block delivery ratio required over 3 consecutive bursts before AUTO LINK RATE steps to a faster write period.',
    lead_us: 'Unmodulated carrier held after TX key-up before frame 0 (~1 ms for MAX2850 synth settling; last 280 µs used by RX for CFO estimation).',
    gap_us: 'Extra unmodulated carrier gap between consecutive frames in a burst (plus ~120 µs JTAG ioctl overhead).',
    turn_ms: 'Quiet wait after the peer\'s final burst frame before this unit keys up.',
    quiet_ms: 'Max wait (up to 3×) for the 8 MB CSI RX ring to fill and park its DMA thread before TX, preventing memory bus stalls during JTAG writes.',
    beacon_min_ms: 'Lower bound of random backoff timeout before transmitting a new burst when no peer frames are heard.',
    beacon_max_ms: 'Upper bound of random backoff timeout before transmitting a new burst when no peer frames are heard.',
    lpf: 'Cascaded boxcar filter length L (L and L+1 taps at 2.336 MS/s) after the DDC. Attenuates ±250–350 kHz synth spurs; decoder guards L samples around each write boundary. 0 = off.',
    pm_lines: 'Number of discrete TX PLL phase-spur tones (25–280 kHz) fitted by least squares and subtracted per frame. 0 = off.',
    det_k: 'Frame-start detector threshold as a multiple of the estimated per-channel noise variance N₀.',
    det_rel: 'Minimum step energy relative to total 8-sample window power, preventing false triggers on strong unmodulated carrier phase wander.',
    rx_gap_us: 'Minimum step-free carrier duration required before a step trigger is queued as a frame start (TX leaves ~680 µs).',
    pre_gate_db: 'Maximum normalized least-squares residual (10 log₁₀ ‖Y − HX‖²/‖Y‖²) over the 24-chip preamble before a candidate is rejected as a false start.',
    sigma_us: 'Gaussian timing prior standard deviation σ (µs) for write boundaries arriving earlier than expected.',
    late_mult: 'Multiplier on TIMING SIGMA for late write boundaries (σ_late = mult × σ), allowing the tracker to follow stretched JTAG writes.',
    late_cap: 'Maximum quadratic timing penalty for late writes, preventing multi-period JTAG stalls from overwhelming the signal fit.',
    win_lo: 'Start of the boundary search window, in multiples of write period T after the previous boundary.',
    win_hi: 'End of the boundary search window, in multiples of write period T after the previous boundary.',
    lam: 'Weight λ applied to the timing prior penalty relative to the log-likelihood signal cost during boundary tracking.',
    la_margin: 'Cost margin within which a competing boundary >0.4T away triggers a 1-write lookahead check. 0 = off.',
    redecide: 'Post-tracking ML passes that re-decide each payload write pair over both full slots using the whole-frame H fit.',
    ls_win: 'Sliding window length (in slots, refit every 8 writes) for decision-directed least-squares H updates during tracking. 0 = LMS only.',
    pll_kp: 'Common-phase tracking loop proportional gain per write slot.',
    pll_ki: 'Common-phase tracking loop frequency integrator gain per write slot.',
    h_mu: 'LMS step size μ for per-slot channel matrix H updates during frame tracking.',
    cfo_alpha: 'Fraction of residual CFO added to the DDC NCO frequency when a valid frame exceeds ±40 kHz residual offset.',
    refine: 'Iterative passes re-aligning preamble write boundaries and re-estimating H using the known 24-chip preamble sequence.',
};

function onSettings(m) {
    const fresh = settings.length !== m.list.length;
    settings = m.list;
    if (fresh) buildSettings();
    syncSettings();
}
const setting = (k) => settings.find((s) => s.k === k);

function fmtVal(s, v) {
    if (s.type === 'sel') return s.opts.split(',')[v] || v;
    if (s.type === 'mask') return bits(v).map((k) => 'EL' + k).join(' ') || 'NONE';
    const d = s.step >= 1 ? 0 : s.step >= 0.1 ? 1 : s.step >= 0.01 ? 2 : 3;
    return v.toFixed(d) + (s.unit ? ' ' + s.unit.replace('us', 'µs') : '');
}

let pending = {};
function push(k, v) {
    pending[k] = v;
    if (push.t) return;
    push.t = setTimeout(() => {
        for (const [kk, vv] of Object.entries(pending)) send({ t: 'set', k: kk, v: vv });
        pending = {};
        push.t = null;
    }, 120);
}

const maskButtons = (k) => [0, 1, 2, 3].map((b) => `<button class="eb" data-m="${k}" data-b="${b}">EL${b}</button>`).join('');

function buildSettings() {
    const body = $('set-body');
    body.innerHTML = '';
    let grp = '';
    for (const s of settings) {
        if (s.group !== grp) {
            grp = s.group;
            const h = document.createElement('div');
            h.className = 'sect';
            h.textContent = grp;
            body.appendChild(h);
        }
        const tipAttr = TIPS[s.k] ? ` data-tip="${esc(TIPS[s.k])}"` : '';
        let row;
        if (s.type === 'bool') {
            row = document.createElement('div');
            row.className = 'tgl';
            row.innerHTML = `<span${tipAttr}>${s.label}</span><div class="sw2"></div>`;
            row.onclick = () => { s.v = s.v ? 0 : 1; push(s.k, s.v); syncSettings(); };
        } else if (s.type === 'sel') {
            row = document.createElement('div');
            row.className = 'srow';
            const opts = s.opts.split(',');
            row.innerHTML = `<label><em${tipAttr} style="font-style:normal">${s.label}</em> <span></span></label><div class="tabs">` +
                opts.map((o, i) => `<button class="tab" data-i="${i}">${o}</button>`).join('') + '</div>';
            row.querySelectorAll('.tab').forEach((b) => {
                b.onclick = () => { s.v = +b.dataset.i; push(s.k, s.v); syncSettings(); };
            });
        } else if (s.type === 'mask') {
            row = document.createElement('div');
            row.className = 'srow';
            row.innerHTML = `<label><em${tipAttr} style="font-style:normal">${s.label}</em> <span></span></label><div class="eg">${maskButtons(s.k)}</div>`;
        } else {
            row = document.createElement('div');
            row.className = 'srow';
            row.innerHTML = `<label><em${tipAttr} style="font-style:normal">${s.label}</em> <span></span></label><input type="range">`;
            const r = row.querySelector('input');
            r.min = s.lo; r.max = s.hi; r.step = s.step;
            r.oninput = () => { s.v = +r.value; row.querySelector('label span').textContent = fmtVal(s, s.v); push(s.k, s.v); };
            r.ondblclick = () => { s.v = s.def; push(s.k, s.v); syncSettings(); };
        }
        row.dataset.k = s.k;
        body.appendChild(row);
    }
    const mod = setting('mod');
    if (mod) {
        $('r-modtabs').innerHTML = mod.opts.split(',').map((o, i) => `<button class="tab" data-i="${i}">${o}</button>`).join('');
        $('r-modtabs').querySelectorAll('.tab').forEach((b) => {
            b.onclick = () => { mod.v = +b.dataset.i; push('mod', mod.v); syncSettings(); };
        });
    }
}

// element toggles, both in the RADIO box and in the config list
document.addEventListener('click', (e) => {
    const b = e.target.closest && e.target.closest('.eb');
    if (!b) return;
    const s = setting(b.dataset.m);
    if (!s) return;
    const v = s.v ^ (1 << +b.dataset.b);
    if (s.k === 'rx_lanes' && !(v & 15)) return;
    s.v = v;
    push(s.k, v);
    syncSettings();
});

function syncSettings() {
    for (const s of settings) {
        const row = document.querySelector(`#set-body [data-k="${s.k}"]`);
        if (!row) continue;
        if (s.type === 'bool') row.classList.toggle('on', !!s.v);
        else if (s.type === 'sel') {
            row.querySelector('label span').textContent = fmtVal(s, s.v);
            row.querySelectorAll('.tab').forEach((b) => b.classList.toggle('on', +b.dataset.i === s.v));
        } else if (s.type === 'mask') {
            row.querySelector('label span').textContent = fmtVal(s, s.v);
        } else {
            const r = row.querySelector('input');
            if (document.activeElement !== r) r.value = s.v;
            row.querySelector('label span').textContent = fmtVal(s, s.v);
        }
    }
    document.querySelectorAll('.eb').forEach((b) => {
        const s = setting(b.dataset.m);
        if (!s) return;
        const on = !!(s.v >> +b.dataset.b & 1);
        b.classList.toggle('on', on);
        if (b.closest('#eg')) b.textContent = on ? 'ON' : 'OFF';
    });
    const mod = setting('mod');
    if (mod) $('r-modtabs').querySelectorAll('.tab').forEach((b) => b.classList.toggle('on', +b.dataset.i === mod.v));
}

$('btn-set').onclick = () => {
    const p = $('set-pop');
    p.classList.toggle('open');
    $('btn-set').classList.toggle('on', p.classList.contains('open'));
};
$('s-defaults').onclick = () => send({ t: 'defaults' });
const help = (on) => { $('help').classList.toggle('open', on); $('btn-help').classList.toggle('on', on); };
$('btn-help').onclick = () => help(!$('help').classList.contains('open'));
$('help-x').onclick = () => help(false);
$('help').addEventListener('click', (e) => { if (e.target === $('help')) help(false); });
document.addEventListener('keydown', (e) => { if (e.key === 'Escape') help(false); });
document.addEventListener('pointerdown', (e) => {
    const p = $('set-pop');
    if (p.classList.contains('open') && !p.contains(e.target) && e.target !== $('btn-set')) {
        p.classList.remove('open');
        $('btn-set').classList.remove('on');
    }
});

// ------------------------------------------------------------------ chat

function esc(s) { return s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

function onChat(msgs) {
    const log = $('chat-log');
    const atEnd = log.scrollTop + log.clientHeight >= log.scrollHeight - 4;
    log.innerHTML = msgs.map((m) => {
        const stt = m.mine ? (m.acked ? '<span class="st">ACK</span>' : '<span class="st pend">' + (m.tries ? 'SENDING' : 'QUEUED') + '</span>') : '';
        return `<div class="m"><span class="who">${m.mine ? 'LOCAL' : 'PEER'}</span><span class="tx">${esc(m.text)}</span>${stt}</div>`;
    }).join('');
    if (atEnd) log.scrollTop = log.scrollHeight;
}
function sendChat() {
    const i = $('chat-in');
    const v = i.value.trim();
    if (!v) return;
    send({ t: 'chat', text: v });
    i.value = '';
}
$('chat-send').onclick = sendChat;
$('chat-in').addEventListener('keydown', (e) => { if (e.key === 'Enter') sendChat(); });

// ------------------------------------------------------------------ plots

function ticks(a, b, n) {
    const span = b - a;
    if (!(span > 0)) return [a];
    const raw = span / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 5, 10].map((m) => m * mag).find((s) => s >= raw) || raw;
    const out = [];
    for (let v = Math.ceil(a / step) * step; v <= b + step * 1e-6; v += step) out.push(Math.abs(v) < step * 1e-6 ? 0 : v);
    return out;
}
function tickTxt(v) {
    const a = Math.abs(v);
    if (a >= 10 || a === 0) return v.toFixed(0);
    if (a >= 1) return v.toFixed(1).replace(/\.0$/, '');
    return v.toFixed(2).replace(/0$/, '');
}

function prep(cv) {
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (w < 4 || h < 4) return null;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
        cv.width = Math.round(w * dpr);
        cv.height = Math.round(h * dpr);
    }
    const ctx = cv.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    return { ctx, w, h };
}

// o: { xr, yr, xl, yl, note, compact, series: [{ y, x?, x0, dx, c, a, pts }], hl: [{ y, c }], vl: [{ x, c }] }
function plot(cv, o) {
    const P = prep(cv);
    if (!P) return;
    const { ctx, w, h } = P;
    const L = o.compact ? 22 : 36, R = 6, T = o.compact ? 4 : 14, B = o.compact ? 12 : 16;
    const pw = w - L - R, ph = h - T - B;
    const [x0, x1] = o.xr, [y0, y1] = o.yr;
    const X = (x) => L + (x - x0) / (x1 - x0) * pw;
    const Y = (y) => T + (1 - (y - y0) / (y1 - y0)) * ph;
    ctx.font = '9px ui-monospace, Menlo, monospace';
    ctx.lineWidth = 1;
    ctx.strokeStyle = GRID;
    ctx.fillStyle = TICK;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const xu = ctx.measureText(o.xl || '').width;
    for (const t of ticks(x0, x1, Math.max(3, Math.floor(pw / 60)))) {
        const x = Math.round(X(t)) + 0.5;
        ctx.beginPath(); ctx.moveTo(x, T); ctx.lineTo(x, T + ph); ctx.stroke();
        const lab = tickTxt(t);
        if (x + ctx.measureText(lab).width / 2 < w - xu - 8) ctx.fillText(lab, x, T + ph + 2);
    }
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const t of ticks(y0, y1, o.yn || Math.max(1, Math.floor(ph / 28)))) {
        const y = Math.round(Y(t)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(L + pw, y); ctx.stroke();
        ctx.fillText(tickTxt(t), L - 4, y);
    }
    ctx.strokeStyle = '#3a423a';
    ctx.strokeRect(L + 0.5, T + 0.5, pw, ph);
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(o.yl || '', 2, 0);
    ctx.textAlign = 'right';
    ctx.textBaseline = 'bottom';
    ctx.fillText(o.xl || '', w - 2, h);

    ctx.save();
    ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
    for (const l of o.hl || []) {
        ctx.strokeStyle = l.c || '#e6b45a';
        ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(L, Y(l.y)); ctx.lineTo(L + pw, Y(l.y)); ctx.stroke();
    }
    for (const l of o.vl || []) {
        ctx.strokeStyle = l.c || '#e6b45a';
        ctx.setLineDash([4, 3]);
        ctx.beginPath(); ctx.moveTo(X(l.x), T); ctx.lineTo(X(l.x), T + ph); ctx.stroke();
    }
    ctx.setLineDash([]);
    for (const s of o.series) {
        const y = s.y;
        if (!y || !y.length) continue;
        ctx.strokeStyle = s.c;
        ctx.fillStyle = s.c;
        ctx.globalAlpha = s.a || 0.9;
        if (s.pts) {
            for (let i = 0; i < y.length; i++) {
                const xv = s.x ? s.x[i] : s.x0 + i * s.dx;
                ctx.fillRect(X(xv) - 1, Y(y[i]) - 1, 2, 2);
            }
        } else {
            ctx.beginPath();
            for (let i = 0; i < y.length; i++) {
                const xv = s.x ? s.x[i] : s.x0 + i * s.dx;
                if (y[i] == null) continue;
                if (i === 0) ctx.moveTo(X(xv), Y(y[i])); else ctx.lineTo(X(xv), Y(y[i]));
            }
            ctx.stroke();
        }
    }
    ctx.globalAlpha = 1;
    if (o.note) {
        ctx.font = '10px ui-monospace, Menlo, monospace';
        ctx.fillStyle = '#e6b45a';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(o.note, L + pw / 2, T + 4);
    }
    ctx.restore();
}

function constel(cv, pts, label) {
    const P = prep(cv);
    if (!P) return;
    const { ctx, w, h } = P;
    const s = Math.min(w, h), cx = w / 2, cy = h / 2, R = 1.6;
    const Xp = (v) => cx + v / R * s / 2, Yp = (v) => cy - v / R * s / 2;
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, cy + 0.5); ctx.lineTo(w, cy + 0.5); ctx.moveTo(cx + 0.5, 0); ctx.lineTo(cx + 0.5, h); ctx.stroke();
    ctx.beginPath(); ctx.arc(cx, cy, s / 2 / R, 0, 2 * Math.PI); ctx.stroke();
    ctx.strokeStyle = '#3a423a';
    ctx.strokeRect(0.5, 0.5, w - 1, h - 1);
    ctx.fillStyle = 'rgba(184,196,184,0.75)';
    for (let i = 0; i + 1 < pts.length; i += 2) ctx.fillRect(Xp(pts[i]) - 1, Yp(pts[i + 1]) - 1, 2, 2);
    ctx.font = '9px ui-monospace, Menlo, monospace';
    ctx.fillStyle = TICK;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText(label, 4, 3);
}

const wrap180 = (d) => d - 360 * Math.round(d / 360);

function drawH() {
    if (!st || !st.f) return;
    const P = prep($('p-h'));
    if (!P) return;
    const { ctx, w, h } = P;
    const f = st.f;
    const L = 22, T = 14;
    const cw = (w - L - 4) / 4, chh = (h - T - 4) / 4;
    const mx = Math.max(...f.Hm);
    ctx.font = '9px ui-monospace, Menlo, monospace';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let k = 0; k < 4; k++) {
        ctx.fillStyle = TICK;
        ctx.globalAlpha = f.mods[k] > 0 ? 1 : 0.4;
        ctx.fillText('EL' + k, L + (k + 0.5) * cw, 6);
    }
    for (let r = 0; r < 4; r++) {
        const lane = st.rx_lanes >> r & 1;
        ctx.globalAlpha = lane ? 1 : 0.4;
        ctx.fillStyle = TICK;
        ctx.fillText('RX' + r, 10, T + (r + 0.5) * chh);
        for (let k = 0; k < 4; k++) {
            const used = lane && f.mods[k] > 0;
            ctx.globalAlpha = used ? 1 : 0.35;
            const db = f.Hm[4 * r + k], rel = Math.max(0, Math.min(1, 1 + (db - mx) / 30));
            ctx.fillStyle = `rgba(184,196,184,${0.08 + 0.55 * rel})`;
            ctx.fillRect(L + k * cw + 1, T + r * chh + 1, cw - 2, chh - 2);
            ctx.fillStyle = rel > 0.6 ? '#0c110c' : '#e8ece8';
            ctx.fillText(`${(db - mx).toFixed(0)} dB`, L + (k + 0.5) * cw, T + (r + 0.42) * chh);
            ctx.fillText(`${wrap180(f.Hp[4 * r + k] - f.Hp[k]).toFixed(0)}°`, L + (k + 0.5) * cw, T + (r + 0.72) * chh);
        }
    }
    ctx.globalAlpha = 1;
    $('r-h').textContent = `COND ${f1(f.cond)}`;
}

const chVals = (vals, lanes, fmt) => vals.map((v, c) =>
    `<i style="color:${CH[c]};opacity:${lanes >> c & 1 ? 1 : 0.35}">${fmt(v)}</i>`).join('');

function onViz(m) {
    const lanes = m.lanes == null ? 15 : m.lanes;
    const note = m.trig === false ? 'NO PEER SIGNAL IN LAST 1 s' : '';
    const la = (c) => lanes >> c & 1 ? 0.8 : 0.15;
    if (m.adc) {
        const a = m.adc, n = a.psd[0].length, fsM = a.fs / 1e6;
        plot($('p-adc'), {
            xr: [-fsM / 2, fsM / 2], yr: [-100, 0], xl: `MHz from LO, fs ${fsM.toFixed(2)} MS/s`, yl: 'dBFS', note,
            series: a.psd.map((y, c) => ({ y, x0: -fsM / 2 + fsM / n / 2, dx: fsM / n, c: CH[c], a: la(c) })),
        });
        $('r-adc').innerHTML = 'RMS' + chVals(a.rms, lanes, (r) => (20 * Math.log10(r / 127 + 1e-9)).toFixed(0)) + ' dBFS';
        // the loudest enabled channel, where clipping would show first
        let c0 = -1;
        for (let c = 0; c < 4; c++) if (lanes >> c & 1 && (c0 < 0 || a.peak[c] > a.peak[c0])) c0 = c;
        if (c0 < 0) c0 = 0;
        const iq = a.iq[c0], I = [], Q = [];
        for (let i = 0; i + 1 < iq.length; i += 2) { I.push(iq[i]); Q.push(iq[i + 1]); }
        const dtu = 1e6 / a.fs;
        plot($('p-iq'), {
            xr: [0, I.length * dtu], yr: [-48, 48], xl: 'µs', yl: 'counts', yn: 6, note,
            series: [{ y: I, x0: 0, dx: dtu, c: CH[c0] }, { y: Q, x0: 0, dx: dtu, c: '#e8ece8', a: 0.5 }],
            hl: [{ y: CLIP, c: '#d05545' }, { y: -CLIP, c: '#d05545' }],
        });
        $('r-iq').innerHTML = `CH${c0} I/Q  PEAK` + chVals(a.peak, lanes, (p) => p.toFixed(0));
    }
    if (m.ddc) {
        const d = m.ddc, n = d.psd[0].length, fsk = d.fs / 1e3;
        plot($('p-ddc'), {
            xr: [-fsk / 2, fsk / 2], yr: [-110, -10], xl: 'kHz from NCO', yl: 'dBFS', note,
            series: d.psd.map((y, c) => ({ y, x0: -fsk / 2 + fsk / n / 2, dx: fsk / n, c: CH[c], a: la(c) })),
            vl: [{ x: 0, c: '#3a423a' }],
        });
        $('r-ddc').textContent = `NCO ${(d.nco / 1e3).toFixed(1)} kHz  ${(d.fs / 1e6).toFixed(3)} MS/s`;
        const dms = d.dt_us / 1e3;
        const envN = (d.env[0] || []).length;
        plot($('p-env'), {
            xr: [0, envN * dms], yr: [0, 48], xl: 'ms', yl: 'counts', yn: 5, note,
            series: d.env.map((y, c) => ({ y, x0: 0, dx: dms, c: CH[c], a: la(c) })),
            hl: [{ y: CLIP, c: '#d05545' }],
        });
        $('r-env').innerHTML = 'PEAK' + chVals(d.env.map((e) => Math.max(...e)), lanes, (p) => p.toFixed(0));
    }
    if (m.fpsd) {
        const p = m.fpsd, n = p.db.length, fsk = p.fs / 1e3, bw = p.bw_sym / 1e3;
        plot($('p-fpsd'), {
            xr: [-150, 150], yr: [-50, 2], xl: 'kHz', yl: 'dB',
            series: [{ y: p.db, x0: -fsk / 2, dx: fsk / n, c: CH[0] }],
            vl: [{ x: -bw }, { x: bw }],
        });
        $('r-fpsd').textContent = `NULLS ±${bw.toFixed(1)} kHz`;
    }
    if (m.fr) {
        const r = m.fr, fsd = 2.336e6;
        if (r.det.length) {
            const dtu = 1e6 / fsd;
            const t0 = -r.b * dtu;
            const ymax = Math.max(...r.det), ymin = Math.min(...r.det);
            plot($('p-det'), {
                xr: [t0, t0 + r.det.length * dtu], yr: [Math.floor(ymin / 10) * 10, Math.ceil(ymax / 10) * 10 + 5],
                xl: 'µs from start', yl: 'dB',
                series: [{ y: r.det, x0: t0, dx: dtu, c: CH[0] }],
                hl: [{ y: r.thr }], vl: [{ x: 0, c: '#8fa7c4' }],
            });
            $('r-det').textContent = `THR ${r.thr.toFixed(1)} dB`;
        }
        if (r.theta.length) {
            const th = r.theta, n = th.length;
            const mn = Math.min(...th), mx = Math.max(...th);
            const pad = Math.max(10, (mx - mn) * 0.1);
            // least-squares slope in deg per write -> Hz
            let sx = 0, sy = 0, sxx = 0, sxy = 0;
            for (let i = 0; i < n; i++) { sx += i; sy += th[i]; sxx += i * i; sxy += i * th[i]; }
            const slope = (n * sxy - sx * sy) / Math.max(n * sxx - sx * sx, 1);
            const hz = slope / 360 / (r.T * 1e-6);
            plot($('p-pll'), {
                xr: [0, 1024], yr: [mn - pad, mx + pad], xl: 'write #', yl: 'deg',
                series: [{ y: th, x0: 0, dx: 1, c: CH[0] }],
            });
            $('r-pll').textContent = `SPAN ${(mx - mn).toFixed(0)}°  CFO ${hz >= 0 ? '+' : ''}${hz.toFixed(0)} Hz`;
        }
        for (let k = 0; k < 4; k++)
            constel($(`p-c${k}`), r.mods[k] > 0 ? r.const[k] || [] : [], `EL${k} ${MODS[r.mods[k]]}${r.mods[k] > 0 ? ' ' + f1(r.snr[k]) + ' dB' : ''}`);
        $('r-cons').textContent = `SEQ ${r.seq}`;
    }
}

window.addEventListener('resize', () => { drawH(); if (st) drawSep(st); });
connect();

/* 월드모델 시뮬레이션 — 화면(캔버스), 조작 패널, 코드 설정 편집기 */
(function () {
  'use strict';

  const W = window.WMCore;
  const root = document.getElementById('wm-sim');
  if (!W || !root) return;
  const { Sim, DEFAULTS, LIMITS, MODES, normalize, runHeadless, CONST, laneY } = W;
  const { LANE_W, CAR_L, CAR_W, DT } = CONST;

  const $ = (sel) => root.querySelector(sel);
  const canvas = $('#wm-canvas');
  const ctx = canvas.getContext('2d');
  const paramsEl = $('#wm-params');
  const codeEl = $('#wm-code');
  const codeMsg = $('#wm-code-msg');
  const statsEl = $('#wm-stats');
  const compareEl = $('#wm-compare');
  const playBtn = $('#wm-play');

  const pct = (v) => Math.round(v * 100) + '%';
  const PARAMS = [
    { k: 'horizon', label: '예측 시간', step: 0.5, fmt: (v) => v.toFixed(1) + '초', hint: '몇 초 뒤까지 상상할지' },
    { k: 'samples', label: '상상하는 미래 수', step: 1, fmt: (v) => v + '개', hint: '뽑아 보는 미래 시나리오 개수' },
    { k: 'fidelity', label: '월드모델 정확도', step: 0.05, fmt: pct, hint: '깜빡이·운전 성향을 얼마나 이해하는지' },
    { k: 'sensorNoise', label: '센서 노이즈', step: 0.1, fmt: (v) => v.toFixed(1) + 'm', hint: '위치 측정 오차의 크기' },
    { k: 'riskAversion', label: '위험 회피 성향', step: 0.05, fmt: pct, hint: '0 = 평균적인 미래, 1 = 최악의 미래 기준' },
    { k: 'safetyGap', label: '안전 거리', step: 0.1, fmt: (v) => v.toFixed(1) + '초', hint: '앞차와 유지할 시간 간격' },
    { k: 'targetSpeed', label: '목표 속도', step: 5, fmt: (v) => v + 'km/h' },
    { k: 'density', label: '교통 밀도', step: 1, fmt: (v) => v + '대/km' },
    { k: 'aggressiveness', label: '주변 운전자 공격성', step: 0.05, fmt: pct, hint: '높을수록 좁은 틈에 끼어듦' },
    { k: 'eventRate', label: '돌발 상황 빈도', step: 1, fmt: (v) => v + '회/분' },
    { k: 'lanes', label: '차로 수', step: 1, fmt: (v) => v + '차로', reset: true },
  ];
  const WM_ONLY = new Set(['samples', 'fidelity']);

  let cfg = normalize(DEFAULTS);
  let sim = new Sim(cfg);
  let running = false;
  let visible = false;
  let simSpeed = 1;
  let showPred = true;
  let showPlan = true;
  let last = 0;
  let acc = 0;

  /* ── 조작 패널 만들기 ── */
  const inputs = {};
  for (const p of PARAMS) {
    const [lo, hi] = LIMITS[p.k];
    const row = document.createElement('label');
    row.className = 'wm-param';
    row.dataset.key = p.k;
    row.innerHTML =
      `<span class="wm-param-head"><span>${p.label}</span><output></output></span>` +
      `<input type="range" min="${lo}" max="${hi}" step="${p.step}">` +
      (p.hint ? `<span class="wm-param-hint">${p.hint}</span>` : '');
    const input = row.querySelector('input');
    const out = row.querySelector('output');
    input.addEventListener('input', () => {
      cfg[p.k] = Number(input.value);
      cfg = normalize(cfg);
      out.textContent = p.fmt(cfg[p.k]);
      if (p.reset) restart();
      else { sim.cfg = cfg; sim.onConfigChange(); }
      writeCode();
    });
    paramsEl.appendChild(row);
    inputs[p.k] = { input, out, p, row };
  }

  root.querySelectorAll('[data-mode]').forEach((btn) => {
    btn.addEventListener('click', () => {
      cfg.mode = btn.dataset.mode;
      sim.cfg = cfg;
      sim.viz = null;
      sim.ego.aQueue = [];
      syncControls();
      writeCode();
    });
  });

  const seedInput = $('#wm-seed');
  seedInput.addEventListener('change', () => {
    cfg.seed = Number(seedInput.value);
    cfg = normalize(cfg);
    seedInput.value = cfg.seed;
    restart();
    writeCode();
  });
  $('#wm-seed-rand').addEventListener('click', () => {
    cfg.seed = 1 + Math.floor(Math.random() * 99998);
    seedInput.value = cfg.seed;
    restart();
    writeCode();
  });

  const speedInput = $('#wm-speed');
  const speedOut = $('#wm-speed-out');
  speedInput.addEventListener('input', () => {
    simSpeed = Number(speedInput.value);
    speedOut.textContent = simSpeed + '×';
  });

  playBtn.addEventListener('click', () => setRunning(!running));
  $('#wm-reset').addEventListener('click', () => { restart(); });
  $('#wm-cutin').addEventListener('click', () => { sim.triggerEvent('cutin'); if (!running) setRunning(true); });
  $('#wm-brake').addEventListener('click', () => { sim.triggerEvent('brake'); if (!running) setRunning(true); });
  $('#wm-show-pred').addEventListener('change', (e) => { showPred = e.target.checked; draw(); });
  $('#wm-show-plan').addEventListener('change', (e) => { showPlan = e.target.checked; draw(); });

  function syncControls() {
    for (const { input, out, p, row } of Object.values(inputs)) {
      input.value = cfg[p.k];
      out.textContent = p.fmt(cfg[p.k]);
      const off = (WM_ONLY.has(p.k) && cfg.mode !== 'worldmodel') ||
        (cfg.mode === 'reactive' && ['horizon', 'riskAversion', 'sensorNoise'].includes(p.k));
      row.classList.toggle('is-off', off);
    }
    seedInput.value = cfg.seed;
    root.querySelectorAll('[data-mode]').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.mode === cfg.mode)));
  }

  function setRunning(on) {
    running = on;
    playBtn.textContent = on ? '일시정지' : '시작';
    playBtn.setAttribute('aria-pressed', String(on));
  }

  function restart() {
    sim = new Sim(cfg);
    acc = 0;
    resize();
    draw();
    renderStats(true);
  }

  /* ── 코드로 설정하기 ── */
  const CODE_NOTES = {
    mode: '"worldmodel" | "constvel" | "reactive"',
    horizon: '예측 시간(초) 0.5 ~ 6',
    samples: '상상하는 미래 개수 1 ~ 30',
    fidelity: '월드모델 정확도 0 ~ 1',
    sensorNoise: '센서 위치 오차(m) 0 ~ 2',
    riskAversion: '0 = 평균 기준, 1 = 최악 기준',
    safetyGap: '앞차와의 시간 간격(초) 0.5 ~ 3',
    targetSpeed: '목표 속도(km/h) 40 ~ 130',
    density: '교통 밀도(대/km/차로) 4 ~ 40',
    aggressiveness: '주변 운전자 공격성 0 ~ 1',
    eventRate: '돌발 상황(회/분) 0 ~ 12',
    lanes: '차로 수 2 ~ 4',
    seed: '랜덤 시드 (같은 값 → 같은 교통 상황)',
  };

  function writeCode() {
    const keys = Object.keys(DEFAULTS);
    const lines = keys.map((k, i) => {
      const val = typeof cfg[k] === 'string' ? JSON.stringify(cfg[k]) : String(+cfg[k].toFixed(2));
      const left = `  ${k}: ${val}${i < keys.length - 1 ? ',' : ''}`;
      return left.padEnd(28) + '// ' + CODE_NOTES[k];
    });
    codeEl.value =
      '// 값을 고치고 [코드 실행] (Ctrl+Enter)을 누르면\n' +
      '// 그 설정으로 시뮬레이션이 처음부터 다시 실행됩니다.\n' +
      '{\n' + lines.join('\n') + '\n}';
  }

  function runCode() {
    let parsed;
    try {
      parsed = new Function('"use strict"; return (\n' + codeEl.value + '\n);')();
      if (!parsed || typeof parsed !== 'object') throw new Error('{ ... } 형태의 객체를 만들어 주세요.');
    } catch (err) {
      codeMsg.textContent = '오류: ' + err.message;
      codeMsg.className = 'wm-code-msg is-error';
      return;
    }
    const unknown = Object.keys(parsed).filter((k) => !(k in DEFAULTS));
    cfg = normalize({ ...cfg, ...parsed });
    syncControls();
    restart();
    setRunning(true);
    codeMsg.textContent = unknown.length
      ? `실행했습니다. 알 수 없는 항목은 무시했어요: ${unknown.join(', ')}`
      : '실행했습니다. 범위를 벗어난 값은 자동으로 조정됩니다.';
    codeMsg.className = 'wm-code-msg is-ok';
    writeCode();
  }

  $('#wm-run-code').addEventListener('click', runCode);
  $('#wm-default-code').addEventListener('click', () => {
    cfg = normalize(DEFAULTS);
    syncControls();
    writeCode();
    restart();
    codeMsg.textContent = '기본값으로 되돌렸습니다.';
    codeMsg.className = 'wm-code-msg';
  });
  codeEl.addEventListener('keydown', (e) => {
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); runCode(); }
    else if (e.key === 'Tab') {
      e.preventDefault();
      const s = codeEl.selectionStart;
      codeEl.setRangeText('  ', s, codeEl.selectionEnd, 'end');
    }
  });

  /* ── 통계 ── */
  let statsTimer = 0;
  function renderStats(force) {
    const now = performance.now();
    if (!force && now - statsTimer < 250) return;
    statsTimer = now;
    const s = sim.summary();
    const pred = cfg.mode !== 'reactive';
    const tiles = [
      ['주행 시간', s.time.toFixed(1), '초'],
      ['현재 속도', (sim.ego.v * 3.6).toFixed(0), 'km/h'],
      ['충돌', s.collisions, '회', s.collisions > 0 ? 'bad' : ''],
      ['위기 상황', s.nearMiss, '회', s.nearMiss > 0 ? 'warn' : ''],
      ['평균 속도', s.avgSpeed.toFixed(0), 'km/h'],
      ['차로 변경', pred ? s.laneChanges : '—', pred ? '회' : ''],
      ['예측 오차 (최빈 미래)', pred && s.fde !== null ? s.fde.toFixed(2) : '—', pred && s.fde !== null ? 'm' : ''],
      ['예측 오차 (최선 샘플)', pred && s.minFde !== null ? s.minFde.toFixed(2) : '—', pred && s.minFde !== null ? 'm' : ''],
      ['계획 1회 연산', pred ? s.planMs.toFixed(1) : '—', pred ? 'ms' : ''],
    ];
    statsEl.innerHTML = tiles
      .map(([l, v, u, cls]) => `<div class="wm-stat ${cls ? 'is-' + cls : ''}"><span>${l}</span><strong>${v}<small>${u}</small></strong></div>`)
      .join('');
  }

  /* ── 비교 실험: 같은 설정·시드로 세 방식을 가상 주행 ── */
  $('#wm-compare-btn').addEventListener('click', () => {
    const btn = $('#wm-compare-btn');
    const secs = 120;
    btn.disabled = true;
    compareEl.innerHTML = '<p class="wm-muted">계산 중… (각 방식 120초 주행)</p>';
    const modes = Object.keys(MODES);
    const results = {};
    let i = 0;
    const next = () => {
      if (i >= modes.length) {
        btn.disabled = false;
        renderCompare(results, secs);
        return;
      }
      const m = modes[i++];
      results[m] = runHeadless({ ...cfg, mode: m }, secs);
      setTimeout(next, 20);
    };
    setTimeout(next, 30);
  });

  function renderCompare(res, secs) {
    const modes = Object.keys(res);
    const minOf = (key) => Math.min(...modes.map((m) => res[m][key]));
    const bestCol = minOf('collisions'), bestNear = minOf('nearMiss');
    const bestV = Math.max(...modes.map((m) => res[m].avgSpeed));
    const maxBar = Math.max(1, ...modes.map((m) => res[m].collisions * 3 + res[m].nearMiss));
    compareEl.innerHTML =
      `<table class="wm-table"><thead><tr><th>방식</th><th>충돌</th><th>위기</th><th>평균 속도</th><th>위험 지수</th></tr></thead><tbody>` +
      modes.map((m) => {
        const r = res[m];
        const risk = r.collisions * 3 + r.nearMiss;
        return `<tr${m === cfg.mode ? ' class="is-current"' : ''}><th>${MODES[m]}</th>` +
          `<td class="${r.collisions === bestCol ? 'is-best' : ''}">${r.collisions}</td>` +
          `<td class="${r.nearMiss === bestNear ? 'is-best' : ''}">${r.nearMiss}</td>` +
          `<td class="${r.avgSpeed === bestV ? 'is-best' : ''}">${r.avgSpeed.toFixed(0)} km/h</td>` +
          `<td><span class="wm-bar"><i style="width:${(risk / maxBar) * 100}%"></i></span></td></tr>`;
      }).join('') +
      `</tbody></table><p class="wm-muted">시드 ${cfg.seed} · ${secs}초 · 위험 지수 = 충돌×3 + 위기. 시드를 바꿔 여러 번 해 보세요.</p>`;
  }

  /* ── 그리기 ── */
  let view = { W: 0, H: 0, sx: 1, sy: 1, viewLen: 170, top: 0 };
  function resize() {
    const Wd = Math.max(280, canvas.clientWidth || canvas.parentElement.clientWidth);
    const viewLen = Wd < 640 ? 105 : 170;
    const sx = Wd / viewLen;
    const sy = sx * (Wd < 640 ? 2.6 : 1.9);
    const roadH = sim.cfg.lanes * LANE_W * sy;
    const Hd = Math.round(roadH + 96);
    const dpr = window.devicePixelRatio || 1;
    canvas.style.height = Hd + 'px';
    canvas.width = Math.round(Wd * dpr);
    canvas.height = Math.round(Hd * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    view = { W: Wd, H: Hd, sx, sy, viewLen, top: (Hd - roadH) / 2 };
  }

  const COLORS = {
    ground: '#dfe6e1', road: '#2a3230', edge: '#f3f0e9', dash: 'rgba(243,240,233,0.5)',
    car: '#ece8df', carLine: '#17201f', ego: '#dd573d', glass: '#3a4542',
    pred: '244,185,66', plan: '#57d6ae', cand: 'rgba(255,255,255,0.14)', brake: '#ff4f3a', blink: '#ffc233',
  };

  function draw() {
    const { W: Wd, H: Hd, sx, sy, viewLen, top } = view;
    const e = sim.ego, c = sim.cfg;
    const x0 = e.x - viewLen * 0.28;
    const X = (x) => (x - x0) * sx;
    const Y = (y) => top + y * sy;
    const roadH = c.lanes * LANE_W * sy;

    ctx.fillStyle = COLORS.ground;
    ctx.fillRect(0, 0, Wd, Hd);
    ctx.fillStyle = COLORS.road;
    ctx.fillRect(0, top - 6, Wd, roadH + 12);
    ctx.strokeStyle = COLORS.edge;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, top); ctx.lineTo(Wd, top);
    ctx.moveTo(0, top + roadH); ctx.lineTo(Wd, top + roadH);
    ctx.stroke();
    ctx.strokeStyle = COLORS.dash;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let L = 1; L < c.lanes; L++) {
      const y = Y(L * LANE_W);
      for (let x = Math.floor(x0 / 12) * 12; x < x0 + viewLen; x += 12) { ctx.moveTo(X(x), y); ctx.lineTo(X(x + 4), y); }
    }
    ctx.stroke();

    const viz = sim.viz;
    if (viz && showPlan) {
      ctx.lineWidth = 1;
      ctx.strokeStyle = COLORS.cand;
      for (const cd of viz.cands) polyEgo(cd.traj, viz.nS, X, Y);
    }
    if (viz && showPred) {
      const K = viz.preds.length;
      const alpha = K > 1 ? Math.max(0.1, 0.75 / Math.sqrt(K)) : 0.85;
      ctx.lineWidth = 1.3;
      for (let i = 0; i < viz.obs.length; i++) {
        const o = viz.obs[i];
        for (let k = K - 1; k >= 0; k--) {
          const P = viz.preds[k][i];
          ctx.strokeStyle = `rgba(${COLORS.pred},${k === 0 ? 0.95 : alpha})`;
          ctx.beginPath();
          ctx.moveTo(X(o.x), Y(o.y));
          for (let j = 0; j < viz.nS; j++) ctx.lineTo(X(P[j * 2]), Y(P[j * 2 + 1]));
          ctx.stroke();
          ctx.fillStyle = `rgba(${COLORS.pred},${k === 0 ? 1 : alpha})`;
          ctx.fillRect(X(P[(viz.nS - 1) * 2]) - 1.5, Y(P[(viz.nS - 1) * 2 + 1]) - 1.5, 3, 3);
        }
      }
    }
    if (viz && showPlan && viz.best) {
      ctx.lineWidth = 3;
      ctx.strokeStyle = COLORS.plan;
      polyEgo(viz.best.traj, viz.nS, X, Y);
    }

    const blinkOn = (sim.t * 2.5) % 1 < 0.55;
    for (const ag of sim.agents) {
      const px = X(ag.x);
      if (px < -40 || px > Wd + 40) continue;
      const signalling = ag.pendingLane >= 0 || Math.abs(ag.vy) > 0.05;
      const dir = ag.pendingLane >= 0 ? ag.blinkDir : Math.sign(ag.vy);
      drawCar(px, Y(ag.y), ag.v, ag.vy, COLORS.car, ag.a < -1.5, signalling && blinkOn ? dir : 0);
    }
    drawCar(X(e.x), Y(e.y), e.v, e.vy, COLORS.ego, e.a < -1.5, 0, true);

    if (sim.flash > 0) {
      ctx.fillStyle = `rgba(221,87,61,${sim.flash * 0.55})`;
      ctx.fillRect(0, 0, Wd, Hd);
    }

    // HUD
    ctx.font = '600 12px "Courier New", monospace';
    ctx.fillStyle = '#17201f';
    ctx.textBaseline = 'middle';
    ctx.fillText(`${MODES[c.mode]}  ·  ${(e.v * 3.6).toFixed(0)} km/h  ·  t=${sim.t.toFixed(1)}s`, 12, 18);
    const msg = sim.messages.length ? sim.messages[sim.messages.length - 1] : null;
    if (msg && sim.t - msg.t < 2.2) {
      ctx.font = '700 13px "Malgun Gothic", sans-serif';
      const w = ctx.measureText(msg.text).width + 22;
      const mx = Wd - w - 10;
      ctx.fillStyle = msg.kind === 'bad' ? '#dd573d' : msg.kind === 'warn' ? '#17201f' : '#68716d';
      ctx.fillRect(mx, 7, w, 24);
      ctx.fillStyle = '#f3f0e9';
      ctx.fillText(msg.text, mx + 11, 19.5);
    }
    if (!running && sim.t === 0) {
      ctx.font = '600 12px "Courier New", monospace';
      ctx.fillStyle = '#68716d';
      ctx.fillText('[시작]을 누르세요', 12, Hd - 16);
    }
  }

  function polyEgo(traj, nS, X, Y) {
    const e = sim.ego;
    ctx.beginPath();
    ctx.moveTo(X(e.x), Y(e.y));
    for (let j = 0; j < nS; j++) ctx.lineTo(X(traj[j * 3]), Y(traj[j * 3 + 1]));
    ctx.stroke();
  }

  function drawCar(px, py, v, vy, fill, braking, blinkDir, isEgo) {
    const { sx, sy } = view;
    const L = CAR_L * sx, Wc = CAR_W * sy;
    const ang = Math.max(-0.35, Math.min(0.35, Math.atan2(vy * sy, Math.max(0.5, v) * sx)));
    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(ang);
    ctx.fillStyle = fill;
    ctx.strokeStyle = COLORS.carLine;
    ctx.lineWidth = 1;
    roundRect(-L / 2, -Wc / 2, L, Wc, Math.min(4, Wc / 3));
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = isEgo ? 'rgba(23,32,31,0.55)' : COLORS.glass;
    ctx.fillRect(L * 0.08, -Wc / 2 + 2, L * 0.2, Wc - 4);
    if (braking) {
      ctx.fillStyle = COLORS.brake;
      ctx.fillRect(-L / 2 - 1, -Wc / 2 + 1, 3, Wc * 0.28);
      ctx.fillRect(-L / 2 - 1, Wc / 2 - 1 - Wc * 0.28, 3, Wc * 0.28);
    }
    if (blinkDir) {
      ctx.fillStyle = COLORS.blink;
      const yy = blinkDir > 0 ? Wc / 2 - 3 : -Wc / 2;
      ctx.fillRect(L / 2 - 4, yy, 5, 3);
      ctx.fillRect(-L / 2 - 1, yy, 5, 3);
    }
    ctx.restore();
  }

  function roundRect(x, y, w, h, r) {
    ctx.beginPath();
    ctx.moveTo(x + r, y);
    ctx.arcTo(x + w, y, x + w, y + h, r);
    ctx.arcTo(x + w, y + h, x, y + h, r);
    ctx.arcTo(x, y + h, x, y, r);
    ctx.arcTo(x, y, x + w, y, r);
    ctx.closePath();
  }

  /* ── 루프 ── */
  function frame(now) {
    const dt = Math.min(0.1, (now - last) / 1000 || 0);
    last = now;
    if (running && visible) {
      acc += dt * simSpeed;
      let n = 0;
      while (acc >= DT && n < 40) { sim.step(DT); acc -= DT; n++; }
      if (n === 40) acc = 0;
      draw();
      renderStats(false);
    }
    requestAnimationFrame(frame);
  }

  if ('IntersectionObserver' in window) {
    new IntersectionObserver((entries) => {
      visible = entries[0].isIntersecting;
      if (visible) draw();
    }, { threshold: 0.05 }).observe(canvas);
  } else {
    visible = true;
  }
  if ('ResizeObserver' in window) new ResizeObserver(() => { resize(); draw(); }).observe(canvas.parentElement);
  window.addEventListener('resize', () => { resize(); draw(); });

  // 콘솔에서도 조작 가능: WM.run({ horizon: 5, samples: 20 })
  window.WM = {
    run(partial) { cfg = normalize({ ...cfg, ...partial }); syncControls(); writeCode(); restart(); setRunning(true); return cfg; },
    get sim() { return sim; },
    get config() { return { ...cfg }; },
  };

  syncControls();
  writeCode();
  resize();
  draw();
  renderStats(true);
  requestAnimationFrame((t) => { last = t; frame(t); });
})();

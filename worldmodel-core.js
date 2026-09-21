/*
 * 월드모델 자율주행 시뮬레이션 — 계산 엔진 (화면과 분리되어 있어 Node에서도 실행 가능)
 *
 * 구조
 *  - 주변 차량(agents): IDM 차간거리 모델 + 확률적 차로 변경(깜빡이 → 이동)으로 "실제 세계"를 만든다.
 *  - 자차(ego): 세 가지 방식 중 하나로 운전한다.
 *      worldmodel : 관측 → 주변 차량의 여러 미래를 샘플링(상상) → 후보 행동을 상상 속에서 평가 → 최저 비용 행동 선택
 *      constvel   : 주변 차량이 지금 속도 그대로 직진한다고 가정하는 단순 예측 + 같은 플래너
 *      reactive   : 예측 없이 앞차에만 반응 (반응 지연 0.7초)
 */
(function (root) {
  'use strict';

  const LANE_W = 3.6;          // 차로 폭 (m)
  const CAR_L = 4.6;           // 차 길이 (m)
  const CAR_W = 1.9;           // 차 폭 (m)
  const DT = 0.05;             // 물리 시뮬레이션 간격 (s)
  const PLAN_DT = 0.2;         // 계획(재상상) 주기 (s)
  const PRED_DT = 0.25;        // 상상 속 시간 간격 (s)
  const REACT_DELAY = 0.7;     // 반응형 운전자의 반응 지연 (s)
  const A_MIN = -8, A_MAX = 2.5;
  const V_MAX = 140 / 3.6;
  const EGO_ACCELS = [-7, -4, -2, -1, 0, 1, 2];
  const EGO_LAT_V = 1.6;       // 자차 차로 변경 횡속도 (m/s)
  const RANGE_BACK = 130, RANGE_FRONT = 300;
  const OBS_BACK = 50, OBS_FRONT = 140;

  const MODES = {
    worldmodel: '월드모델',
    constvel: '등속 예측',
    reactive: '예측 없음(반응형)',
  };

  const DEFAULTS = Object.freeze({
    mode: 'worldmodel',
    horizon: 3,
    samples: 12,
    fidelity: 0.8,
    sensorNoise: 0.3,
    riskAversion: 0.5,
    safetyGap: 1.5,
    targetSpeed: 100,
    density: 14,
    aggressiveness: 0.5,
    eventRate: 4,
    lanes: 3,
    seed: 42,
  });

  const LIMITS = {
    horizon: [0.5, 6], samples: [1, 30], fidelity: [0, 1], sensorNoise: [0, 2],
    riskAversion: [0, 1], safetyGap: [0.5, 3], targetSpeed: [40, 130], density: [4, 40],
    aggressiveness: [0, 1], eventRate: [0, 12], lanes: [2, 4], seed: [1, 99999],
  };
  const INTS = new Set(['samples', 'lanes', 'seed']);

  function normalize(input) {
    const out = { ...DEFAULTS };
    if (input) for (const k of Object.keys(DEFAULTS)) if (input[k] !== undefined) out[k] = input[k];
    if (!MODES[out.mode]) out.mode = DEFAULTS.mode;
    for (const [k, [lo, hi]] of Object.entries(LIMITS)) {
      let v = Number(out[k]);
      if (!Number.isFinite(v)) v = DEFAULTS[k];
      v = Math.min(hi, Math.max(lo, v));
      out[k] = INTS.has(k) ? Math.round(v) : v;
    }
    return out;
  }

  const kmh = (v) => v / 3.6;
  const laneY = (i) => (i + 0.5) * LANE_W;
  const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

  function mulberry32(a) {
    return function () {
      a |= 0; a = (a + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), 1 | a);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }
  function gauss(r) {
    let u = 0;
    while (u === 0) u = r();
    return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * r());
  }

  // Intelligent Driver Model: 앞차와의 간격·속도차로 가속도를 정한다.
  function idm(v, v0, lead, T, amax, b) {
    const free = 1 - Math.pow(v / Math.max(v0, 0.1), 4);
    if (!lead) return amax * free;
    const s = Math.max(0.1, lead.gap);
    const ss = 2 + Math.max(0, v * T + (v * (v - lead.v)) / (2 * Math.sqrt(amax * b)));
    return amax * (free - (ss / s) * (ss / s));
  }

  class Sim {
    constructor(cfg) {
      this.cfg = normalize(cfg);
      this.reset();
    }

    reset() {
      const c = this.cfg;
      this.rTraffic = mulberry32(c.seed);
      this.rPlan = mulberry32(c.seed * 7 + 13);
      this.rSense = mulberry32(c.seed * 31 + 5);
      this.t = 0;
      this.agents = [];
      this.nextId = 1;
      const mid = Math.floor(c.lanes / 2);
      this.ego = {
        id: 0, x: 0, y: laneY(mid), v: kmh(c.targetSpeed) * 0.85, vy: 0, a: 0,
        cmdA: 0, targetLane: mid, fromLane: mid, aQueue: [],
      };
      this.stats = {
        collisions: 0, nearMiss: 0, distance: 0, time: 0, laneChanges: 0,
        fdeSum: 0, minFdeSum: 0, fdeN: 0, planMs: 0,
      };
      this.pending = [];
      this.viz = null;
      this.planTimer = 0;
      this.flash = 0;
      this.messages = [];
      this.nearFlag = new Map();
      this._veh = [];
      this.nextEvent = this.drawEventGap();
      this.populate();
    }

    drawEventGap() {
      const rate = this.cfg.eventRate;
      if (rate <= 0) return Infinity;
      return this.t + 4 + (-Math.log(1 - this.rTraffic()) * 60) / rate;
    }

    onConfigChange() {
      // 실시간으로 바뀐 설정 반영 (차로 수·시드는 reset 필요)
      this.nextEvent = this.drawEventGap();
    }

    say(text, kind) {
      this.messages.push({ text, kind: kind || 'info', t: this.t });
      if (this.messages.length > 4) this.messages.shift();
    }

    laneBaseSpeed(L) {
      return 82 + (this.cfg.lanes - 1 - L) * 7;
    }

    makeAgent(x, L, speedKmh) {
      const c = this.cfg, r = this.rTraffic;
      const v0 = kmh(speedKmh !== undefined ? speedKmh : this.laneBaseSpeed(L) + gauss(r) * 6);
      const aggr = clamp(c.aggressiveness + gauss(r) * 0.2, 0, 1);
      const ag = {
        id: this.nextId++, x, y: laneY(L), v: v0, vy: 0, a: 0, v0, v0orig: v0,
        lane: L, pendingLane: -1, blink: 0, blinkDir: 0,
        aggr, T: 1.6 - 0.8 * aggr, latV: 1.0 + 1.6 * aggr,
        cool: 2 + r() * 5, brakeT: 0, slowT: 0,
        perc: r(), // 월드모델이 이 차의 의도를 읽어낼 수 있는지 (perc < 정확도)
      };
      this.agents.push(ag);
      return ag;
    }

    populate() {
      const c = this.cfg, r = this.rTraffic, e = this.ego;
      for (let L = 0; L < c.lanes; L++) {
        let x = -RANGE_BACK + 10 + r() * 20;
        while (x < RANGE_FRONT - 20) {
          const nearEgo = Math.abs(laneY(L) - e.y) < 1 && Math.abs(x - e.x) < 30;
          if (!nearEgo) this.makeAgent(x, L);
          x += Math.max(16, (-Math.log(1 - r()) * 1000) / c.density);
        }
      }
    }

    // yRef 주변 폭(width) 안에서 x보다 앞에 있는 가장 가까운 차
    findLeader(x, yRef, self, width) {
      let best = null;
      for (const o of this._veh) {
        if (o === self || o.x <= x || Math.abs(o.y - yRef) >= width) continue;
        const gap = o.x - x - CAR_L;
        if (!best || gap < best.gap) best = { gap, v: o.v, a: o.a, ref: o };
      }
      return best;
    }

    gapOk(ag, L) {
      const yL = laneY(L);
      for (const o of this._veh) {
        if (o === ag || Math.abs(o.y - yL) >= 2.4) continue;
        const dx = o.x - ag.x;
        if (dx >= 0) {
          if (dx - CAR_L < 3 + 8 * (1 - ag.aggr)) return false;
        } else {
          const closing = Math.max(0, o.v - ag.v);
          if (-dx - CAR_L < 3 + 10 * (1 - ag.aggr) + closing * (0.5 + 1.5 * (1 - ag.aggr))) return false;
        }
      }
      return true;
    }

    stepAgent(ag, dt) {
      const c = this.cfg, r = this.rTraffic;
      let lead = this.findLeader(ag.x, ag.y, ag, 2.0);
      if (Math.abs(ag.y - laneY(ag.lane)) > 0.05) {
        const l2 = this.findLeader(ag.x, laneY(ag.lane), ag, 2.0);
        if (l2 && (!lead || l2.gap < lead.gap)) lead = l2;
      }
      let a = idm(ag.v, ag.v0, lead, ag.T, 1.6, 2.5);
      if (ag.brakeT > 0) { ag.brakeT -= dt; a = Math.min(a, -6.5); }
      if (ag.slowT > 0) { ag.slowT -= dt; if (ag.slowT <= 0) ag.v0 = ag.v0orig; }
      a = clamp(a, A_MIN, A_MAX);
      ag.a = a;
      ag.v = Math.max(0, ag.v + a * dt);
      ag.x += ag.v * dt;

      // 깜빡이 → 차로 이동
      if (ag.blink > 0) {
        ag.blink -= dt;
        if (ag.blink <= 0 && ag.pendingLane >= 0) { ag.lane = ag.pendingLane; ag.pendingLane = -1; }
      }
      const dy = laneY(ag.lane) - ag.y;
      const stepY = ag.latV * dt;
      if (Math.abs(dy) <= stepY) { ag.y += dy; ag.vy = 0; } else { ag.vy = Math.sign(dy) * ag.latV; ag.y += ag.vy * dt; }

      // 차로 변경 결정
      ag.cool -= dt;
      if (ag.cool <= 0 && ag.pendingLane < 0 && Math.abs(dy) < 0.05) {
        const blocked = lead && lead.gap < 45 && lead.v < ag.v0 - 1.5;
        const rate = (0.02 + 0.18 * ag.aggr * ag.aggr) * (blocked ? 4 : 1);
        if (r() < rate * dt) {
          const dir = r() < 0.5 ? -1 : 1;
          let L = ag.lane + dir;
          if (L < 0 || L >= c.lanes) L = ag.lane - dir;
          if (this.gapOk(ag, L)) {
            ag.pendingLane = L;
            ag.blinkDir = Math.sign(L - ag.lane);
            ag.blink = 0.5 + 1.0 * (1 - ag.aggr);
            ag.cool = 4 + r() * 6;
          } else {
            ag.cool = 0.5;
          }
        }
      }
    }

    stepEgo(dt) {
      const c = this.cfg, e = this.ego;
      const vT = kmh(c.targetSpeed);
      let a;
      if (c.mode === 'reactive') {
        // 예측 없음: 이미 내 차로에 들어온 앞차에만 반응, 그것도 0.7초 늦게
        const lead = this.findLeader(e.x, e.y, e, 1.6);
        e.aQueue.push(clamp(idm(e.v, vT, lead, c.safetyGap, 2.0, 3.0), A_MIN, A_MAX));
        const n = Math.round(REACT_DELAY / dt);
        a = e.aQueue.length > n ? e.aQueue.shift() : 0;
        this.viz = null;
      } else {
        this.planTimer -= dt;
        if (this.planTimer <= 0) { this.plan(); this.planTimer += PLAN_DT; }
        const jerk = 18 * dt;
        a = e.a + clamp(e.cmdA - e.a, -jerk, jerk);
      }
      e.a = clamp(a, A_MIN, A_MAX);
      e.v = clamp(e.v + e.a * dt, 0, V_MAX);
      e.x += e.v * dt;
      const dy = laneY(e.targetLane) - e.y;
      const stepY = EGO_LAT_V * dt;
      if (Math.abs(dy) <= stepY) { e.y += dy; e.vy = 0; } else { e.vy = Math.sign(dy) * EGO_LAT_V; e.y += e.vy * dt; }
      this.stats.distance += e.v * dt;
      this.stats.time += dt;
    }

    // ── 1) 관측: 센서는 잡음이 있고, 범위가 제한된다 ──
    observe() {
      const c = this.cfg, e = this.ego, r = this.rSense, s = c.sensorNoise;
      const obs = [];
      for (const ag of this.agents) {
        const dx = ag.x - e.x;
        if (dx < -OBS_BACK || dx > OBS_FRONT) continue;
        const intentSeen = c.mode === 'worldmodel' && ag.perc < c.fidelity;
        obs.push({
          id: ag.id,
          x: ag.x + gauss(r) * s,
          y: ag.y + gauss(r) * s * 0.25,
          v: Math.max(0, ag.v + gauss(r) * s * 0.4),
          a: ag.a + gauss(r) * s * 0.5,
          vy: ag.vy + gauss(r) * s * 0.1,
          blinkDir: intentSeen && ag.pendingLane >= 0 ? ag.blinkDir : 0,
          blinkLeft: Math.max(0, ag.blink),
          // 월드모델이 과거 관측으로 "학습"한 운전 성향 추정치
          aggrEst: c.fidelity * ag.aggr + (1 - c.fidelity) * 0.3,
        });
      }
      return obs;
    }

    // ── 2) 상상: 주변 차량 한 대의 k번째 미래를 롤아웃 ──
    predictAgent(o, k, nS) {
      const c = this.cfg, r = this.rPlan;
      const out = new Float32Array(nS * 2);
      let x = o.x, y = o.y, v = o.v;
      if (c.mode !== 'worldmodel') {
        for (let j = 0; j < nS; j++) { x += v * PRED_DT; out[j * 2] = x; out[j * 2 + 1] = y; }
        return out;
      }
      const f = c.fidelity, H = c.horizon;
      let a0 = o.a;
      if (k > 0) a0 += gauss(r) * (0.25 + 2.0 * (1 - f));
      let targetY = y, startT = 0, latV = 1.0 + 1.6 * o.aggrEst;
      const cur = clamp(Math.round(y / LANE_W - 0.5), 0, c.lanes - 1);
      if (Math.abs(o.vy) > 0.3) {
        // 이미 옆으로 움직이는 중 → 진행 방향의 다음 차로로
        const L = o.vy > 0 ? Math.ceil(y / LANE_W - 0.5) : Math.floor(y / LANE_W - 0.5);
        targetY = laneY(clamp(L, 0, c.lanes - 1));
        latV = Math.abs(o.vy);
      } else if (o.blinkDir !== 0) {
        // 깜빡이를 봤다 → 곧 끼어들 것
        if (k === 0 || r() < 0.9) {
          targetY = laneY(clamp(cur + o.blinkDir, 0, c.lanes - 1));
          startT = Math.max(0, o.blinkLeft + (k > 0 ? gauss(r) * 0.3 : 0));
        }
      } else if (k > 0) {
        // 신호가 없어도 차로를 바꿀 가능성 (학습된 성향 기반)
        const pLC = Math.min(0.6, (0.03 + 0.3 * o.aggrEst * o.aggrEst) * H);
        if (r() < pLC) {
          let L = cur + (r() < 0.5 ? -1 : 1);
          if (L < 0 || L >= c.lanes) L = cur - (L - cur);
          targetY = laneY(L);
          startT = r() * H * 0.7;
        }
      }
      for (let j = 0; j < nS; j++) {
        const t = (j + 1) * PRED_DT;
        const a = a0 * Math.pow(0.8, j);
        v = Math.max(0, v + a * PRED_DT);
        x += v * PRED_DT;
        if (t > startT) {
          const dy = targetY - y, st = latV * PRED_DT;
          y += Math.abs(dy) <= st ? dy : Math.sign(dy) * st;
        }
        out[j * 2] = x;
        out[j * 2 + 1] = y;
      }
      return out;
    }

    rollEgo(acc, L, nS) {
      const e = this.ego;
      const out = new Float32Array(nS * 3);
      let x = e.x, y = e.y, v = e.v;
      const ty = laneY(L);
      for (let j = 0; j < nS; j++) {
        const t = (j + 1) * PRED_DT;
        v = clamp(v + (t <= 1.5 ? acc : 0) * PRED_DT, 0, V_MAX);
        x += v * PRED_DT;
        const dy = ty - y, st = EGO_LAT_V * PRED_DT;
        y += Math.abs(dy) <= st ? dy : Math.sign(dy) * st;
        out[j * 3] = x; out[j * 3 + 1] = y; out[j * 3 + 2] = v;
      }
      return out;
    }

    // ── 3) 평가: 후보 행동을 상상한 모든 미래에 넣어 비용 계산 ──
    evaluate(traj, acc, L, preds, obs, nS) {
      const c = this.cfg, e = this.ego;
      const vT = kmh(c.targetSpeed);
      let base = 0.05 * acc * acc + 0.04 * (acc - e.cmdA) * (acc - e.cmdA) + (L !== e.targetLane ? 0.4 : 0);
      for (let j = 0; j < nS; j++) {
        const d = (vT - traj[j * 3 + 2]) / vT;
        base += 20 * Math.pow(0.95, j) * d * d * PRED_DT;
      }
      let sum = 0, worst = 0;
      for (let k = 0; k < preds.length; k++) {
        const P = preds[k];
        let sc = 0;
        for (let j = 0; j < nS; j++) {
          const w = Math.pow(0.95, j);
          const ex = traj[j * 3], ey = traj[j * 3 + 1], ev = traj[j * 3 + 2];
          let hit = false;
          for (let i = 0; i < P.length; i++) {
            const px = P[i][j * 2], py = P[i][j * 2 + 1];
            if (Math.abs(py - ey) >= 2.1) continue;
            const dx = px - ex, gap = Math.abs(dx) - CAR_L;
            if (dx >= 0) {
              if (gap < 0.3) { sc += 800 * w; hit = true; }
              else {
                const des = Math.max(3, c.safetyGap * ev);
                if (gap < des) sc += 60 * w * ((des - gap) / des) ** 2 * PRED_DT;
              }
            } else {
              if (gap < 0.3) { sc += 150 * w; hit = true; }
              else {
                const des = Math.max(3, 0.5 * obs[i].v);
                if (gap < des) sc += 10 * w * ((des - gap) / des) ** 2 * PRED_DT;
              }
            }
          }
          if (hit) break;
        }
        sum += sc;
        if (sc > worst) worst = sc;
      }
      const beta = c.riskAversion;
      return base + (1 - beta) * (sum / preds.length) + beta * worst;
    }

    // ── 4) 계획: 관측 → 상상 → 평가 → 선택 (모델 예측 제어, MPC) ──
    plan() {
      const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
      const c = this.cfg, e = this.ego;
      const nS = Math.max(1, Math.ceil(c.horizon / PRED_DT));
      const obs = this.observe();
      const K = c.mode === 'worldmodel' ? c.samples : 1;
      const preds = [];
      for (let k = 0; k < K; k++) preds.push(obs.map((o) => this.predictAgent(o, k, nS)));

      const centered = Math.abs(e.y - laneY(e.targetLane)) < 0.4;
      const opts = centered ? [e.targetLane, e.targetLane - 1, e.targetLane + 1] : [e.targetLane, e.fromLane];
      const lanes = [...new Set(opts)].filter((L) => L >= 0 && L < c.lanes);
      const cands = [];
      let best = null;
      for (const L of lanes) {
        for (const acc of EGO_ACCELS) {
          const traj = this.rollEgo(acc, L, nS);
          const cost = this.evaluate(traj, acc, L, preds, obs, nS);
          const cand = { acc, L, traj, cost };
          cands.push(cand);
          if (!best || cost < best.cost) best = cand;
        }
      }
      if (best.L !== e.targetLane) {
        if (centered) this.stats.laneChanges++;
        e.fromLane = e.targetLane;
        e.targetLane = best.L;
      }
      e.cmdA = best.acc;

      // 예측 정확도 채점을 위해 "가장 그럴듯한 미래(k=0)"와 모든 샘플의 끝점을 저장
      const due = this.t + nS * PRED_DT;
      obs.forEach((o, i) => {
        const ends = new Float32Array(K * 2);
        for (let k = 0; k < K; k++) { ends[k * 2] = preds[k][i][(nS - 1) * 2]; ends[k * 2 + 1] = preds[k][i][(nS - 1) * 2 + 1]; }
        this.pending.push({ due, id: o.id, ends });
      });

      this.viz = { obs, preds, cands, best, nS };
      const t1 = typeof performance !== 'undefined' ? performance.now() : Date.now();
      this.stats.planMs = this.stats.planMs * 0.9 + (t1 - t0) * 0.1;
    }

    checkPredictions() {
      if (!this.pending.length) return;
      const keep = [];
      for (const p of this.pending) {
        if (p.due > this.t + 1e-9) { keep.push(p); continue; }
        const ag = this.agents.find((a) => a.id === p.id);
        if (!ag) continue;
        let minErr = Infinity;
        for (let k = 0; k < p.ends.length / 2; k++) {
          const err = Math.hypot(ag.x - p.ends[k * 2], ag.y - p.ends[k * 2 + 1]);
          if (k === 0) this.stats.fdeSum += err;
          if (err < minErr) minErr = err;
        }
        this.stats.minFdeSum += minErr;
        this.stats.fdeN++;
      }
      this.pending = keep;
    }

    checkSafety() {
      const e = this.ego;
      for (let i = this.agents.length - 1; i >= 0; i--) {
        const ag = this.agents[i];
        const dx = ag.x - e.x, dy = Math.abs(ag.y - e.y);
        if (Math.abs(dx) < CAR_L && dy < CAR_W) {
          this.stats.collisions++;
          this.flash = 0.6;
          this.say('충돌 발생!', 'bad');
          this.agents.splice(i, 1);
          e.v *= 0.5;
          e.a = 0;
          continue;
        }
        if (dx > 0 && dy < CAR_W + 0.4) {
          const gap = dx - CAR_L, closing = e.v - ag.v;
          const ttc = closing > 0.1 ? gap / closing : Infinity;
          if (gap < 3 || ttc < 1.2) {
            const last = this.nearFlag.get(ag.id);
            if (last === undefined || this.t - last > 4) {
              this.stats.nearMiss++;
              this.say('위기 상황 (충돌 1.2초 전 이내)', 'warn');
            }
            this.nearFlag.set(ag.id, this.t);
          }
        }
      }
    }

    // 주변 차량끼리 겹치면 (드묾) 뒤차를 제거
    resolveAgentOverlaps() {
      const A = this.agents;
      for (let i = A.length - 1; i >= 0; i--) {
        for (let j = 0; j < A.length; j++) {
          if (i === j) continue;
          if (Math.abs(A[i].x - A[j].x) < CAR_L * 0.8 && Math.abs(A[i].y - A[j].y) < CAR_W * 0.8 && A[i].x <= A[j].x) {
            A.splice(i, 1);
            break;
          }
        }
      }
    }

    maintain() {
      const c = this.cfg, e = this.ego, r = this.rTraffic;
      this.agents = this.agents.filter((ag) => ag.x - e.x > -RANGE_BACK && ag.x - e.x < RANGE_FRONT);
      const target = (c.density * (RANGE_BACK + RANGE_FRONT)) / 1000;
      for (let L = 0; L < c.lanes; L++) {
        const inLane = this.agents.filter((ag) => Math.abs(ag.y - laneY(L)) < 1.8);
        if (inLane.length >= target) continue;
        const base = kmh(this.laneBaseSpeed(L));
        const spots = base > e.v + 1 && r() < 0.5 ? [e.x - RANGE_BACK + 8, e.x + RANGE_FRONT - 12] : [e.x + RANGE_FRONT - 12, e.x - RANGE_BACK + 8];
        for (const x of spots) {
          if (inLane.every((ag) => Math.abs(ag.x - x) > 35)) { this.makeAgent(x, L); break; }
        }
      }
    }

    // ── 돌발 상황 ──
    triggerEvent(type, auto) {
      const c = this.cfg, e = this.ego, r = this.rTraffic;
      const egoLane = e.targetLane;
      if (type === 'cutin') {
        const sides = [egoLane - 1, egoLane + 1].filter((L) => L >= 0 && L < c.lanes);
        let pick = null;
        for (const ag of this.agents) {
          const dx = ag.x - e.x;
          if (dx < 3 || dx > 22 || ag.pendingLane >= 0) continue;
          if (!sides.some((L) => Math.abs(ag.y - laneY(L)) < 0.3)) continue;
          if (!pick || Math.abs(dx - 10) < Math.abs(pick.x - e.x - 10)) pick = ag;
        }
        if (!pick) {
          const L = sides[Math.floor(r() * sides.length)];
          const x = e.x + 9 + r() * 8;
          this.agents = this.agents.filter((ag) => !(Math.abs(ag.y - laneY(L)) < 2 && Math.abs(ag.x - x) < 14));
          pick = this.makeAgent(x, L, (e.v * 3.6) * 0.9);
          pick.v = e.v * 0.9;
        }
        pick.pendingLane = egoLane;
        pick.blinkDir = Math.sign(egoLane - Math.round(pick.y / LANE_W - 0.5));
        pick.blink = 0.6;
        pick.latV = 2.4;
        pick.cool = 12;
        pick.v0 = Math.min(pick.v0, e.v * 0.85);
        pick.slowT = 6;
        this.say(auto ? '돌발: 옆 차가 끼어듭니다' : '끼어들기 발생!', 'warn');
      } else {
        let lead = null;
        for (const ag of this.agents) {
          const dx = ag.x - e.x;
          if (dx > 0 && dx < 90 && Math.abs(ag.y - laneY(egoLane)) < 1.2 && (!lead || dx < lead.x - e.x)) lead = ag;
        }
        if (!lead) {
          lead = this.makeAgent(e.x + 32, egoLane, e.v * 3.6);
          lead.v = e.v;
        }
        lead.brakeT = 1.8;
        lead.v0 = lead.v * 0.35;
        lead.slowT = 6;
        this.say(auto ? '돌발: 앞차 급정거' : '앞차 급정거!', 'warn');
      }
    }

    step(dt = DT) {
      this.t += dt;
      this._veh = this.agents.concat([this.ego]);
      for (const ag of this.agents) this.stepAgent(ag, dt);
      this.stepEgo(dt);
      this.checkSafety();
      this.resolveAgentOverlaps();
      this.maintain();
      this.checkPredictions();
      if (this.t >= this.nextEvent) {
        this.triggerEvent(this.rTraffic() < 0.5 ? 'cutin' : 'brake', true);
        this.nextEvent = this.drawEventGap();
      }
      this.flash = Math.max(0, this.flash - dt);
    }

    summary() {
      const s = this.stats;
      return {
        time: s.time,
        collisions: s.collisions,
        nearMiss: s.nearMiss,
        avgSpeed: s.time > 0 ? (s.distance / s.time) * 3.6 : 0,
        laneChanges: s.laneChanges,
        fde: s.fdeN ? s.fdeSum / s.fdeN : null,
        minFde: s.fdeN ? s.minFdeSum / s.fdeN : null,
        planMs: s.planMs,
      };
    }
  }

  function runHeadless(cfg, seconds) {
    const sim = new Sim(cfg);
    const n = Math.round(seconds / DT);
    for (let i = 0; i < n; i++) sim.step(DT);
    return sim.summary();
  }

  const api = {
    Sim, DEFAULTS, LIMITS, MODES, normalize, runHeadless,
    CONST: { LANE_W, CAR_L, CAR_W, DT, PLAN_DT, PRED_DT, REACT_DELAY },
    laneY,
  };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.WMCore = api;
})(typeof window !== 'undefined' ? window : globalThis);

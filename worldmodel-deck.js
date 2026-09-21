/* 발표 모드: 슬라이드 이동, 5분 타이머, 대본 보기 */
(function () {
  'use strict';

  const slides = Array.from(document.querySelectorAll('.slide'));
  const countEl = document.getElementById('deck-count');
  const progressEl = document.getElementById('deck-progress');
  const timeEl = document.getElementById('deck-time');
  const timerBtn = document.getElementById('deck-timer');
  const notesBtn = document.getElementById('deck-notes');
  const TOTAL = 300;
  let current = 0;
  let elapsed = 0;
  let timerOn = false;
  let tick = null;

  const pad = (n) => String(n).padStart(2, '0');
  const fmt = (s) => `${Math.floor(s / 60)}:${pad(Math.floor(s % 60))}`;

  function render() {
    countEl.textContent = `${pad(current + 1)} / ${pad(slides.length)}`;
    progressEl.style.width = ((current + 1) / slides.length) * 100 + '%';
  }

  let lockUntil = 0;
  function go(i) {
    current = Math.max(0, Math.min(slides.length - 1, i));
    lockUntil = Date.now() + 900; // 부드러운 스크롤 도중에는 위치로 다시 계산하지 않음
    slides[current].scrollIntoView({ behavior: 'smooth', block: 'start' });
    render();
  }

  // 화면 위쪽 35% 지점에 걸친 슬라이드를 현재 슬라이드로 본다 (긴 슬라이드도 정확히)
  let ticking = false;
  function syncCurrent() {
    ticking = false;
    if (Date.now() < lockUntil) return;
    const line = window.innerHeight * 0.35;
    let idx = 0;
    slides.forEach((s, i) => { if (s.getBoundingClientRect().top <= line) idx = i; });
    if (idx !== current) { current = idx; render(); }
  }
  window.addEventListener('scroll', () => {
    if (!ticking) { ticking = true; requestAnimationFrame(syncCurrent); }
  }, { passive: true });

  function renderTime() {
    timeEl.textContent = fmt(elapsed);
    timerBtn.classList.toggle('is-on', timerOn);
    timerBtn.classList.toggle('is-over', elapsed > TOTAL);
    // 현재 시간에 맞는 슬라이드를 표시
    let due = 0;
    slides.forEach((s, i) => { if (elapsed >= Number(s.dataset.start || 0)) due = i; });
    slides.forEach((s, i) => s.classList.toggle('is-due', timerOn && i === due));
  }

  function toggleTimer(reset) {
    if (reset) { elapsed = 0; timerOn = false; }
    else timerOn = !timerOn;
    clearInterval(tick);
    if (timerOn) tick = setInterval(() => { elapsed++; renderTime(); }, 1000);
    renderTime();
  }

  function toggleNotes() {
    const on = document.body.classList.toggle('show-notes');
    notesBtn.setAttribute('aria-pressed', String(on));
  }

  document.getElementById('deck-prev').addEventListener('click', () => go(current - 1));
  document.getElementById('deck-next').addEventListener('click', () => go(current + 1));
  timerBtn.addEventListener('click', () => toggleTimer(false));
  timerBtn.addEventListener('dblclick', () => toggleTimer(true));
  notesBtn.addEventListener('click', toggleNotes);

  document.addEventListener('keydown', (e) => {
    const tag = (e.target.tagName || '').toLowerCase();
    if (['input', 'textarea', 'select'].includes(tag) || e.target.isContentEditable) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return;
    switch (e.key) {
      case 'ArrowRight': case 'PageDown': e.preventDefault(); go(current + 1); break;
      case 'ArrowLeft': case 'PageUp': e.preventDefault(); go(current - 1); break;
      case 'Home': e.preventDefault(); go(0); break;
      case 'End': e.preventDefault(); go(slides.length - 1); break;
      case 'n': case 'N': toggleNotes(); break;
      case 't': case 'T': toggleTimer(false); break;
      case 'r': case 'R': toggleTimer(true); break;
      case 'f': case 'F':
        if (!document.fullscreenElement) document.documentElement.requestFullscreen?.();
        else document.exitFullscreen?.();
        break;
    }
  });

  render();
  renderTime();
})();

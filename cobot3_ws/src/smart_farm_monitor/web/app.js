// 관제 화면 공통 도우미. 프레임워크 없이 순수 자바스크립트로 쓴다.

const PAGES = [
  ['/', '대시보드'],
  ['/history', '사이클 이력'],
  ['/inspection', '검사 결과'],
  ['/dock', '도킹 품질'],
  ['/pallet', '트레이 추적'],
];

function renderNav(current) {
  const nav = document.getElementById('nav');
  if (!nav) return;
  nav.innerHTML = PAGES.map(
    ([href, name]) =>
      `<a href="${href}" class="${href === current ? 'on' : ''}">${name}</a>`
  ).join('');
}

async function getJson(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(url + ' -> ' + response.status);
  return response.json();
}

async function postJson(url, body) {
  const response = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body || {}),
  });
  if (!response.ok) throw new Error(url + ' -> ' + response.status);
  return response.json();
}

// DB 가 바뀔 때 서버가 밀어 주는 상태를 받는다. 끊기면 3초 뒤 다시 붙는다.
function subscribeState(onState) {
  const mark = document.getElementById('link-state');
  let source = null;

  function connect() {
    source = new EventSource('/api/events');
    source.onopen = () => {
      if (mark) {
        mark.textContent = '실시간 연결됨';
        mark.className = 'link-state live';
      }
    };
    source.onmessage = (event) => {
      try {
        onState(JSON.parse(event.data));
      } catch (error) {
        console.error('상태 파싱 실패', error);
      }
    };
    source.onerror = () => {
      if (mark) {
        mark.textContent = '연결 끊김 — 다시 시도 중';
        mark.className = 'link-state down';
      }
      source.close();
      setTimeout(connect, 3000);
    };
  }
  connect();
}

function seconds(value) {
  if (value === null || value === undefined) return '—';
  const number = Number(value);
  if (!isFinite(number)) return '—';
  if (number < 60) return number.toFixed(1) + ' s';
  const minutes = Math.floor(number / 60);
  return `${minutes}m ${(number - minutes * 60).toFixed(0)}s`;
}

function meters(value, digits = 3) {
  if (value === null || value === undefined) return '—';
  return Number(value).toFixed(digits) + ' m';
}

function degrees(value) {
  if (value === null || value === undefined) return '—';
  return Number(value).toFixed(2) + '°';
}

function wallTime(value) {
  if (!value) return '—';
  const date = new Date(Number(value) * 1000);
  return date.toLocaleString('ko-KR', { hour12: false });
}

function percent(part, total) {
  if (!total) return '—';
  return ((Number(part) / Number(total)) * 100).toFixed(0) + ' %';
}

function text(value, fallback = '—') {
  return value === null || value === undefined || value === '' ? fallback : value;
}

function tag(value) {
  const safe = text(value, 'none');
  return `<span class="tag ${safe}">${safe}</span>`;
}

// map 좌표(m) -> 지도 이미지 pixel. 이미지 원점이 좌상단이라 y 를 뒤집는다.
function mapToPixel(meta, x, y) {
  return {
    px: (x - meta.origin_x) / meta.resolution,
    py: meta.height_px - (y - meta.origin_y) / meta.resolution,
  };
}

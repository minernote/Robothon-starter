from __future__ import annotations

import html
import json
from pathlib import Path


def write_trace_html(path: Path, trace: list[dict], metrics: dict) -> None:
    """写出轻量级轨迹可视化，便于评审直接查看。"""
    data = json.dumps(trace, ensure_ascii=False)
    metrics_json = json.dumps(metrics, indent=2, ensure_ascii=False)
    doc = f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ChargeMate Trace</title>
  <style>
    body {{ margin: 0; background: #111317; color: #e7ecef; font-family: ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }}
    main {{ max-width: 1080px; margin: 0 auto; padding: 28px; }}
    h1 {{ font-size: 28px; margin: 0 0 6px; }}
    .sub {{ color: #9fb0bd; margin-bottom: 22px; }}
    .grid {{ display: grid; grid-template-columns: 1fr 320px; gap: 18px; align-items: start; }}
    canvas {{ width: 100%; height: auto; background: #171b21; border: 1px solid #2a3038; }}
    pre {{ background: #171b21; border: 1px solid #2a3038; padding: 14px; overflow: auto; font-size: 12px; }}
    .row {{ display: flex; gap: 10px; margin: 12px 0; align-items: center; }}
    button {{ background: #2f81f7; color: white; border: 0; padding: 8px 12px; border-radius: 6px; cursor: pointer; }}
    input {{ width: 100%; }}
    .badge {{ display: inline-block; background: #203a2a; color: #8ff0a4; border: 1px solid #2f6f43; padding: 4px 8px; border-radius: 999px; font-size: 12px; }}
  </style>
</head>
<body>
<main>
  <h1>ChargeMate: Closed-Loop EV Charging Benchmark</h1>
  <div class="sub">Trace visualization generated from the same run that writes dataset labels and metrics.</div>
  <div class="grid">
    <section>
      <canvas id="view" width="900" height="520"></canvas>
      <div class="row">
        <button id="play">Play</button>
        <input id="scrub" type="range" min="0" max="0" value="0">
      </div>
      <div id="status" class="badge"></div>
    </section>
    <aside>
      <h2>Metrics</h2>
      <pre>{html.escape(metrics_json)}</pre>
      <h2>Frame</h2>
      <pre id="frame"></pre>
    </aside>
  </div>
</main>
<script>
const trace = {data};
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
const scrub = document.getElementById('scrub');
const statusEl = document.getElementById('status');
const frameEl = document.getElementById('frame');
const playBtn = document.getElementById('play');
let idx = 0;
let timer = null;
scrub.max = Math.max(0, trace.length - 1);

function sx(x) {{ return 90 + x * 620; }}
function sy(y) {{ return 260 - y * 260; }}
function draw(i) {{
  const row = trace[i] || trace[0];
  if (!row) return;
  const tip = row.tip;
  const port = row.port;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#171b21';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = '#2b313a';
  ctx.fillRect(sx(1.0), 70, 82, 360);
  ctx.fillStyle = '#121417';
  ctx.beginPath();
  ctx.arc(sx(port[0]), sy(port[1]), 36, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#4ade80';
  ctx.lineWidth = 3;
  ctx.stroke();

  ctx.strokeStyle = '#6b7280';
  ctx.lineWidth = 7;
  ctx.beginPath();
  ctx.moveTo(sx(-0.55), sy(0));
  ctx.lineTo(sx(tip[0]), sy(tip[1]));
  ctx.stroke();

  ctx.fillStyle = '#60a5fa';
  ctx.fillRect(sx(-0.55) - 28, sy(0) - 18, 56, 36);
  ctx.fillStyle = '#fbbf24';
  ctx.beginPath();
  ctx.arc(sx(tip[0]), sy(tip[1]), 13, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 5;
  ctx.beginPath();
  ctx.moveTo(sx(tip[0]), sy(tip[1]));
  ctx.lineTo(sx(tip[0]) + 42, sy(tip[1]));
  ctx.stroke();

  ctx.fillStyle = '#e5e7eb';
  ctx.font = '16px ui-monospace, Menlo, monospace';
  ctx.fillText(`stage: ${{row.stage}}`, 24, 34);
  ctx.fillText(`t=${{row.t}}s  force=${{row.contact_force_n}}N  door=${{row.door_angle_rad}}rad`, 24, 58);
  statusEl.textContent = `${{row.stage}} · t=${{row.t}}s · contact=${{row.contact_force_n}}N`;
  frameEl.textContent = JSON.stringify(row, null, 2);
  scrub.value = i;
}}

scrub.addEventListener('input', () => {{ idx = Number(scrub.value); draw(idx); }});
playBtn.addEventListener('click', () => {{
  if (timer) {{ clearInterval(timer); timer = null; playBtn.textContent = 'Play'; return; }}
  playBtn.textContent = 'Pause';
  timer = setInterval(() => {{
    idx = (idx + 1) % trace.length;
    draw(idx);
  }}, 40);
}});
draw(0);
</script>
</body>
</html>
"""
    path.write_text(doc, encoding="utf-8")

#!/usr/bin/env python3
"""Convert telemetry CSV or legacy logs to a self-contained interactive HTML plot."""

import argparse
import csv
from datetime import datetime
import html
import io
import json
import math
from pathlib import Path
import re


NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
FRICTION = re.compile(rf"set:\s*({NUMBER}),\s*left:\s*({NUMBER}),\s*right:\s*({NUMBER})")
STAMPED = re.compile(rf"^\[([^\]]+)\]\s+([^:]+):\s*({NUMBER})\s*$")


def finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("nonfinite log value")
    return result


def read_samples(text, *, sample_period=None, columns=None, max_samples=100000):
    """Return normalized (time_s, name, value); never infer headerless units."""
    lines = [line.strip() for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]
    if not lines:
        raise ValueError("log contains no samples")
    samples = []

    def append(timestamp, name, value):
        if len(samples) >= max_samples:
            raise ValueError("sample limit exceeded; split the input or raise --max-samples")
        name = name.strip()
        if not name or any(ord(char) < 32 for char in name):
            raise ValueError("invalid variable name")
        samples.append((finite(timestamp), name, finite(value)))

    if FRICTION.search(lines[0]):
        if sample_period is None or not math.isfinite(sample_period) or sample_period <= 0:
            raise ValueError("legacy friction logs require explicit --sample-period in seconds")
        for index, line in enumerate(lines):
            match = FRICTION.search(line)
            if not match:
                raise ValueError(f"invalid friction sample at line {index + 1}")
            for name, value in zip(("set", "left", "right"), match.groups()):
                append(index * sample_period, name, value)
    elif lines[0].startswith("["):
        origin = None
        for index, line in enumerate(lines):
            match = STAMPED.fullmatch(line)
            if not match:
                raise ValueError(f"invalid timestamped sample at line {index + 1}")
            stamp = datetime.fromisoformat(match[1])
            if origin is None:
                origin = stamp
            append((stamp - origin).total_seconds(), match[2], match[3])
    else:
        reader = csv.DictReader(io.StringIO("\n".join(lines)), fieldnames=columns)
        fields = reader.fieldnames or []
        if not fields or len(set(fields)) != len(fields) or not any(key in fields for key in ("time_s", "time_ms")):
            raise ValueError("CSV needs time_s/time_ms header; old CSV needs --columns time_ms,command,feedback")
        time_key = "time_s" if "time_s" in fields else "time_ms"
        long_format = fields == [time_key, "name", "value"]
        if len(fields) < 2 or ("time_s" in fields and "time_ms" in fields):
            raise ValueError("ambiguous CSV columns")
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise ValueError("inconsistent CSV row width")
            timestamp = finite(row[time_key]) / (1000.0 if time_key == "time_ms" else 1.0)
            if long_format:
                append(timestamp, row["name"], row["value"])
            else:
                for name in fields:
                    if name != time_key:
                        append(timestamp, name, row[name])
    if not samples:
        raise ValueError("log contains no numeric samples")
    return samples


PAGE = r'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title><style>
body{margin:0;background:#f2f5f7;color:#172d3c;font:15px system-ui,sans-serif}main{max-width:1100px;margin:auto;padding:30px}
h1{font-size:25px;margin:0 0 8px}p{color:#526b7b}section{background:white;padding:22px;border-radius:12px;margin-top:18px}
svg{width:100%;height:auto;display:block}label{margin-right:18px;white-space:nowrap;display:inline-block;padding:5px}
input[type=range]{width:65%;vertical-align:middle}button{padding:6px 15px;border:1px solid #a1b1bc;border-radius:5px;background:white}
.controls{display:flex;gap:20px;align-items:center;flex-wrap:wrap}output{font-variant-numeric:tabular-nums}
</style><main><h1>__TITLE__</h1><p>Read-only telemetry · time in seconds · signs are preserved unless explicitly inverted.</p>
<section><div id="series"></div><svg viewBox="0 0 1040 460" role="img" aria-label="Telemetry time series"></svg>
<div class="controls"><label>Window <input id="window" type="range" min="1" max="100" value="100"> <output id="windowValue">100%</output></label>
<label>Position <input id="position" type="range" min="0" max="1000" value="0"></label><button id="reset">Reset</button></div>
<p id="details"></p></section></main><script>
const data=__DATA__, colors=['#137c8b','#d56a27','#7355a2','#41834b','#b14362','#607586'];
const names=[...new Set(data.map(p=>p[1]))], active=new Set(names);
const controls=document.querySelector('#series'),svg=document.querySelector('svg');
names.forEach((name,index)=>{const label=document.createElement('label'),check=document.createElement('input');check.type='checkbox';check.checked=true;
label.style.color=colors[index%colors.length];label.append(check,document.createTextNode(' '+name));controls.append(label);
check.addEventListener('change',()=>{check.checked?active.add(name):active.delete(name);draw()})});
const windowSlider=document.querySelector('#window'),positionSlider=document.querySelector('#position');
const allTimes=data.map(p=>p[0]), minTime=allTimes.reduce((a,b)=>Math.min(a,b)),maxTime=allTimes.reduce((a,b)=>Math.max(a,b));
function node(tag,attrs,text){const element=document.createElementNS('http://www.w3.org/2000/svg',tag);Object.entries(attrs).forEach(([key,val])=>element.setAttribute(key,val));if(text!==undefined)element.textContent=text;svg.append(element);return element}
function draw(){svg.replaceChildren();const full=Math.max(maxTime-minTime,0.001),span=full*windowSlider.value/100,start=minTime+(full-span)*positionSlider.value/1000,end=start+span;
const visible=data.filter(p=>active.has(p[1])&&p[0]>=start&&p[0]<=end),values=visible.map(p=>p[2]);
let low=values.length?values.reduce((a,b)=>Math.min(a,b)):0,high=values.length?values.reduce((a,b)=>Math.max(a,b)):1;
const pad=Math.max((high-low)*0.08,Math.abs(high)*0.01,0.001);low-=pad;high+=pad;
const x=t=>75+(t-start)/span*940,y=v=>410-(v-low)/(high-low)*370;
for(let i=0;i<=5;i++){const t=start+span*i/5,v=low+(high-low)*i/5;node('line',{x1:75,x2:1015,y1:y(v),y2:y(v),stroke:'#e0e7ec'});
node('text',{x:65,y:y(v)+4,'text-anchor':'end',fill:'#526b7b','font-size':12},v.toPrecision(4));
node('text',{x:x(t),y:435,'text-anchor':'middle',fill:'#526b7b','font-size':12},t.toPrecision(4))}
names.forEach((name,index)=>{const points=visible.filter(p=>p[1]===name).sort((a,b)=>a[0]-b[0]);if(!points.length)return;
node('polyline',{points:points.map(p=>x(p[0])+','+y(p[2])).join(' '),fill:'none',stroke:colors[index%colors.length],'stroke-width':2});
if(points.length===1)node('circle',{cx:x(points[0][0]),cy:y(points[0][2]),r:3,fill:colors[index%colors.length]})});
document.querySelector('#windowValue').textContent=windowSlider.value+'%';
document.querySelector('#details').textContent=`${visible.length} visible samples / ${data.length} total · ${start.toFixed(4)}–${end.toFixed(4)} s`}
windowSlider.addEventListener('input',draw);positionSlider.addEventListener('input',draw);
document.querySelector('#reset').addEventListener('click',()=>{windowSlider.value=100;positionSlider.value=0;draw()});draw();
</script></html>'''


def render_html(samples, title):
    # Escape '<' in JSON so a log name cannot terminate the script element.
    data = json.dumps(samples, ensure_ascii=True, allow_nan=False).replace("<", "\\u003c")
    return PAGE.replace("__TITLE__", html.escape(title)).replace("__DATA__", data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-period", type=float)
    parser.add_argument("--columns", help="explicit old CSV header, e.g. time_ms,command,feedback")
    parser.add_argument("--invert", action="append", default=[], help="explicitly negate this series (repeatable)")
    parser.add_argument("--max-samples", type=int, default=100000)
    args = parser.parse_args()
    if args.input.resolve() == args.output.resolve():
        parser.error("input and output must differ; input logs are read-only")
    try:
        samples = read_samples(args.input.read_text(encoding="utf-8"), sample_period=args.sample_period,
                               columns=args.columns.split(",") if args.columns else None, max_samples=args.max_samples)
        missing = set(args.invert) - {name for _, name, _ in samples}
        if missing:
            raise ValueError(f"unknown inversion series: {', '.join(sorted(missing))}")
        samples = [(time_s, name + " (inverted)" if name in args.invert else name,
                    -value if name in args.invert else value) for time_s, name, value in samples]
        with args.output.open("x", encoding="utf-8") as output:
            output.write(render_html(samples, args.input.name))
    except (OSError, ValueError) as error:
        parser.exit(1, f"plot failed: {error}\n")
    print(args.output.resolve())


if __name__ == "__main__":
    main()

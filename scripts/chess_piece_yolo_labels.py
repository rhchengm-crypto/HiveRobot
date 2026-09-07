#!/usr/bin/env python3
"""Review actual YOLO labels on original images; shared by the live web server."""
from __future__ import annotations

import hashlib
import json
import math
import mimetypes
import os
import threading
import uuid
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from chessboard_vision_v2_7 import CHESS_PIECE_YOLO_CLASSES

LOCK = threading.RLock()
EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def sample_paths(root, key):
    parts = key.split("/")
    if len(parts) != 2 or parts[0] not in ("train", "val", "test") or Path(parts[1]).name != parts[1] or "\\" in key:
        raise ValueError("Invalid sample key")
    root = Path(root).resolve()
    image = (root / "images" / key).resolve()
    image.relative_to(root / "images")
    if image.suffix.lower() not in EXTENSIONS or not image.is_file():
        raise ValueError("Sample image does not exist")
    label = root / "labels" / parts[0] / (image.stem + ".txt")
    label.resolve().relative_to(root / "labels")
    return image, label


def digest(image, label):
    return hashlib.sha256(image.read_bytes() + b"\0" + (label.read_bytes() if label.exists() else b"")).hexdigest()


def read_reviews(root):
    path = Path(root) / "label_reviews.json"
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def samples(root):
    reviews = read_reviews(root)
    result = []
    for split in ("train", "val", "test"):
        for image in sorted((Path(root) / "images" / split).glob("*")):
            if image.suffix.lower() not in EXTENSIONS or not image.is_file():
                continue
            key = split + "/" + image.name
            image, label = sample_paths(root, key)
            result.append({"key": key, "reviewed": label.exists() and reviews.get(key) == digest(image, label)})
    return result


def require_reviewed_labels(root):
    pending = [s["key"] for s in samples(root) if s["key"].split("/")[0] in ("train", "val") and not s["reviewed"]]
    if pending:
        raise ValueError(f"{len(pending)} train/val samples need full-piece label review. Open /yolo-labels first. Example: {pending[0]}")


def validate_rows(rows):
    clean = []
    for row in rows:
        if len(row) != 5:
            raise ValueError("Each box must contain class,cx,cy,width,height")
        cls, cx, cy, width, height = map(float, row)
        if not all(math.isfinite(x) for x in (cls, cx, cy, width, height)):
            raise ValueError("Box coordinates must be finite")
        if cls != int(cls) or not 0 <= cls < len(CHESS_PIECE_YOLO_CLASSES):
            raise ValueError("Invalid class")
        if width <= 0 or height <= 0 or min(cx-width/2, cy-height/2) < -1e-6 or max(cx+width/2, cy+height/2) > 1+1e-6:
            raise ValueError("Box must be nonempty and inside the image")
        clean.append([int(cls), cx, cy, width, height])
    return clean


def load_sample(root, key):
    image, label = sample_paths(root, key)
    rows = [list(map(float, line.split())) for line in label.read_text().splitlines() if line.strip()] if label.exists() else []
    return {"key": key, "rows": validate_rows(rows), "revision": digest(image, label)}


def atomic_write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        temp.write_text(content, encoding="utf-8")
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def save_sample(root, key, rows, revision):
    rows = validate_rows(rows)
    with LOCK:
        image, label = sample_paths(root, key)
        if digest(image, label) != revision:
            raise ValueError("Sample changed since opening; reload before saving")
        if label.exists():
            backup = Path(root) / "label_backups" / key.split("/")[0] / (label.stem + "." + uuid.uuid4().hex + ".txt")
            backup.parent.mkdir(parents=True, exist_ok=True)
            backup.write_bytes(label.read_bytes())
        atomic_write(label, "".join(f"{r[0]} " + " ".join(f"{v:.8f}" for v in r[1:]) + "\n" for r in rows))
        # Ultralytics keeps the parsed labels beside the split directory.
        label.parent.with_suffix(".cache").unlink(missing_ok=True)
        reviews = read_reviews(root)
        reviews[key] = digest(image, label)
        atomic_write(Path(root) / "label_reviews.json", json.dumps(reviews, indent=2))
    return load_sample(root, key)


def handle_get(handler, root):
    parsed = urlparse(handler.path)
    if parsed.path == "/yolo-labels":
        handler.send_bytes(PAGE.encode("utf-8"), "text/html; charset=utf-8")
        return True
    if not parsed.path.startswith("/api/labels/"):
        return False
    try:
        with LOCK:
            if parsed.path == "/api/labels/list":
                handler.send_json({"samples": samples(root), "classes": CHESS_PIECE_YOLO_CLASSES})
            else:
                key = parse_qs(parsed.query)["key"][0]
                if parsed.path == "/api/labels/sample":
                    handler.send_json(load_sample(root, key))
                elif parsed.path == "/api/labels/image":
                    image, _ = sample_paths(root, key)
                    handler.send_bytes(image.read_bytes(), mimetypes.guess_type(image.name)[0] or "image/jpeg")
                else:
                    raise ValueError("Unknown label route")
    except (ValueError, OSError, KeyError) as exc:
        handler.send_json({"error": str(exc)}, 400)
    return True


def handle_post(handler, root):
    if urlparse(handler.path).path != "/api/labels/save":
        handler.send_json({"error": "Unknown route"}, 404)
        return
    try:
        size = int(handler.headers.get("Content-Length", "0"))
        if not 0 < size <= 1000000:
            raise ValueError("Invalid request size")
        data = json.loads(handler.rfile.read(size))
        handler.send_json(save_sample(root, data["key"], data["rows"], data["revision"]))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        handler.send_json({"error": str(exc)}, 400)


PAGE = r'''<!doctype html><html lang="zh"><meta charset="utf-8"><title>棋子标注复核</title>
<style>body{font:16px system-ui;background:#161c23;color:#eee;margin:24px}button,select{font:inherit;margin:5px;padding:8px}canvas{max-width:100%;height:auto;display:block;touch-action:none;background:#333}p{max-width:1000px;line-height:1.6}#status{white-space:pre-wrap;color:#ffd979}</style>
<h1>完整棋子标注</h1><p>选择图片和框，在原图上从棋冠左上方拖到整个底座右下方，重画选中的框。框可跨格；贴近完整轮廓，避免大块背景。检查类别，并给画面中的每枚棋子标注。绿色框为当前选中项。保存会备份旧标签。</p>
<select id="sample"></select><button id="prev">上一张</button><button id="next">下一张</button><button id="reload">撤销本页修改</button><br>
<select id="box"></select><select id="cls"></select><button id="add">新增框</button><button id="remove">删除选中框</button><button id="save">确认完整并保存 → 下一张</button>
<p id="status"></p><canvas id="canvas"></canvas>
<script>
const $=id=>document.getElementById(id), canvas=$('canvas'),ctx=canvas.getContext('2d');
let entries=[],classes=[],rows=[],current=null,img=new Image(),start=null,dirty=false,busy=false;
async function api(url,options){const r=await fetch(url,options);const d=await r.json();if(!r.ok||d.error)throw Error(d.error||r.statusText);return d;}
function options(el,items){el.replaceChildren(...items.map(([value,label])=>new Option(label,value)));}
function draw(){ctx.drawImage(img,0,0);rows.forEach((r,i)=>{let [c,x,y,w,h]=r;x=(x-w/2)*canvas.width;y=(y-h/2)*canvas.height;ctx.strokeStyle=i===Number($('box').value)?'#00ff88':'#ffcc33';ctx.lineWidth=2;ctx.strokeRect(x,y,w*canvas.width,h*canvas.height);ctx.font='14px sans-serif';ctx.fillStyle=ctx.strokeStyle;ctx.fillText(classes[c],x,Math.max(14,y-4));});}
function boxes(selected=0){options($('box'),rows.map((r,i)=>[i,`${i+1}: ${classes[r[0]]}`]));$('box').value=String(Math.min(selected,rows.length-1));if(rows.length)$('cls').value=rows[Number($('box').value)][0];draw();}
async function openSample(index,force=false){if(busy)return;if(dirty&&!force&&!confirm('放弃当前未保存修改？')){$('sample').value=current.key;return;}busy=true;try{
const key=entries[index].key,d=await api('/api/labels/sample?key='+encodeURIComponent(key));const fresh=new Image();fresh.src='/api/labels/image?key='+encodeURIComponent(key)+'&t='+Date.now();await fresh.decode();img=fresh;current=d;rows=d.rows;canvas.width=img.naturalWidth;canvas.height=img.naturalHeight;$('sample').value=key;dirty=false;boxes();$('status').textContent=`${index+1}/${entries.length} · ${entries[index].reviewed?'已复核':'待复核'} · ${key}`;
}finally{busy=false;}}
function index(){return entries.findIndex(e=>e.key===current?.key);}
function point(e){const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(1,(e.clientX-r.left)/r.width)),Math.max(0,Math.min(1,(e.clientY-r.top)/r.height))];}
canvas.onpointerdown=e=>{if(busy||!rows.length)return;start=point(e);canvas.setPointerCapture(e.pointerId);};
canvas.onpointerup=e=>{if(!start)return;const end=point(e),a=start;start=null;const w=Math.abs(end[0]-a[0]),h=Math.abs(end[1]-a[1]);if(w*canvas.width<3||h*canvas.height<3)return;rows[Number($('box').value)]=[Number($('cls').value),(end[0]+a[0])/2,(end[1]+a[1])/2,w,h];dirty=true;draw();};
canvas.onpointercancel=()=>start=null;
canvas.onpointermove=e=>{if(!start)return;draw();const p=point(e);ctx.strokeStyle='#00ff88';ctx.strokeRect(start[0]*canvas.width,start[1]*canvas.height,(p[0]-start[0])*canvas.width,(p[1]-start[1])*canvas.height);};
function action(fn){return async()=>{try{await fn();}catch(e){$('status').textContent=e.message;}};}
$('sample').onchange=action(()=>openSample(entries.findIndex(e=>e.key===$('sample').value)));
$('prev').onclick=action(()=>openSample(Math.max(0,index()-1)));$('next').onclick=action(()=>openSample(Math.min(entries.length-1,index()+1)));
$('reload').onclick=action(()=>openSample(index()));$('box').onchange=()=>{if(rows.length)$('cls').value=rows[Number($('box').value)][0];draw();};
$('cls').onchange=()=>{if(busy||!rows.length)return;const i=Number($('box').value);rows[i][0]=Number($('cls').value);dirty=true;boxes(i);};
$('add').onclick=()=>{if(busy||!current)return;rows.push([Number($('cls').value),.5,.5,.1,.1]);dirty=true;boxes(rows.length-1);};
$('remove').onclick=()=>{if(busy||!rows.length)return;rows.splice(Number($('box').value),1);dirty=true;boxes();};
$('save').onclick=action(async()=>{if(busy||!current)return;busy=true;try{const d=await api('/api/labels/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({key:current.key,revision:current.revision,rows})});current=d;dirty=false;entries[index()].reviewed=true;refreshList();$('status').textContent='已保存并备份旧标签';}finally{busy=false;}if(index()<entries.length-1)await openSample(index()+1);});
function refreshList(){const key=current?.key;options($('sample'),entries.map(e=>[e.key,(e.reviewed?'✓ ':'待复核 ')+e.key]));if(key)$('sample').value=key;}
window.onbeforeunload=e=>{if(dirty){e.preventDefault();e.returnValue='';}};
(async()=>{const d=await api('/api/labels/list');entries=d.samples;classes=d.classes;options($('cls'),classes.map((c,i)=>[i,c]));refreshList();if(entries.length)await openSample(Math.max(0,entries.findIndex(e=>!e.reviewed)));else $('status').textContent='没有训练图片，请先保存样本。';})().catch(e=>$('status').textContent=e.message);
</script></html>'''


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Check full-piece label review before CLI training")
    parser.add_argument("--dataset-dir", required=True)
    args = parser.parse_args()
    if not samples(args.dataset_dir):
        parser.error("No dataset images found")
    require_reviewed_labels(args.dataset_dir)
    print("All train/val image-label pairs have been reviewed")

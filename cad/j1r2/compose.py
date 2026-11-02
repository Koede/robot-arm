"""Compose render sheets.  per-concept 3x2 sheet + a master overview sheet."""
import sys, os, json
from PIL import Image, ImageDraw, ImageFont
from matplotlib import font_manager
R = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "renders")
F = lambda s, b=False: ImageFont.truetype(font_manager.findfont("DejaVu Sans:bold" if b else "DejaVu Sans"), s)
TITLES = json.load(open(os.path.join(R, "titles.json"))) if os.path.exists(os.path.join(R, "titles.json")) else {}
def sheet(cid, views, cols=3, w=600, h=467, out=None):
    rows = (len(views) + cols - 1) // cols; top = 64
    S = Image.new("RGB", (cols * w, rows * h + top), (244, 245, 247)); d = ImageDraw.Draw(S)
    d.text((16, 10), f"{cid}  {TITLES.get(cid, '')}", font=F(26, True), fill=(20, 24, 36)); d.text((16, 42), "proxy geometry on the MakeHuman wearer; park pose S1 45 / S2 25; grey = unchanged stage, blue = armour, yellow = assumed clasp seats", font=F(13), fill=(90, 96, 110))
    for i, v in enumerate(views):
        im = Image.open(os.path.join(R, f"{cid}_{v}.png")).resize((w, h), Image.LANCZOS); x, y = (i % cols) * w, top + (i // cols) * h
        S.paste(im, (x, y)); lab = {"hero": "rear-right, high", "heroG": "rear-right, armour ghosted", "rear": "rear", "side": "right side", "sideG": "right side, armour ghosted", "top": "top", "topG": "top, armour ghosted",
                                     "front": "front-right, high", "frontG": "front-right, armour ghosted", "low": "from below-behind", "lowG": "below-behind, armour ghosted"}.get(v, v)
        d.rectangle([x + 8, y + 8, x + 8 + 11 * len(lab) + 10, y + 30], fill=(255, 255, 255)); d.text((x + 13, y + 11), lab, font=F(13), fill=(30, 34, 46))
    out = out or os.path.join(R, f"sheet_{cid}.png"); S.save(out); return out
def master(cids, view="heroG", cols=4, w=520, h=405):
    rows = (len(cids) + cols - 1) // cols; top = 60
    S = Image.new("RGB", (cols * w, rows * h + top), (244, 245, 247)); d = ImageDraw.Draw(S)
    d.text((16, 10), "Flat shoulder joint alternatives: overview (rear-right, park pose)", font=F(26, True), fill=(20, 24, 36))
    for i, c in enumerate(cids):
        im = Image.open(os.path.join(R, f"{c}_{view}.png")).resize((w, h), Image.LANCZOS); x, y = (i % cols) * w, top + (i // cols) * h; S.paste(im, (x, y))
        lab = f"{c}  {TITLES.get(c, '')}"; d.rectangle([x + 8, y + 8, x + 16 + 10 * len(lab), y + 34], fill=(255, 255, 255)); d.text((x + 13, y + 12), lab, font=F(15, True), fill=(20, 24, 36))
    out = os.path.join(R, "overview.png"); S.save(out); return out
if __name__ == "__main__":
    if sys.argv[1] == "master": print(master(sys.argv[2].split(","), sys.argv[3] if len(sys.argv) > 3 else "heroG"))
    else: print(sheet(sys.argv[1], sys.argv[2].split(",")))

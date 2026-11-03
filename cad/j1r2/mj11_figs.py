"""MJ11 loads/balancer figures from stl/j1r2_mounts/MJ11/*.json -> renders_j1r2/mj11_loads_balancer.png"""
import json, os, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
R=os.path.join(os.path.dirname(__file__),"..","..")
D=os.path.join(R,"stl","j1r2_mounts","MJ11")
L=json.load(open(f"{D}/loads.json")); T1=json.load(open(f"{D}/torque_options.json")); T3=json.load(open(f"{D}/torque_options3.json"))
fig,ax=plt.subplots(1,3,figsize=(17,5.2))
names=[];sf=[];sfp=[]
for n,v in L["bolt_variants"].items():
    names.append(n.split(":")[0].split(" ")[0]); sf.append(v["reach th1 -90 (R 0.5 kg, L park)"]["insert_SF"]); sfp.append(v["stall S1' AK70 peak 24.8"]["insert_SF"])
x=range(len(names)); ax[0].bar([i-.2 for i in x],sf,.4,label="reach th1 -90, x2 dyn"); ax[0].bar([i+.2 for i in x],sfp,.4,label="S1' stall at AK70 peak 24.8")
ax[0].axhline(2,c="k",ls="--",lw=1); ax[0].set_xticks(list(x)); ax[0].set_xticklabels(names); ax[0].set_ylabel("insert pull-out SF (1 kN, UNVERIFIED)"); ax[0].set_title("Pad bolt layouts V0-V6"); ax[0].legend(fontsize=8)
for ws,c in(("BOX","C0"),("REACH","C1")):
    q=T1["payload_link_trade"][ws]
    for s,ls in((1.0,"-"),(0.8,"--"),(0.7,":")):
        pts=[(r["payload_kg"],r["SF"]) for r in q if r["link_scale"]==s]; ax[1].plot(*zip(*pts),c=c,ls=ls,marker="o",label=f"{ws} links x{s}")
ax[1].axhline(2,c="k",lw=1); ax[1].set_xlabel("payload kg"); ax[1].set_ylabel("worst SF = 8.3 / max(S1',S2)"); ax[1].set_title("Payload / link-length trade"); ax[1].legend(fontsize=7)
cc=T3["cap_coverage_4D"]["BOX"]; caps=list(next(iter(cc.values())).keys())
for p,v in cc.items(): ax[2].plot([float(c) for c in caps],[v[c] for c in caps],marker="o",label=f"payload {p} kg")
ax[2].axvline(4.15,c="k",ls="--",lw=1); ax[2].set_xlabel("firmware cap N.m"); ax[2].set_ylabel("fraction of BOX configs (th1,phi,E,S3) reachable"); ax[2].set_title("Cap coverage (no balancer)"); ax[2].legend(fontsize=8)
plt.tight_layout(); plt.savefig(os.path.join(R,"renders_j1r2","mj11_loads_balancer.png"),dpi=130)

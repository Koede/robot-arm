import sys, json, time
sys.path.insert(0,'/home/claude/robot-arm/cad/j1r2')
import mj_frame as M
import mj_eval as E
out={}
for ef in (0.05,0.10,0.15,0.30):
    M.E_FOAM=ef
    r=E.run_frame('MJ11')
    out[ef]={lc:{'liner':r[lc]['contact'].get('liner'),'tip':r[lc]['tip_mm']} for lc in r}
    print(ef,out[ef],flush=True)
json.dump(out,open('/home/claude/ts/efoam.json','w'))

#!/usr/bin/env python3
import argparse, csv, json
from collections import defaultdict
import numpy as np

p=argparse.ArgumentParser(); p.add_argument("csv",nargs="+"); p.add_argument("--output",required=True); a=p.parse_args()
groups=defaultdict(list)
for path in a.csv:
    with open(path,newline="") as f:
        for row in csv.DictReader(f): groups[(row["solver"],row["scenario"])].append(row)
result={}
for (solver,scenario),rows in sorted(groups.items()):
    success=[r for r in rows if r["status"]=="SUCCESS"]
    times=np.array([float(r.get("solve_time_ms",r.get("time_ms"))) for r in rows])
    result.setdefault(solver,{})[scenario]={"samples":len(rows),"success_rate":len(success)/len(rows),
      "mean_time_ms":float(times.mean()),"p95_time_ms":float(np.percentile(times,95)),
      "success_mean_position_error_m":float(np.mean([float(r["position_error_m"]) for r in success])) if success else None,
      "success_mean_orientation_error_rad":float(np.mean([float(r["orientation_error_rad"]) for r in success])) if success else None}
with open(a.output,"w") as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))

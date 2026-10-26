#!/usr/bin/env python3
import argparse, csv, json, os, sys
import numpy as np
sys.path.insert(0,os.path.dirname(__file__))
from optimization_ik import OptimizationIk

p=argparse.ArgumentParser(); p.add_argument("--urdf",required=True); p.add_argument("--count",type=int,default=100)
p.add_argument("--seed",type=int,default=7506); p.add_argument("--output",required=True); p.add_argument("--samples-csv")
a=p.parse_args(); rng=np.random.default_rng(a.seed); ik=OptimizationIk(a.urdf); rows=[]; samples=[]
if a.samples_csv:
    with open(a.samples_csv,newline="") as f:
        for row in csv.DictReader(f):
            if row["solver"]!="kdl": continue
            samples.append((row["scenario"],np.array([float(row[f"target_q{k}"]) for k in range(1,8)]),
                            np.array([float(row[f"seed_q{k}"]) for k in range(1,8)])))
            if len(samples)>=a.count: break
else:
    for _ in range(a.count):
        samples.append(("random_6d",(ik.lower+ik.upper)/2+rng.uniform(-.82,.82,7)*(ik.upper-ik.lower)/2,
                        (ik.lower+ik.upper)/2+rng.uniform(-.55,.55,7)*(ik.upper-ik.lower)/2))
for i,(scenario,truth,seed) in enumerate(samples):
    result=ik.solve(ik.forward(truth),seed,warm_start=False,orientation_enabled=scenario!="random_position")
    result.update(sample=i,scenario=scenario,solver="casadi_ipopt"); rows.append(result)
a.count=len(rows)
fields=["solver","sample","scenario","status","solve_time_ms","iterations","position_error_m","orientation_error_rad","objective","min_limit_margin_rad","detail"]
with open(a.output+".csv","w",newline="") as f:
    w=csv.DictWriter(f,fields,extrasaction="ignore");w.writeheader();w.writerows(rows)
success=[r for r in rows if r["status"]=="SUCCESS"]; orientation_success=[r for r in success if r["scenario"]!="random_position"]
times=np.array([r["solve_time_ms"] for r in rows])
summary={"model":"RM75-6FB","solver":"CasADi 3.7.2 / Ipopt","seed":a.seed,"samples":a.count,
 "success_rate":len(success)/a.count,"mean_time_ms":float(times.mean()),"p50_time_ms":float(np.percentile(times,50)),
 "p95_time_ms":float(np.percentile(times,95)),"p99_time_ms":float(np.percentile(times,99)),
 "success_mean_position_error_m":float(np.mean([r["position_error_m"] for r in success])) if success else None,
 "orientation_tasks_success_mean_error_rad":float(np.mean([r["orientation_error_rad"] for r in orientation_success])) if orientation_success else None,
 "failure_count":a.count-len(success),"numeric_error_count":sum(r["status"]=="NUMERICAL_ERROR" for r in rows)}
with open(a.output+".json","w") as f:json.dump(summary,f,indent=2)
print(json.dumps(summary,indent=2))

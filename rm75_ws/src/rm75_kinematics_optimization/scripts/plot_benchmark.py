#!/usr/bin/env python3
"""Create benchmark plots from the raw CSV; never synthesizes missing data."""
import argparse, csv
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

p=argparse.ArgumentParser()
p.add_argument("csv")
p.add_argument("--output",required=True)
a=p.parse_args()
data=defaultdict(list)
with open(a.csv,newline="") as f:
    for row in csv.DictReader(f): data[row["solver"]].append(row)

names=list(data)
success=[sum(r["status"]=="SUCCESS" for r in data[n])/len(data[n]) for n in names]
plt.figure(figsize=(8,4)); plt.bar(names,success); plt.ylim(0,1); plt.ylabel("success rate")
plt.xticks(rotation=15); plt.tight_layout(); plt.savefig(a.output+"_success.png",dpi=150); plt.close()
for field,suffix,xlabel,log in [
    ("time_ms","time","solve time (ms)",True),
    ("position_error_m","position_error","position error (m)",True),
    ("min_limit_margin_rad","limit_margin","minimum joint-limit margin (rad)",False)]:
    plt.figure(figsize=(8,4))
    for n in names:
        vals=[float(r[field]) for r in data[n]]
        plt.hist(vals,bins=40,histtype="step",density=True,label=n)
    if log: plt.xscale("log")
    plt.xlabel(xlabel); plt.ylabel("density"); plt.legend(); plt.tight_layout()
    plt.savefig(a.output+"_"+suffix+".png",dpi=150); plt.close()

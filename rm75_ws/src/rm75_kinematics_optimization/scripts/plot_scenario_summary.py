#!/usr/bin/env python3
import argparse, json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser(); p.add_argument("json"); p.add_argument("--output",required=True); a=p.parse_args()
d=json.load(open(a.json)); solvers=list(d); scenarios=list(next(iter(d.values())))
x=np.arange(len(scenarios)); width=.8/len(solvers)
plt.figure(figsize=(12,5))
for i,solver in enumerate(solvers):
    plt.bar(x-.4+width/2+i*width,[d[solver][s]["success_rate"] for s in scenarios],width,label=solver)
plt.xticks(x,scenarios,rotation=20,ha="right");plt.ylim(0,1.05);plt.ylabel("success rate")
plt.legend(fontsize=8,ncol=3);plt.tight_layout();plt.savefig(a.output,dpi=160);plt.close()

#!/usr/bin/env python3
"""Offline CasADi/Ipopt constrained IK for a serial URDF chain."""
import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import List, Optional

import casadi as ca
import numpy as np


def _values(text, default):
    return np.array([float(x) for x in text.split()]) if text else np.array(default, dtype=float)


def _rpy(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def _transform(rotation, translation):
    t = np.eye(4); t[:3, :3] = rotation; t[:3, 3] = translation; return t


@dataclass
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


def load_chain(path: str, base: str, tip: str) -> List[Joint]:
    root = ET.parse(path).getroot(); by_child = {}
    for e in root.findall("joint"):
        origin = e.find("origin")
        xyz = _values(origin.get("xyz") if origin is not None else None, [0, 0, 0])
        rpy = _values(origin.get("rpy") if origin is not None else None, [0, 0, 0])
        axis_e = e.find("axis"); axis = _values(axis_e.get("xyz") if axis_e is not None else None, [1, 0, 0])
        limit = e.find("limit"); kind = e.get("type")
        lower = float(limit.get("lower")) if limit is not None and limit.get("lower") else 0.0
        upper = float(limit.get("upper")) if limit is not None and limit.get("upper") else 0.0
        j = Joint(e.get("name"), kind, e.find("parent").get("link"), e.find("child").get("link"),
                  _transform(_rpy(rpy), xyz), axis / np.linalg.norm(axis), lower, upper)
        by_child[j.child] = j
    chain=[]; link=tip
    while link != base:
        if link not in by_child: raise ValueError(f"No URDF path from {base} to {tip}; stopped at {link}")
        chain.append(by_child[link]); link=chain[-1].parent
    chain.reverse()
    moving=[j for j in chain if j.kind in ("revolute", "continuous", "prismatic")]
    if len(moving) != 7: raise ValueError(f"Expected 7 moving joints, got {len(moving)}")
    return chain


def _axis_rotation(axis, q):
    x, y, z = [float(v) for v in axis]
    k = ca.DM([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return ca.DM.eye(3) + ca.sin(q)*k + (1-ca.cos(q))*ca.mtimes(k, k)


class OptimizationIk:
    def __init__(self, urdf: str, base="base_link", tip="Link7", max_iterations=200, max_cpu_time=0.2):
        self.chain=load_chain(urdf, base, tip); self.moving=[j for j in self.chain if j.kind != "fixed"]
        self.lower=np.array([j.lower for j in self.moving]); self.upper=np.array([j.upper for j in self.moving])
        q=ca.SX.sym("q", 7); t=ca.SX.eye(4); qi=0
        for joint in self.chain:
            t=ca.mtimes(t, ca.DM(joint.origin))
            if joint.kind in ("revolute", "continuous"):
                tj=ca.SX.eye(4); tj[:3,:3]=_axis_rotation(joint.axis,q[qi]); t=ca.mtimes(t,tj); qi+=1
            elif joint.kind == "prismatic":
                tj=ca.SX.eye(4); tj[:3,3]=ca.DM(joint.axis)*q[qi]; t=ca.mtimes(t,tj); qi+=1
        self.fk=ca.Function("rm75_symbolic_fk",[q],[t])
        target=ca.SX.sym("target",12); qref=ca.SX.sym("qref",7); orientation_scale=ca.SX.sym("orientation_scale")
        pt=target[:3]; rt=ca.reshape(target[3:],3,3).T
        r=t[:3,:3]; rel=ca.mtimes(rt,r.T)
        cosine=ca.fmin(1-1e-10,ca.fmax(-1+1e-10,(ca.trace(rel)-1)/2)); theta=ca.acos(cosine)
        vee=ca.vertcat(rel[2,1]-rel[1,2],rel[0,2]-rel[2,0],rel[1,0]-rel[0,1])
        rotvec=ca.if_else(theta<1e-5,0.5*vee,theta/(2*ca.sin(theta))*vee)
        center=(self.lower+self.upper)/2; half=(self.upper-self.lower)/2
        normalized=(q-ca.DM(center))/ca.DM(half)
        objective=400*ca.sumsqr(t[:3,3]-pt)+orientation_scale*25*ca.sumsqr(rotvec)+0.02*ca.sumsqr(q-qref)+0.001*ca.sumsqr(normalized)
        nlp={"x":q,"p":ca.vertcat(target,qref,orientation_scale),"f":objective}
        options={"print_time":False,"ipopt.print_level":0,"ipopt.max_iter":max_iterations,
                 "ipopt.max_cpu_time":max_cpu_time,"ipopt.tol":1e-8,"ipopt.acceptable_tol":1e-6,
                 "ipopt.warm_start_init_point":"yes"}
        self.solver=ca.nlpsol("rm75_ipopt","ipopt",nlp,options); self._lam_x=None

    def forward(self,q): return np.array(self.fk(np.asarray(q,dtype=float))).astype(float)

    def solve(self,target,seed,position_tolerance=.002,orientation_tolerance=math.radians(2),warm_start=True,orientation_enabled=True):
        seed=np.asarray(seed,dtype=float); target=np.asarray(target,dtype=float)
        if seed.shape != (7,) or target.shape != (4,4) or not np.isfinite(seed).all() or not np.isfinite(target).all():
            return {"status":"NUMERICAL_ERROR","detail":"invalid input"}
        p=np.r_[target[:3,3],target[:3,:3].reshape(-1),seed,float(orientation_enabled)]; args={"x0":seed,"p":p,"lbx":self.lower,"ubx":self.upper}
        if warm_start and self._lam_x is not None: args["lam_x0"]=self._lam_x
        started=time.perf_counter()
        try:
            sol=self.solver(**args); elapsed=(time.perf_counter()-started)*1000; q=np.array(sol["x"]).ravel(); self._lam_x=sol["lam_x"]
            actual=self.forward(q); pe=float(np.linalg.norm(actual[:3,3]-target[:3,3]))
            re=float(math.acos(np.clip((np.trace(target[:3,:3]@actual[:3,:3].T)-1)/2,-1,1)))
            stats=self.solver.stats(); success=pe<=position_tolerance and (not orientation_enabled or re<=orientation_tolerance) and np.all(q>=self.lower) and np.all(q<=self.upper)
            detail=stats.get("return_status","")
            if success: status="SUCCESS"
            elif "Maximum_Iterations" in detail: status="MAX_ITERATIONS"
            elif "Maximum_CpuTime" in detail or "Maximum_WallTime" in detail: status="TIMEOUT"
            elif "Infeasible" in detail: status="JOINT_LIMIT"
            else: status="UNREACHABLE_OR_STALLED"
            return {"status":status,"detail":detail,
                    "q":q.tolist(),"solve_time_ms":elapsed,"iterations":stats.get("iter_count",0),
                    "position_error_m":pe,"orientation_error_rad":re,"objective":float(sol["f"]),
                    "min_limit_margin_rad":float(np.min(np.minimum(q-self.lower,self.upper-q)))}
        except Exception as exc:
            return {"status":"NUMERICAL_ERROR","detail":str(exc),"solve_time_ms":(time.perf_counter()-started)*1000}


def main():
    p=argparse.ArgumentParser(); p.add_argument("--urdf",required=True); p.add_argument("--target-q",nargs=7,type=float,required=True)
    p.add_argument("--seed",nargs=7,type=float,default=[0]*7); p.add_argument("--no-warm-start",action="store_true")
    a=p.parse_args(); ik=OptimizationIk(a.urdf); target=ik.forward(a.target_q)
    result=ik.solve(target,a.seed,warm_start=not a.no_warm_start); print(json.dumps(result,indent=2)); return 0 if result["status"]=="SUCCESS" else 1


if __name__ == "__main__": raise SystemExit(main())

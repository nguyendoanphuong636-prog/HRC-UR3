#!/usr/bin/env python3
"""Estimate camera optical -> robot base_link rigid transforms from measured 3D point pairs (m)."""
import argparse, csv, json
from pathlib import Path
import numpy as np


def fit(src, dst):
    src, dst = np.asarray(src,float), np.asarray(dst,float)
    if len(src) < 4 or src.shape != dst.shape or src.shape[1:] != (3,) or not np.isfinite([src,dst]).all():
        raise ValueError('Need >=4 finite paired positions')
    a,b=src-src.mean(0),dst-dst.mean(0)
    for v in (a,b):
        s=np.linalg.svd(v, compute_uv=False)
        if s[1] < 0.05: raise ValueError('Points nearly collinear; spread points along different axes')
    u,_,vt=np.linalg.svd(a.T@b)
    d=np.eye(3); d[2,2]=np.linalg.det(vt.T@u.T)
    R=vt.T@d@u.T
    t=dst.mean(0)-R@src.mean(0)
    return R,t


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv'); p.add_argument('--out',required=True)
    p.add_argument('--max-fit-rmse-m',type=float,default=.05)
    p.add_argument('--max-check-error-m',type=float,default=.08)
    args=p.parse_args()
    if not (args.max_fit_rmse_m>0 and args.max_check_error_m>0): p.error('thresholds must be positive')
    with open(args.csv,newline='') as f: rows=list(csv.DictReader(f))
    result={'schema':'phase67_measured_v1','target_frame':'base_link','cameras':{}}
    for cam in ('front','back'):
        r=[x for x in rows if x['camera']==cam]
        if any(x['split'] not in ('fit','check') for x in r): raise ValueError('split must be fit or check')
        if len({x['point_id'] for x in r})!=len(r): raise ValueError('point IDs must be unique per camera')
        def xyz(rows, prefix): return np.array([[float(x[prefix+a]) for a in ('x','y','z')] for x in rows])
        f=[x for x in r if x['split']=='fit']; c=[x for x in r if x['split']=='check']
        if len(c)<1: raise ValueError(cam+': at least one independent check point required')
        a,b=xyz(f,'cam_'),xyz(f,'base_'); R,t=fit(a,b)
        err=np.linalg.norm(a@R.T+t-b,axis=1)
        ca,cb=xyz(c,'cam_'),xyz(c,'base_')
        if not np.isfinite([ca,cb]).all(): raise ValueError('Nonfinite check points')
        check=np.linalg.norm(ca@R.T+t-cb,axis=1)
        rmse=float(np.sqrt(np.mean(err**2)))
        print(f'{cam}: fit RMSE={rmse*1000:.1f} mm, max fit={max(err)*1000:.1f} mm, independent check errors (mm)={list(np.round(check*1000,1))}')
        if rmse>args.max_fit_rmse_m or max(check)>args.max_check_error_m: raise ValueError(cam+': preliminary research thresholds exceeded; remeasure')
        result['cameras'][cam]={'source_frame':f'kinect_{cam}_depth_optical_frame','R':R.tolist(),'t':t.tolist(),'fit_rmse_m':rmse,'check_errors_m':check.tolist()}
    out=Path(args.out); out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2)+'\n'); print('Saved measured, preliminary calibration:',out)
    print('Manual point uncertainty, limited check points; NOT safety validation.')

if __name__=='__main__': main()

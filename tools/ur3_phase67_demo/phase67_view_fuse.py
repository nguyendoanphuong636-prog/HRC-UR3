#!/usr/bin/env python3
"""Read-only skeleton visualizer and preliminary calibrated fusion. No robot commands."""
import argparse, json, math, time, copy
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from ur3_human_fusion.fusion_core import BODY_IDS, fuse

EDGES=((0,11),(0,12),(11,12),(11,13),(13,15),(12,14),(14,16),(11,23),(12,24),(23,24))

def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec/1e9

def decode(m):
    return {j: (p.position.x,p.position.y,p.position.z) for j,p in zip(BODY_IDS,m.poses)
            if p.orientation.w>0 and all(math.isfinite(i) for i in (p.position.x,p.position.y,p.position.z))}

def array(points, header, frame):
    m=PoseArray(); m.header=copy.deepcopy(header); m.header.frame_id=frame
    for j in BODY_IDS:
        p=Pose(); p.position.x,p.position.y,p.position.z=points.get(j,(float('nan'),)*3)
        p.orientation.w=1.0 if j in points else 0.0; m.poses.append(p)
    return m

class Viewer(Node):
    def __init__(self,args):
        super().__init__('phase67_view_fuse')
        self.cal={}; self.q={c:deque(maxlen=40) for c in ('front','back')}; self.last={c:0 for c in self.q}
        self.max_skew=args.max_skew; self.max_age=args.max_age; self.hold=args.hold
        self.colors={'front':(.15,.65,1.),'back':(1.,.55,.1),'fused':(.2,1.,.35)}
        if args.calibration:
            with open(args.calibration) as f: data=json.load(f)
            if data.get('schema')!='phase67_measured_v1' or data.get('target_frame')!='base_link': raise ValueError('Invalid calibration schema')
            for c in self.q:
                v=data['cameras'][c]; R=np.array(v['R'],float); t=np.array(v['t'],float)
                if v['source_frame']!=f'kinect_{c}_depth_optical_frame' or R.shape!=(3,3) or t.shape!=(3,) or not np.isfinite(R).all() or not np.isfinite(t).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(R),1): raise ValueError('Invalid transform '+c)
                self.cal[c]=(R,t)
        self.mp={c:self.create_publisher(MarkerArray,f'/phase67/{c}/markers',10) for c in (*self.q,'fused')}
        self.status=self.create_publisher(String,'/phase67/status',10)
        self.basepub={}; self.out=None; self.last_event='NO_INPUT'; self.last_output=0.
        if self.cal:
            self.basepub={c:self.create_publisher(PoseArray,f'/phase67/{c}/human_skeleton_base',10) for c in self.q}
            self.out=self.create_publisher(PoseArray,'/phase67/human_skeleton_base',10)
        for c in self.q:
            self.create_subscription(PoseArray,f'/kinect_{c}/human_skeleton_depth',lambda m,c=c:self.recv(c,m),10)
        self.create_timer(.05,self.tick); self.create_timer(1.,self.heartbeat)
        self.get_logger().info('CALIBRATED PRELIMINARY RESEARCH' if self.cal else 'RAW CAMERA FRAMES ONLY: no base fusion published')
    def report(self,mode,**kw):
        self.last_event=mode
        self.status.publish(String(data=json.dumps({'mode':mode,'calibration_loaded':bool(self.cal),'research_only':True,**kw})))
    def markers(self,c,points,header):
        arr=MarkerArray()
        for i,typ in enumerate((Marker.SPHERE_LIST,Marker.LINE_LIST)):
            m=Marker(); m.header=copy.deepcopy(header); m.ns='phase67_'+c; m.id=i; m.type=typ; m.action=Marker.ADD
            m.pose.orientation.w=1.; m.scale.x=.035 if i==0 else .014; m.scale.y=m.scale.z=.035
            m.color.r,m.color.g,m.color.b=self.colors[c];m.color.a=1.;m.lifetime=Duration(sec=1)
            vals=list(points.values()) if i==0 else [points[j] for a,b in EDGES if a in points and b in points for j in (a,b)]
            m.points=[Point(x=float(x),y=float(y),z=float(z)) for x,y,z in vals]
            if not vals: m.action=Marker.DELETE
            arr.markers.append(m)
        self.mp[c].publish(arr)
    def recv(self,c,m):
        if m.header.frame_id!=f'kinect_{c}_depth_optical_frame' or len(m.poses)!=9: return
        s=stamp(m); now=self.get_clock().now().nanoseconds/1e9
        if s<=self.last[c] or not (-.05<=now-s<=self.max_age): return
        self.last[c]=s; p=decode(m); self.markers(c,p,m.header)
        if not self.cal: return
        R,t=self.cal[c]; p={j:tuple(R@v+t) for j,v in p.items()}
        b=array(p,m.header,'base_link'); self.basepub[c].publish(b)
        self.q[c].append((s,time.monotonic(),p,b.header))
    def output(self,f,b):
        front=f[2] if f else {}; back=b[2] if b else {}
        earliest=min((x for x in (f,b) if x),key=lambda x:x[0]); header=earliest[3]
        # Original baseline: average close points, prefer Front for >20 cm conflicts.
        points,mode=fuse(front,back,BODY_IDS,.20)
        conflicts=sum(math.dist(front[j],back[j])>.20 for j in front.keys() & back.keys())
        self.out.publish(array(points,header,'base_link')); self.markers('fused',points,header)
        self.last_output=time.monotonic()
        self.report(mode, paired=bool(f and b), skew_ms=round(abs(f[0]-b[0])*1000,1) if f and b else None,
            front_joints=len(front),back_joints=len(back),fused=len(points),conflicts=conflicts)
    def tick(self):
        if not self.cal: return
        now=self.get_clock().now().nanoseconds/1e9; wall=time.monotonic()
        for q in self.q.values():
            while q and now-q[0][0]>self.max_age:q.popleft()
        # Consume each source stamp at most once. Wait briefly to find nearest partner.
        while any(self.q.values()):
            c=min((c for c in self.q if self.q[c]),key=lambda c:self.q[c][0][0]); other='back' if c=='front' else 'front'
            x=self.q[c][0]
            if wall-x[1]<self.hold: break
            self.q[c].popleft(); q=self.q[other]; y=None
            if q:
                i=min(range(len(q)),key=lambda i:abs(q[i][0]-x[0]))
                if abs(q[i][0]-x[0])<=self.max_skew: y=q[i]; del q[i]
            self.output(x,y) if c=='front' else self.output(y,x)
    def heartbeat(self):
        now=self.get_clock().now().nanoseconds/1e9
        if not self.cal: self.report('RAW_ONLY_UNCALIBRATED',front_fresh=0<=now-self.last['front']<=self.max_age,back_fresh=0<=now-self.last['back']<=self.max_age)
        elif time.monotonic()-self.last_output>self.max_age: self.report('STALE_OR_NO_INPUT')

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--calibration');p.add_argument('--max-skew',type=float,default=.15);p.add_argument('--hold',type=float,default=.25);p.add_argument('--max-age',type=float,default=1.0)
    a=p.parse_args();
    if not 0<a.max_skew<=a.hold<a.max_age: p.error('Require 0 < max-skew <= hold < max-age')
    rclpy.init();n=None
    try: n=Viewer(a);rclpy.spin(n)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if n:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()

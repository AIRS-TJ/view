#!/usr/bin/env python
# -*- coding: UTF8 -*-ss

import rosbag
import sys
import os
import logging
import numpy as np
import matplotlib.pyplot as plt
from evo.core import sync
from evo.core.trajectory import PosePath3D, PoseTrajectory3D
import evo.core.transformations as tr
import evo.core.lie_algebra as lie
from evo.tools import plot, file_interface

def se3_poses_to_xyz_quat_wxyz(poses):
    xyz = np.array([pose[:3, 3] for pose in poses])
    quat_wxyz = np.array([tr.quaternion_from_matrix(pose) for pose in poses])
    return xyz, quat_wxyz

def transform(self, t):
    right_mul,propagate= False, False

    if right_mul and not propagate:
        # Transform each pose individually.
        self._poses_se3 = [np.dot(p, t) for p in self.poses_se3]
    elif right_mul and propagate:
        # Transform each pose and propagate resulting drift to the next.
        ids = np.arange(0, self.num_poses, 1)
        rel_poses = [
            lie.relative_se3(self.poses_se3[i], self.poses_se3[j]).dot(t)
            for i, j in zip(ids, ids[1:])
        ]
        self._poses_se3 = [self.poses_se3[0]]
        for i, j in zip(ids[:-1], ids):
            self._poses_se3.append(self._poses_se3[j].dot(rel_poses[i]))
    else:
        self._poses_se3 = [np.dot(t, p) for p in self.poses_se3]
    self._positions_xyz, self._orientations_quat_wxyz \
        = se3_poses_to_xyz_quat_wxyz(self.poses_se3)


def align_origin(self, traj_ref):
    if self.num_poses == 0 or traj_ref.num_poses == 0:
        raise Exception("can't align an empty trajectory...")
    traj_origin = self.poses_se3[0]
    traj_ref_origin = traj_ref.poses_se3[0]
    to_ref_origin =np.r_[np.c_[np.identity(3),traj_ref_origin[0:3,3]-traj_origin[0:3,3]],[[0,0,0,1]]]
    # to_ref_origin = np.dot(traj_ref_origin, lie.se3_inverse(traj_origin))
    logging.debug(
        "Origin alignment transformation:\n{}".format(to_ref_origin))
    transform(self,to_ref_origin )
    return to_ref_origin

def set_origin(traj1, traj2):
    if traj1.num_poses == 0 or traj2.num_poses == 0:
        raise Exception("can't align an empty trajectory...")
    traj1_origin = traj1.poses_se3[0]
    traj2_origin = traj2.poses_se3[0]
    traj1_to_coordinate_origin=np.r_[np.c_[np.identity(3),np.zeros(3)-traj1_origin[0:3,3]],[[0,0,0,1]]] 
    traj2_to_coordinate_origin=np.r_[np.c_[np.identity(3),np.zeros(3)-traj2_origin[0:3,3]],[[0,0,0,1]]]
    transform(traj1,traj1_to_coordinate_origin)
    transform(traj2,traj2_to_coordinate_origin)


def umeyama_alignment(x, y,with_scale):

    if x.shape != y.shape:
        raise Exception("data matrices must have the same shape")
    m, n = x.shape

    mean_x = x.mean(axis=1)
    mean_y = y.mean(axis=1)

    sigma_x = 1.0 / n * (np.linalg.norm(x - mean_x[:, np.newaxis])**2)

    outer_sum = np.zeros((m, m))
    for i in range(n):
        outer_sum += np.outer((y[:, i] - mean_y), (x[:, i] - mean_x))
    cov_xy = np.multiply(1.0 / n, outer_sum)

    u, d, v = np.linalg.svd(cov_xy)
    if np.count_nonzero(d > np.finfo(d.dtype).eps) < m - 1:
        raise Exception("Degenerate covariance rank, "
                                "Umeyama alignment is not possible")

    s = np.eye(m)
    if np.linalg.det(u) * np.linalg.det(v) < 0.0:
        s[m - 1, m - 1] = -1

    r = u.dot(s).dot(v)

    c = 1 / sigma_x * np.trace(np.diag(d).dot(s)) if with_scale else 1.0
    t = mean_y - np.multiply(c, r.dot(mean_x))

    return r, t, c

if __name__=="__main__":

    if len(sys.argv) < 4:
        print("usage: my_node.py arg1:bag_path arg2:trj_ref arg3:trj_est")
    else:
        if not os.path.exists(sys.argv[1]):
            sys.exit('ERROR: %s was not found!' % sys.argv[1])
        bag = rosbag.Bag(sys.argv[1])
        topic_ref=sys.argv[2]
        topic_est=sys.argv[3]
        traj_ref = file_interface.read_bag_trajectory(bag, sys.argv[2])
        traj_est = file_interface.read_bag_trajectory(bag, sys.argv[3])
        synv_traj_ref, synv_traj_est= sync.associate_trajectories(traj_ref, traj_est)
        r_a, t_a, s = umeyama_alignment(synv_traj_est._positions_xyz.T,synv_traj_ref.positions_xyz.T,True)
        T=np.r_[np.c_[r_a, t_a],[[0,0,0,1]]]
        transform(traj_est,T)
        traj_ref.scale(s)
        traj_est.scale(s)
        align_origin(traj_est,traj_ref)
        set_origin(traj_est,traj_ref)  
        with rosbag.Bag('output.bag', 'w') as outbag:
            i=0
            for topic_ref, msg_ref, t in rosbag.Bag(sys.argv[1]).read_messages(topic_ref):
                msg_ref_pub = msg_ref
                msg_ref_pub.pose.position.x=traj_ref.positions_xyz[i,0]
                msg_ref_pub.pose.position.y=traj_ref.positions_xyz[i,1]
                msg_ref_pub.pose.position.z=traj_ref.positions_xyz[i,2]
                msg_ref_pub.pose.orientation.w=traj_ref.orientations_quat_wxyz[i,0]
                msg_ref_pub.pose.orientation.x=traj_ref.orientations_quat_wxyz[i,1]
                msg_ref_pub.pose.orientation.y=traj_ref.orientations_quat_wxyz[i,2]
                msg_ref_pub.pose.orientation.z=traj_ref.orientations_quat_wxyz[i,3]
                i=i+1
                outbag.write(topic_ref, msg_ref_pub, msg_ref_pub.header.stamp if msg_ref_pub._has_header else t)
            i=0
            for topic_est, msg_est, t in rosbag.Bag(sys.argv[1]).read_messages(topic_est):
                msg_est_pub = msg_est
                msg_est_pub.pose.pose.position.x=traj_est.positions_xyz[i,0]
                msg_est_pub.pose.pose.position.y=traj_est.positions_xyz[i,1]
                msg_est_pub.pose.pose.position.z=traj_est.positions_xyz[i,2]
                msg_est_pub.pose.pose.orientation.w=traj_est.orientations_quat_wxyz[i,0]
                msg_est_pub.pose.pose.orientation.x=traj_est.orientations_quat_wxyz[i,1]
                msg_est_pub.pose.pose.orientation.y=traj_est.orientations_quat_wxyz[i,2]
                msg_est_pub.pose.pose.orientation.z=traj_est.orientations_quat_wxyz[i,3]
                i=i+1
                outbag.write(topic_est, msg_est_pub, msg_est_pub.header.stamp if msg_est_pub._has_header else t)

        bag.close()
        print("bag.close")
        
        #plot
        fig = plt.figure(figsize=(8, 8))
        plot_mode = plot.PlotMode.xy
        ax = plot.prepare_axis(fig, plot_mode)
        plot.traj(ax, plot_mode, traj_ref, '--', 'gray','reference')
        plot.traj(ax, plot_mode, traj_est, '-', 'blue','est')
        fig.axes.append(ax)
        plt.title('align trajectory')
        plt.show()
  
        
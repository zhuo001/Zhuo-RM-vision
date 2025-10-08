"""
ByteTrack 多目标跟踪算法
基于检测框进行目标跟踪，为每个目标分配唯一ID
"""

import numpy as np
from collections import deque
import lap


class STrack:
    """单个跟踪目标"""
    
    shared_kalman = None
    track_id_count = 0
    
    def __init__(self, tlbr, score):
        """
        Args:
            tlbr: [x1, y1, x2, y2] 边界框
            score: 检测置信度
        """
        self.tlbr = np.asarray(tlbr, dtype=np.float32)
        self.score = score
        
        self.track_id = 0
        self.is_activated = False
        self.tracklet_len = 0
        
        # 状态
        self.state = 'new'  # 'new', 'tracked', 'lost', 'removed'
        
        # 历史
        self.frame_id = 0
        self.start_frame = 0
        
        # 平滑位置
        self.smooth_tlbr = deque(maxlen=30)
        
    @staticmethod
    def next_id():
        STrack.track_id_count += 1
        return STrack.track_id_count
    
    def activate(self, frame_id):
        """激活跟踪"""
        self.track_id = self.next_id()
        self.tracklet_len = 0
        self.state = 'tracked'
        self.is_activated = True
        self.frame_id = frame_id
        self.start_frame = frame_id
        
    def re_activate(self, new_track, frame_id, new_id=False):
        """重新激活丢失的跟踪"""
        self.tlbr = new_track.tlbr
        self.score = new_track.score
        self.tracklet_len = 0
        self.state = 'tracked'
        self.is_activated = True
        self.frame_id = frame_id
        if new_id:
            self.track_id = self.next_id()
    
    def update(self, new_track, frame_id):
        """更新跟踪"""
        self.frame_id = frame_id
        self.tracklet_len += 1
        
        self.tlbr = new_track.tlbr
        self.score = new_track.score
        self.state = 'tracked'
        self.is_activated = True
        
        # 保存平滑历史
        self.smooth_tlbr.append(self.tlbr.copy())
    
    def mark_lost(self):
        """标记为丢失"""
        self.state = 'lost'
    
    def mark_removed(self):
        """标记为移除"""
        self.state = 'removed'
    
    @property
    def tlwh(self):
        """[x1, y1, width, height]"""
        ret = self.tlbr.copy()
        ret[2:] -= ret[:2]
        return ret
    
    @property
    def xyxy(self):
        """[x1, y1, x2, y2]"""
        return self.tlbr.copy()


def iou_distance(tracks, detections):
    """
    计算跟踪目标和检测之间的IOU距离矩阵
    
    Args:
        tracks: list of STrack
        detections: list of STrack
        
    Returns:
        cost_matrix: (len(tracks), len(detections))
    """
    if len(tracks) == 0 or len(detections) == 0:
        return np.zeros((len(tracks), len(detections)), dtype=np.float32)
    
    # 提取边界框
    track_boxes = np.array([t.xyxy for t in tracks])
    det_boxes = np.array([d.xyxy for d in detections])
    
    # 计算IOU
    ious = batch_iou(track_boxes, det_boxes)
    
    # 转换为距离（1 - IOU）
    cost_matrix = 1 - ious
    
    return cost_matrix


def batch_iou(boxes_a, boxes_b):
    """
    计算两组边界框之间的IOU
    
    Args:
        boxes_a: (N, 4) [x1, y1, x2, y2]
        boxes_b: (M, 4) [x1, y1, x2, y2]
        
    Returns:
        iou: (N, M)
    """
    boxes_a = np.expand_dims(boxes_a, axis=1)  # (N, 1, 4)
    boxes_b = np.expand_dims(boxes_b, axis=0)  # (1, M, 4)
    
    # 计算交集
    inter_x1 = np.maximum(boxes_a[..., 0], boxes_b[..., 0])
    inter_y1 = np.maximum(boxes_a[..., 1], boxes_b[..., 1])
    inter_x2 = np.minimum(boxes_a[..., 2], boxes_b[..., 2])
    inter_y2 = np.minimum(boxes_a[..., 3], boxes_b[..., 3])
    
    inter_w = np.maximum(0, inter_x2 - inter_x1)
    inter_h = np.maximum(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    
    # 计算并集
    area_a = (boxes_a[..., 2] - boxes_a[..., 0]) * (boxes_a[..., 3] - boxes_a[..., 1])
    area_b = (boxes_b[..., 2] - boxes_b[..., 0]) * (boxes_b[..., 3] - boxes_b[..., 1])
    union_area = area_a + area_b - inter_area
    
    # IOU
    iou = inter_area / np.maximum(union_area, 1e-6)
    
    return iou.squeeze()


def linear_assignment(cost_matrix, thresh):
    """
    线性分配（匈牙利算法）
    
    Args:
        cost_matrix: (N, M) 代价矩阵
        thresh: 阈值，大于此值的匹配将被拒绝
        
    Returns:
        matches: list of (track_idx, detection_idx)
        unmatched_tracks: list of track indices
        unmatched_detections: list of detection indices
    """
    if cost_matrix.size == 0:
        return [], tuple(range(cost_matrix.shape[0])), tuple(range(cost_matrix.shape[1]))
    
    matches, unmatched_a, unmatched_b = [], [], []
    cost, x, y = lap.lapjv(cost_matrix, extend_cost=True, cost_limit=thresh)
    
    for ix, mx in enumerate(x):
        if mx >= 0:
            matches.append([ix, mx])
    unmatched_a = np.where(x < 0)[0]
    unmatched_b = np.where(y < 0)[0]
    matches = np.asarray(matches)
    
    return matches, unmatched_a, unmatched_b


class ByteTracker:
    """ByteTrack 跟踪器"""
    
    def __init__(self, track_thresh=0.5, track_buffer=30, match_thresh=0.8):
        """
        Args:
            track_thresh: 高置信度跟踪阈值
            track_buffer: 最大丢失帧数
            match_thresh: IOU匹配阈值
        """
        self.track_thresh = track_thresh
        self.track_buffer = track_buffer
        self.match_thresh = match_thresh
        
        self.tracked_tracks = []  # 正在跟踪的目标
        self.lost_tracks = []     # 丢失的目标
        self.removed_tracks = []  # 移除的目标
        
        self.frame_id = 0
        
    def update(self, detections):
        """
        更新跟踪器
        
        Args:
            detections: list of [x1, y1, x2, y2, score]
            
        Returns:
            tracked_tracks: list of STrack (有效跟踪)
        """
        self.frame_id += 1
        activated_tracks = []
        refind_tracks = []
        lost_tracks = []
        removed_tracks = []
        
        if len(detections) > 0:
            # 分离高低置信度检测
            detections = np.array(detections)
            scores = detections[:, 4]
            
            remain_inds = scores > self.track_thresh
            inds_low = scores > 0.1
            inds_high = scores < self.track_thresh
            
            inds_second = np.logical_and(inds_low, inds_high)
            dets_second = detections[inds_second]
            dets = detections[remain_inds]
            
            # 转换为 STrack
            detections_high = [STrack(det[:4], det[4]) for det in dets]
            detections_low = [STrack(det[:4], det[4]) for det in dets_second]
        else:
            detections_high = []
            detections_low = []
        
        # 第一步：高分检测与已跟踪目标匹配
        unconfirmed = []
        tracked_tracks = []
        for track in self.tracked_tracks:
            if not track.is_activated:
                unconfirmed.append(track)
            else:
                tracked_tracks.append(track)
        
        # 已跟踪目标与高分检测匹配
        track_pool = tracked_tracks
        dists = iou_distance(track_pool, detections_high)
        matches, u_track, u_detection = linear_assignment(dists, thresh=self.match_thresh)
        
        for itracked, idet in matches:
            track = track_pool[itracked]
            det = detections_high[idet]
            track.update(det, self.frame_id)
            activated_tracks.append(track)
        
        # 第二步：低分检测与剩余跟踪匹配
        detections_left = [detections_high[i] for i in u_detection]
        dists = iou_distance(self.lost_tracks, detections_left)
        matches, u_lost, u_detection = linear_assignment(dists, thresh=0.5)
        
        for ilost, idet in matches:
            track = self.lost_tracks[ilost]
            det = detections_left[idet]
            track.re_activate(det, self.frame_id, new_id=False)
            refind_tracks.append(track)
        
        # 处理未匹配的跟踪
        for it in u_track:
            track = track_pool[it]
            track.mark_lost()
            lost_tracks.append(track)
        
        # 处理未确认的跟踪
        detections_left = [detections_high[i] for i in u_detection]
        dists = iou_distance(unconfirmed, detections_left)
        matches, u_unconfirmed, u_detection = linear_assignment(dists, thresh=0.7)
        
        for itracked, idet in matches:
            unconfirmed[itracked].update(detections_left[idet], self.frame_id)
            activated_tracks.append(unconfirmed[itracked])
        
        for it in u_unconfirmed:
            track = unconfirmed[it]
            track.mark_removed()
            removed_tracks.append(track)
        
        # 初始化新跟踪
        for inew in u_detection:
            track = detections_left[inew]
            if track.score < self.track_thresh:
                continue
            track.activate(self.frame_id)
            activated_tracks.append(track)
        
        # 更新丢失跟踪状态
        for track in self.lost_tracks:
            if self.frame_id - track.frame_id > self.track_buffer:
                track.mark_removed()
                removed_tracks.append(track)
        
        # 更新跟踪列表
        self.tracked_tracks = [t for t in self.tracked_tracks if t.state == 'tracked']
        self.tracked_tracks = activated_tracks + refind_tracks
        self.lost_tracks = [t for t in self.lost_tracks if t.state == 'lost']
        self.lost_tracks.extend(lost_tracks)
        self.removed_tracks.extend(removed_tracks)
        
        # 返回活跃跟踪
        output_tracks = [track for track in self.tracked_tracks if track.is_activated]
        
        return output_tracks

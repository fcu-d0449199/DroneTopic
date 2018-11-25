# -*- coding: utf-8 -*-
from __future__ import division
import cv2
import numpy as np
from matplotlib import pyplot as plt


class FeatureMatching:
    # 官方教程的目標圖片是query image
    def __init__(self, query_image='data/query.jpg'):
        # 建立SURF探測器，並設置Hessian閾值，由於效果不好，我改成了SIFT方法
        # self.min_hessian = 400（surf方法使用）
        # self.surf = cv2.xfeatures2d.SURF_create(min_hessian)
        # 比對圖SIFT特徵獲取
        self.sift = cv2.xfeatures2d.SIFT_create()
        self.img_query = cv2.imread(query_image, 0)
        # 讀取一個目標模板
        if self.img_query is None:
            print("Could not find train image " + query_image)
            raise SystemExit
        self.shape_query = self.img_query.shape[:2]  # 注意，rows，cols，對應的是y和x，後面的角點坐標的x,y要搞清楚
        #  detectAndCompute函數返回關鍵點和描述符
        self.key_query, self.desc_query = self.sift.detectAndCompute(self.img_query, None)
        # 設置FLANN對象
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        # 保存最後一次計算的單應矩陣
        self.last_hinv = np.zeros((3, 3))
        # 保存沒有找到目標的幀的數量
        self.num_frames_no_success = 0
        # 最大連續沒有找到目標的幀的次數
        self.max_frames_no_success = 5
        self.max_error_hinv = 50.
        # 防止第一次檢測到時由於單應矩陣變化過大而退出
        self.first_frame = True

    def _extract_features(self, frame):
        # self.min_hessian = 400
        # sift = cv2.xfeatures2d.SURF_create(self.min_hessian)
        sift = cv2.xfeatures2d.SIFT_create()
        #  detectAndCompute函數返回關鍵點和描述符，mask為None
        key_train, desc_train = sift.detectAndCompute(frame, None)
        return key_train, desc_train

    # FLANN方法特徵匹配
    def _match_features(self, desc_frame):
        # 函數返回一個訓練集和詢問集的一致性列表
        matches = self.flann.knnMatch(self.desc_query, desc_frame, k=2)
        # 丟棄壞的匹配
        good_matches = []
        # matches中每個元素是兩個對象，分別是與測試的點距離最近的兩個點的資訊
        # 留下距離更近的那個匹配點
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)
        return good_matches

    # Homography取得映射公式
    def _detect_corner_points(self, key_frame, good_matches):
        # 將所有好的匹配的對應點的坐標存儲下來
        src_points = np.float32([self.key_query[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        dst_points = np.float32([key_frame[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)
        H, mask = cv2.findHomography(src_points, dst_points, cv2.RANSAC, 5.0)
        matchesMask = mask.ravel().tolist()
        # 有了H單應性矩陣，我們可以查看源點被映射到img_query中的位置
        # src_corners = np.float32([(0, 0), (self.shape_train[1], 0), (self.shape_train[1], self.shape_train[0]),
        #                           (0, self.shape_train[0])]).reshape(-1, 1, 2)
        h, w = self.img_query.shape[:2]
        src_corners = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        # perspectiveTransform返回點的列表
        dst_corners = cv2.perspectiveTransform(src_corners, H)
        return dst_corners, H, matchesMask

    def match(self, frame, query_image):
        x = y = 0
        img_train = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        shape_train = img_train.shape[:2]

        # 獲得好的matches
        key_train, desc_train = self._extract_features(img_train)
        good_matches = self._match_features(desc_train)

        # 為了讓RANSAC方法可以儘快工作，至少需要4個好的匹配，否則視為匹配失敗
        if len(good_matches) < 4:
            self.num_frames_no_success += 1
            return False, x, y
        # 畫出匹配的點
        #img_match = cv2.drawMatchesKnn(self.img_query, self.key_query, img_train, key_train, [good_matches], None,
        #                              flags=2)
        #plt.imshow(img_match), plt.show()

        # 在query_image中找到對應的角點
        dst_corners, Hinv, matchesMask = self._detect_corner_points(key_train, good_matches)
        # 如果這些點位置距離圖片內太遠（至少20像素），那麼意味著我們沒有找到我們感興趣
        # 的目標或者說是目標沒有完整的出現在圖片內，對於這兩種情況，我們都視為False
        dst_ravel = dst_corners.ravel()
        if (dst_ravel > shape_train[0] + 20).any() and (dst_ravel > -20).any() \
                and (dst_ravel > shape_train[1] + 20).any():
            self.num_frames_no_success += 1
            return False, x, y

        # 如果4個角點沒有圍出一個合理的四邊形，意味著我們可能沒有找到我們的目標。
        # 通過行列式計算四邊形面積
        area = 0.
        for i in range(0, 4):
            D = np.array([[1., 1., 1.],
                          [dst_corners[i][0][0], dst_corners[(i + 1) % 4][0][0], dst_corners[(i + 2) % 4][0][0]],
                          [dst_corners[i][0][1], dst_corners[(i + 1) % 4][0][1], dst_corners[(i + 2) % 4][0][1]]])
            area += abs(np.linalg.det(D)) / 2.
        area /= 2.
        # 以下注釋部分是書中的計算方式，我使用時是錯誤的
        # for i in range(0, 4):
        #     next_i = (i + 1) % 4
        #     print(dst_corners[i][0][0])
        #     print(dst_corners[i][0][1])
        #     area += (dst_corners[i][0][0] * dst_corners[next_i][0][1] - dst_corners[i][0][1] * dst_corners[next_i][0][
        #         0]) / 2.
        # 如果面積太大或太小，將它排除
        if area < np.prod(shape_train) / 16. or area > np.prod(shape_train) / 2.:
            self.num_frames_no_success += 1
            return False, x, y

        # 如果我們此時發現的單應性矩陣和上一次發現的單應性矩陣變化太大，意味著我們可能找到了
        # 另一個對象，這種情況我們丟棄這個幀並返回False
        # 這裡要用到self.max_frames_no_success的，作用就是距離上一次發現的單應性矩陣
        # 不能太久時間，如果時間過長的話，完全可以將上一次的hinv拋棄，使用當前計算得到
        # 的Hinv
        recent = self.num_frames_no_success < self.max_frames_no_success
        similar = np.linalg.norm(Hinv - self.last_hinv) < self.max_error_hinv
        if recent and not similar and not self.first_frame:
            self.num_frames_no_success += self.num_frames_no_success
            return False, x, y
        # 第一次檢測標誌置否
        self.first_frame = False
        self.num_frames_no_success = 0
        self.last_hinv = Hinv

        draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                           singlePointColor=None,
                           matchesMask=matchesMask,  # draw only inliers
                           flags=2)

        img_dst = cv2.polylines(img_train, [np.int32(dst_corners)], True, (0, 255, 255), 5, cv2.LINE_AA)
        img_dst = cv2.drawMatches(self.img_query, self.key_query, img_dst, key_train, good_matches, None,
                                  **draw_params)

        query_image = cv2.cvtColor(query_image, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(8, 8))
        plt.subplot(2, 2, 1)
        plt.title("goal image")
        plt.imshow(query_image)

        plt.subplot(2, 2, 3)
        plt.title("match inf")
        plt.imshow(img_dst)
        #plt.show()

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        plt.subplot(2, 2, 2)
        plt.title("camera's image")
        plt.imshow(frame)

        img_dst2 = cv2.polylines(frame, [np.int32(dst_corners)], True, (55,255,155), 5, cv2.LINE_AA)
        # mark the picture's center point
        (h, w) = img_dst2.shape[:2]
        center = (w // 2, h // 2)
        cv2.circle(img_dst2, center, 10, (255, 0, 0), -1)

        # mark the position point(Splicing comparison chart)
        p_center = (np.int32(dst_corners[0][0][0]) + ((np.int32(dst_corners[2][0][0]) - np.int32(dst_corners[0][0][0])) // 2),
                    np.int32(dst_corners[0][0][1]) + ((np.int32(dst_corners[2][0][1]) - np.int32(dst_corners[0][0][1])) // 2))
        # mark the comparison chart's center point
        cv2.circle(img_dst2, p_center, 10, (0, 0, 255), -1)

        #draw the line
        cv2.line(img_dst2, center, p_center, (256, 256, 3), 5)
        #if (np.int32(center[1]) - np.int32(p_center[1]) == 1 and np.int32(center[0]) - np.int32(p_center[0]) == 1): print('mission clear!!!')
        #else:
        #    if(np.int32(p_center[1]) - np.int32(center[1]) + 1 > 0): print '>>down:', np.int32(p_center[1]) - np.int32(center[1]) + 1
        #    elif(np.int32(p_center[1]) - np.int32(center[1]) + 1 < 0): print '>>up:', np.int32(center[1]) - np.int32(p_center[1]) - 1
        #    if(np.int32(p_center[0]) - np.int32(center[0]) + 1 > 0): print '>>right:', np.int32(p_center[0]) - np.int32(center[0]) + 1
        #    elif(np.int32(p_center[0]) - np.int32(center[0]) + 1 < 0): print '>>left:', np.int32(center[0]) - np.int32(p_center[0]) - 1
        x = np.int32(p_center[0]) - np.int32(center[0]) + 1
        y = np.int32(center[1]) - np.int32(p_center[1]) - 1
        #print(x, y)

        #Calculate size
        #(H, W) = query_image.shape[:2]
        #size = abs(np.int32(dst_corners[2][0][0]) - np.int32(dst_corners[0][0][0])) / W * 100
        #print 'size:', size, '%'

        plt.subplot(2, 2, 4)
        plt.title('position inf')
        plt.imshow(img_dst2)

        plt.show(block=False)
        plt.pause(1)

        return True, x, y#, size
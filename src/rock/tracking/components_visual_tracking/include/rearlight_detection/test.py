from matplotlib import markers
import numpy as np
import cv2
import matplotlib.pyplot as plt

colors = [np.array([255, 0, 0]), np.array([0, 255, 0]), np.array([0, 0, 255]), np.array([255, 255, 0]), \
    np.array([255, 0, 255]), np.array([0, 255, 255])]
K = 8
TOPK = 4

def test(image):
    img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    last_bbox_size = (image.shape[0] / 2, image.shape[1] / 2)
    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_mask_1 = cv2.inRange(img_hsv, (0, 0, 0), (10, 255, 255))
    red_mask_2 = cv2.inRange(img_hsv, (170, 0, 0), (180, 255, 255))
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    img_masked = cv2.bitwise_and(image, image, mask=red_mask)
    img_masked_rgb = cv2.cvtColor(img_masked, cv2.COLOR_BGR2RGB)
    
    img_YCrCb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    # weighting_chann = 0.1 * img_YCrCb[:, :, 0] + 0.9 * img_YCrCb[:, :, 1]
    weighting_chann = 0.9 * img_YCrCb[:, :, 0] + 0.1 * img_YCrCb[:, :, 1]
    weighting_chann[red_mask == 0] = 0.0

    non_zero_mask = np.zeros(weighting_chann.shape, dtype=np.int)
    non_zero_mask[red_mask != 0] = 1
    col_non_zero_cnt = np.sum(non_zero_mask, axis=0).flatten()
    row_non_zero_cnt = np.sum(non_zero_mask, axis=1).flatten()
    # NOTE: 分母可能为0!!!
    col_avg = np.sum(weighting_chann, axis=0).flatten() / col_non_zero_cnt
    row_avg = np.sum(weighting_chann, axis=1).flatten() / row_non_zero_cnt
    col_avg[np.isnan(col_avg)] = 0
    row_avg[np.isnan(row_avg)] = 0
    col_avg[np.isinf(col_avg)] = 0
    row_avg[np.isinf(row_avg)] = 0
    # col_avg = np.mean(weighting_chann, axis=0)
    # row_avg = np.mean(weighting_chann, axis=1)
    col_top_k_idx = col_avg.argsort()[::-1][:TOPK]
    row_top_k_idx = row_avg.argsort()[::-1][:TOPK]
    print(col_top_k_idx, row_top_k_idx)
    roi_center = np.array([img_masked.shape[0] / 2, img_masked.shape[1] / 2])
    print(roi_center)
    min_dis = 1e9
    best_candidate = (None, None)
    for i in range(TOPK):
        for j in range(TOPK):
            pt = np.array([row_top_k_idx[i], col_top_k_idx[j]])
            dis = np.linalg.norm(pt - roi_center)
            if dis < min_dis:
                best_candidate = (col_top_k_idx[j], row_top_k_idx[i])
    # best_candidate = (int(image.shape[1] / 2), int(image.shape[0] / 2))
    print(best_candidate)

    plt.figure()
    plt.bar(range(col_avg.shape[0]), col_avg)
    plt.title('col_avg')
    plt.figure()
    plt.barh(range(row_avg.shape[0]), row_avg)
    plt.title('row_avg')
    plt.figure()
    plt.imshow(img_masked_rgb)
    plt.show()
    # cv2.imshow('test', image)
    # cv2.waitKey(10000)
    
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    flags = cv2.KMEANS_PP_CENTERS
    samples = image.reshape(-1, 3).astype(np.float32) / 255
    # grid_x, grid_y = np.meshgrid(np.arange(0, 1, 1 / img_masked.shape[0]), np.arange(0, 1, 1 / img_masked.shape[1]))
    # grid_x = grid_x.reshape(-1, 1).astype(np.float32)
    # grid_y = grid_y.reshape(-1, 1).astype(np.float32)
    # samples = np.concatenate([samples, grid_x, grid_y], axis=1)
    compactness, labels, centers = cv2.kmeans(samples, K, None, criteria, 10, flags)
    labels = labels.reshape(image.shape[0], image.shape[1])
    best_candidate_label = labels[best_candidate[1], best_candidate[0]]
    best_candidata_bgr = centers[best_candidate_label]
    
    cc = []
    for i in range(K):
        cluster_mask = np.zeros(labels.shape, dtype=np.uint8)
        cluster_mask[labels == i] = 255
        cc_num, cc_labels, cc_stats, cc_centroids = cv2.connectedComponentsWithStats(cluster_mask)
        for j in range(1, cc_num):
            cluster_cc_mask = (cc_labels == j)
            dis = np.linalg.norm([best_candidate[0] - cc_centroids[j][0], best_candidate[1] - cc_centroids[j][1]])
            bgr_dis = np.linalg.norm(centers[i] - best_candidata_bgr)
            cluster_cc_info = (cluster_cc_mask, centers[i], cc_stats[i], dis, bgr_dis) # (mask, center of BGR, stats)
            cc.append(cluster_cc_info)
    cc.sort(key=lambda x: x[3])
    final_rect = (None, None, None, None)
    all_bgr_dis = [c[4] for c in cc]
    max_bgr_dis = max(all_bgr_dis)
    selected = list(range(len(cc)))
    # for i in range(len(cc)):
    #     bgr_dis = cc[i][4]
    #     if bgr_dis < 100.0 * max_bgr_dis:
    #         c_left_x = cc[i][2][0]
    #         c_top_y = cc[i][2][1]
    #         c_width = cc[i][2][2]
    #         c_height = cc[i][2][3]
    #         if final_rect[0] is None:
    #             if c_width / last_bbox_size[1] > 1.2 or c_height / last_bbox_size[0] > 1.2:
    #                 continue
    #             final_rect = (c_left_x, c_top_y, c_width, c_height)
    #             selected.append(i)
    #         else:
    #             last_rect = final_rect
    #             last_lt_x = last_rect[0]
    #             last_lt_y = last_rect[1]
    #             last_rb_x = last_rect[0] + last_rect[2]
    #             last_rb_y = last_rect[1] + last_rect[3]
    #             curr_lt_x = c_left_x
    #             curr_lt_y = c_top_y
    #             curr_rb_x = c_left_x + c_width
    #             curr_rb_y = c_top_y + c_height
                
    #             final_lt_x = min(last_lt_x, curr_lt_x)
    #             final_lt_y = min(last_lt_y, curr_lt_y)
    #             final_rb_x = max(last_rb_x, curr_rb_x)
    #             final_rb_y = max(last_rb_y, curr_rb_y)
    #             final_width = final_rb_x - final_lt_x
    #             final_height = final_rb_y - final_lt_y
    #             if final_width / last_bbox_size[1] > 1.2 or final_height / last_bbox_size[0] > 1.2:
    #                 continue
                
                
    #             final_rect = (final_lt_x, final_lt_y, final_width, final_height)
    #             selected.append(i)

    clustered = image
    for i in range(len(cc)):
        if i not in selected:
            continue
        color = np.random.randint(0, 255, (3,))
        clustered[cc[i][0], :] = color
    # cv2.imshow('clustering', clustered)
    # cv2.waitKey()
    clustered_rgb = cv2.cvtColor(clustered, cv2.COLOR_BGR2RGB)
    
    # cv2.circle(img_rgb, best_candidate, radius=2, color=(0, 0, 0))
    # cv2.circle(clustered_rgb, best_candidate, radius=2, color=(0, 0, 0))
    plt.subplot(1, 2, 1)
    plt.imshow(img_rgb)
    # plt.title('车尾灯局部图像')
    plt.subplot(1, 2, 2)
    plt.imshow(clustered_rgb)
    # plt.title('车尾灯局部色彩量化')
    plt.show()

    # img_show = image
    # cv2.circle(img_show, best_candidate, radius=2, color=(0, 255, 0))
    # cv2.imshow('t', img_show)
    # cv2.waitKey(10000)



if __name__ == '__main__':
    image = cv2.imread('/home/wuhr/Workspace/cyberc3_platooning/init_images/init.jpg')
    # image = cv2.imread('/media/wuhr/data/platoon_dataset/2022_10_20/platoon_dataset/image/1666254511.0099747.jpg')
    # test(image[451:477, 897:933, :])
    img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    plt.imshow(img_rgb)
    plt.show()
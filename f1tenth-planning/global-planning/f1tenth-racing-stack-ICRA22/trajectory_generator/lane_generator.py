import cv2
import numpy as np
import matplotlib.pyplot as plt
import csv
import yaml
import os
 
WIDTH = 0.27  # (m)
WHEEL_LENGTH = 0.325  # (m)
MAX_STEER = 0.6 # (rad)
map_name = "0816"
lane_colors = [(0, 0, 255),
               (0, 255, 255),
               (0, 255, 0),
               (255, 255, 0),
               (255, 0, 0),
               (255, 0, 255)]


def show_result(imgs, title):
    if not imgs:
        return False
    height, width = imgs[0].shape[:2]
    w_show = 800
    scale_percent = float(w_show / width)
    h_show = int(scale_percent * height)
    dim = (w_show, h_show)
    img_resizes = []
    for img in imgs:
        img_resize = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        img_resizes.append(img_resize)
    img_show = cv2.hconcat(img_resizes)
    cv2.imshow(title, img_show)

    print("Press Q to abort / other keys to proceed")

    if cv2.waitKey(0) == ord('q'):
        cv2.destroyAllWindows()
        return False 
    else:
        cv2.destroyAllWindows()
        return True


def reorder_vertex(image, lane, plot=False):
    if len(lane) == 0:
        return np.array([])
        
    path_img = np.zeros_like(image)
    for idx in range(len(lane)):
        # 오류 해결: np.array를 정수형 튜플로 변환하여 전달
        center = tuple(lane[idx].astype(int))
        cv2.circle(path_img, center, 1, (255, 255, 255), -1)
        
    curr_kernel = np.ones((3, 3), np.uint8)
    iter_cnt = 0
    while True:
        if iter_cnt > 50:
            print("Warning: Vertex reordering took too long. Check for gaps or noise.")
            exit(0)
        curr_contours, curr_hierarchy = cv2.findContours(path_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(curr_contours) == 2 and curr_hierarchy[0][-1][-1] == 0:
            break
        path_img = cv2.dilate(path_img, curr_kernel, iterations=1)
        iter_cnt += 1
    
    path_img = cv2.ximgproc.thinning(path_img)
    curr_contours, curr_hierarchy = cv2.findContours(path_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if plot and not show_result([path_img], title="track"):
        exit(0)
        
    if not curr_contours:
        return np.array([])
        
    return np.squeeze(curr_contours[0])


def remove_duplicate(arr):
    _, idx = np.unique(arr, return_index=True)
    return arr[np.sort(idx)]


def draw_lane(img, lane, color=(0, 0, 255), show_arrow=True):
    h, w = img.shape[:2]
    lane = lane[:, 0:2].astype(int)
    for idx in range(len(lane) - 1):
        # 튜플 변환 추가 (안정성)
        pt1 = tuple(lane[idx])
        pt2 = tuple(lane[idx + 1])
        cv2.line(img, pt1, pt2, color, 1)
    
    cv2.line(img, tuple(lane[-1]), tuple(lane[0]), color, 1)  # connect tail to head

    if show_arrow:
        start = tuple(lane[0])
        vec = lane[1] - lane[0]
        direction = vec / (np.linalg.norm(vec) + 1e-8)
        end = lane[0] + direction * min([h, w]) * 0.1
        end = tuple(end.astype(int))
        cv2.arrowedLine(img, start, end, (238, 130, 238), 1, tipLength=0.2)


def transform_coords(path, height, s, tx, ty):
    if len(path) == 0:
        return path
    new_path_x = path[:, 0] * s + tx
    new_path_y = (height - path[:, 1]) * s + ty
    if path.shape[1] > 2:
        new_right_dist = path[:, 2] * s # scale 변수 대신 s 사용
        new_left_dist = path[:, 3] * s
        return np.vstack((new_path_x, new_path_y, new_right_dist, new_left_dist)).T
    else:
        return np.vstack((new_path_x, new_path_y)).T 


def save_csv(data, csv_name, header=None):
    with open(csv_name, mode='w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        if header:
            csv_writer.writerow(header)
        for line in data:
            csv_writer.writerow(line.tolist()) 


if __name__ == "__main__":
    # 경로 설정 (사용자 환경 유지)
    module = os.path.dirname(os.path.abspath(__file__))
    config_file = "/home/kimseokjin/lane_ws/src/f1tenth-racing-stack-ICRA22/config/params.yaml"
    
    with open(config_file, 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
    input_map = parsed_yaml["map_name"]
    num_lanes = parsed_yaml["num_lanes"]
    clockwise = parsed_yaml["clockwise"]
    inner_safe_dist = parsed_yaml["inner_safe_dist"]
    outer_safe_dist = parsed_yaml["outer_safe_dist"]
    opp_safe_dist = parsed_yaml["opp_safe_dist"]

    yaml_file = "/home/kimseokjin/lane_ws/src/f1tenth-racing-stack-ICRA22/maps/0816.yaml"
    with open(yaml_file, 'r') as stream:
        parsed_yaml = yaml.safe_load(stream)
    scale = parsed_yaml["resolution"]
    offset_x = parsed_yaml["origin"][0]
    offset_y = parsed_yaml["origin"][1]  

    lane_ratios = np.arange(1, num_lanes + 1) / np.arange(num_lanes, 0, -1)
    if not np.any(lane_ratios == 1.0):
        lane_ratios = np.append(lane_ratios, 1.0)

    img_path = "/home/kimseokjin/lane_ws/src/f1tenth-racing-stack-ICRA22/maps/0816.png"
    input_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if input_img is None:
        print(f"Error: Could not load image at {img_path}")
        exit(0)
    h, w = input_img.shape[:2]

    print("Metadata: ", h, w, scale, offset_x, offset_y)

    output_img = ~input_img
    ret, output_img = cv2.threshold(output_img, thresh=127, maxval=255, type=cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(output_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    for i, contour in enumerate(contours):
        if cv2.contourArea(contour) < 70:
            cv2.fillPoly(output_img, pts=[contour], color=(0, 0, 0))

    kernel = np.ones((5, 5), np.uint8)
    output_img = cv2.dilate(output_img, kernel, iterations=1)
    output_img = cv2.ximgproc.thinning(output_img)

    if not show_result([input_img, output_img], title="input & output"):
        exit(0)

    contours, hierarchy = cv2.findContours(output_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    parents = hierarchy[0][:, 3]
    assert np.max(parents) >= 1

    node = np.argmax(parents)
    tree_indices = []
    while node != -1:
        tree_indices.append(node)
        node = parents[node]
    tree_indices.reverse()

    outer_bound = contours[tree_indices[1]]
    inner_bound = contours[tree_indices[2]]

    track_img = np.zeros_like(output_img)
    track_contours = [contours[i] for i in tree_indices]
    cv2.drawContours(track_img, track_contours, -1, (255, 255, 255), 1)

    if not show_result([track_img], title="bounds"):
        exit(0)

    print("Performing Euclidean distance transform...")
    X, Y = np.meshgrid(np.arange(w), np.arange(h))
    X = X.flatten().tolist()
    Y = Y.flatten().tolist()
    valid_pts = []
    opp_track_pts = []
    opp_inner_bound, opp_outer_bound = [], []
    
    for (x, y) in zip(X, Y):
        outer_dist = cv2.pointPolygonTest(outer_bound, (x, y), True)
        inner_dist = cv2.pointPolygonTest(inner_bound, (x, y), True)
        if outer_dist > outer_safe_dist / scale and inner_dist < -inner_safe_dist / scale:
            ratio = np.abs(inner_dist) / (np.abs(outer_dist) + 1e-8)
            valid_pts.append([x, y, inner_dist, outer_dist, ratio])
        if outer_dist > opp_safe_dist / scale and inner_dist < -opp_safe_dist / scale:
            opp_track_pts.append([x, y])
        if abs(outer_dist - opp_safe_dist / scale) < 2:
            opp_outer_bound.append([x, y])
        if abs(inner_dist + opp_safe_dist / scale) < 2:
            opp_inner_bound.append([x, y])
            
    valid_pts = np.array(valid_pts)
    opp_track_pts = np.array(opp_track_pts)
    opp_outer_bound = np.array(opp_outer_bound)
    opp_inner_bound = np.array(opp_inner_bound)
    
    # 여기서 reorder_vertex 호출
    opp_outer_bound = reorder_vertex(output_img, opp_outer_bound)
    opp_inner_bound = reorder_vertex(output_img, opp_inner_bound)

    print("Plotting track bounds...")
    track_img = cv2.cvtColor(~output_img, cv2.COLOR_GRAY2BGR)
    for pt in valid_pts:
        cv2.circle(track_img, (int(pt[0]), int(pt[1])), 1, (0, 0, 255), 1)
        
    opp_bound_img = cv2.cvtColor(~output_img, cv2.COLOR_GRAY2BGR)
    # 선 그리기 시 튜플 변환 적용
    if len(opp_outer_bound) > 0:
        for i in range(len(opp_outer_bound)):
            cv2.line(opp_bound_img, tuple(opp_outer_bound[i - 1]), tuple(opp_outer_bound[i]), (0, 0, 255), 1)
    if len(opp_inner_bound) > 0:
        for i in range(len(opp_inner_bound)):
            cv2.line(opp_bound_img, tuple(opp_inner_bound[i - 1]), tuple(opp_inner_bound[i]), (0, 0, 255), 1)
            
    if not show_result([track_img, opp_bound_img], title="track"):
        exit(0)

    lanes = []
    for idx in range(len(lane_ratios)):
        print(f"Calculating lane {idx}")
        valid_ratio = (np.abs(valid_pts[:, -1] - lane_ratios[idx]) < lane_ratios[idx] / 10)
        lane = valid_pts[valid_ratio, 0:2].astype(int)
        lane = reorder_vertex(output_img, lane)
        if clockwise:
            lane = np.flipud(lane)
        left_dists, right_dists = [], []
        for (x, y) in lane:
            outer_d = cv2.pointPolygonTest(outer_bound, (int(x), int(y)), True)
            inner_d = cv2.pointPolygonTest(inner_bound, (int(x), int(y)), True)
            outer_d = outer_d - outer_safe_dist / scale
            inner_d = abs(inner_d) - inner_safe_dist / scale
            if clockwise:
                left_dists.append(outer_d)
                right_dists.append(inner_d)
            else:
                left_dists.append(inner_d)
                right_dists.append(outer_d)
        lane = np.vstack((lane.T, right_dists, left_dists)).T
        lanes.append(lane)

    print("Plotting lanes...")
    res_img = cv2.cvtColor(~output_img, cv2.COLOR_GRAY2BGR)
    for idx in range(len(lanes)):
        draw_lane(res_img, lanes[idx], color=lane_colors[idx % len(lane_colors)])

    if not show_result([res_img], title="track"):
        exit(0)

    print("Scaling...")
    for idx in range(len(lanes)):
        lanes[idx] = transform_coords(lanes[idx], h, scale, offset_x, offset_y)
    opp_track_pts = transform_coords(opp_track_pts, h, scale, offset_x, offset_y)

    plt.figure(figsize=(10, 8))
    for idx in range(len(lanes)):
        plt.plot(lanes[idx][:, 0], lanes[idx][:, 1], 'o', markersize=1, label=f'lane_{idx}')
    plt.axis('equal')
    plt.legend()
    # plt.show() # 필요시 주석 해제

    print("Saving result...")
    os.makedirs(os.path.join(module, "outputs"), exist_ok=True)
    csv_folder = os.path.join(module, "outputs", input_map)
    os.makedirs(csv_folder, exist_ok=True)
    
    for idx in range(len(lanes)):
        if lane_ratios[idx] == 1.0:
            csv_path = os.path.join(csv_folder, "centerline.csv")
        else:
            csv_path = os.path.join(csv_folder, f"lane_{idx}.csv")
        save_csv(lanes[idx], csv_path, header=["#x_m", "y_m", "w_tr_right_m", "w_tr_left_m"])

    save_csv(opp_track_pts, os.path.join(csv_folder, "track.csv"))
    np.save(os.path.join(csv_folder, "track"), opp_track_pts)
    save_csv(opp_outer_bound, os.path.join(csv_folder, "outer_bound.csv"))
    save_csv(opp_inner_bound, os.path.join(csv_folder, "inner_bound.csv"))

    print("Finish") 
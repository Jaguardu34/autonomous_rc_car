from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO("yolo_models/yolo11m-road-seg.pt")
model.to("cuda")
cap = cv2.VideoCapture("Python/test_images/video4.mp4")

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.resize(frame, (640, 480))
    h, w, _ = frame.shape
    results = model(frame, stream=False, verbose=False)
    r = results[0]
    
    if r.masks is None:
        cv2.imshow("Trajectory", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        continue
    
    # === SEGMENTATION ===
    mask = r.masks.data[0].cpu().numpy()
    mask = cv2.resize(mask, (w, h))
    mask = (mask > 0.5).astype(np.uint8)
    
    # overlay gris
    mask_gray = (mask * 255).astype(np.uint8)
    mask_gray_bgr = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR)
    frame = cv2.addWeighted(frame, 1.0, mask_gray_bgr, 0.4, 0)
    
    # === CALCUL DU HAUT DE LA BOUNDING BOX ===
    # Trouver la ligne la plus haute où il y a de la route
    rows_with_road = np.where(mask.any(axis=1))[0]
    
    if len(rows_with_road) == 0:
        cv2.imshow("Trajectory", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        continue
    
    y_top = rows_with_road[0]  # Ligne la plus haute avec de la route
    y_bottom = h  # Bas de l'image
    
    # Optionnel : dessiner la bounding box pour visualiser
    y_box_bottom = rows_with_road[-1]
    cols_with_road = np.where(mask.any(axis=0))[0]
    if len(cols_with_road) > 0:
        x_left = cols_with_road[0]
        x_right = cols_with_road[-1]
        cv2.rectangle(frame, (x_left, y_top), (x_right, y_box_bottom), (255, 255, 0), 2)
    
    # === TRAJECTOIRE ===
    trajectory_points = []
    for y in range(y_top, y_bottom, 20):
        xs = np.where(mask[y] > 0)[0]
        if len(xs) > 0:
            x_center = int(xs.mean())
            trajectory_points.append((x_center, y))
            cv2.circle(frame, (x_center, y), 4, (0, 255, 0), -1)
    
    if len(trajectory_points) > 1:
        cv2.polylines(
            frame,
            [np.array(trajectory_points)],
            isClosed=False,
            color=(0, 0, 255),
            thickness=3
        )
    
    direction = 0
    if len(trajectory_points) >= 2:
        # Point proche (bas)
        p_near = trajectory_points[-1]
        
        # Point lointain : premier tiers de la trajectoire
        # (75% d'anticipation environ)
        far_index = len(trajectory_points) // 5
        p_far = trajectory_points[far_index]
        
        dx = p_far[0] - p_near[0]
        dy = p_near[1] - p_far[1]
        angle_rad = np.arctan2(dx, dy)
        angle_deg = np.degrees(angle_rad)
        
        max_angle = 60.0
        direction = int(np.clip(angle_deg / max_angle, -1.0, 1.0) * 100)
    
    print(f"Direction: {direction}")
    cv2.putText(frame, f"Direction: {direction}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    
    # ligne de référence depuis le haut de la détection jusqu'en bas
    cv2.line(frame, (w // 2, h), (w // 2, y_top), (255, 0, 0), 2)
    
    cv2.imshow("Trajectory", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
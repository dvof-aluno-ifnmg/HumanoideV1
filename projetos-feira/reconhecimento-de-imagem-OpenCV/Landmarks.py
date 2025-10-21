# Braço Esquerdo (5)
l_shoulder = np.array([landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].z])
l_elbow = np.array([landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].z])
l_wrist = np.array([landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].z])
l_pinky = np.array([landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].y, landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].z])
l_index = np.array([landmarks[mp_pose.PoseLandmark.LEFT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.LEFT_INDEX.value].y, landmarks[mp_pose.PoseLandmark.LEFT_INDEX.value].z])
l_thumb = np.array([landmarks[mp_pose.PoseLandmark.LEFT_THUMB.value].x, landmarks[mp_pose.PoseLandmark.LEFT_THUMB.value].y, landmarks[mp_pose.PoseLandmark.LEFT_THUMB.value].z])

# Braço Direito (5)
r_shoulder = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].z])
r_elbow = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].z])
r_wrist = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].z])
r_pinky = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].z])
r_index = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].z])
r_thumb = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_THUMB.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_THUMB.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_THUMB.value].z])

# Tronco (2)
l_hip = np.array([landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].z])
r_hip = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].z])

# Perna Esquerda (4)
l_knee = np.array([landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].z])
l_ankle = np.array([landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].y, landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].z])
l_heel = np.array([landmarks[mp_pose.PoseLandmark.LEFT_HEEL.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HEEL.value].y, landmarks[mp_pose.PoseLandmark.LEFT_HEEL.value].z])
l_foot_index = np.array([landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].y, landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].z])

# Perna Direita (4)
r_knee = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].z])
r_ankle = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].z])
r_heel = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_HEEL.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HEEL.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_HEEL.value].z])
r_foot_index = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].z])
# --- Fim da extração ---

#Superiores
ombEsqY = int(calculate_angle_3d(l_hip,l_shoulder,l_elbow))
ombEsqX = int(calculate_angle_3d(r_shoulder,l_shoulder,l_elbow))
ombEsqZ = int(calculate_angle_3d(l_elbow,l_wrist,l_pinky))# realmente não é medido como os outros (teste)

cotEsq = int(calculate_angle_3d(l_shoulder,l_elbow,l_wrist))

#Inferiores 
coxEsqY = int(calculate_angle_3d(l_shoulder,l_hip,l_knee))
coxEsqX = int(calculate_angle_3d(r_hip,l_hip,l_knee))
coxEsqZ = int(calculate_angle_3d(l_knee,l_ankle,l_foot_index))

joe = int(calculate_angle_3d(l_hip, l_knee, l_ankle))
tornz = int(calculate_angle_3d(l_knee,l_ankle,l_foot_index))





EXAMPLE = int(calculate_angle_3d())

import cv2
import mediapipe as mp
import numpy as np
import math
import serial

SUB_CHAR = '\0'
GRP_CHAR = '\t'
TP_CHAR = '\n'


def calculate_angle_3d(a, b, c):
    """
    Calcula o ângulo entre 3 pontos 3D (em graus).
    O ângulo é calculado na articulação 'b'.

    Args:
        a: Coordenadas do ponto A (ex: ombro) como um array numpy [x, y, z].
        b: Coordenadas do ponto B (ex: cotovelo) como um array numpy [x, y, z].
        c: Coordenadas do ponto C (ex: pulso) como um array numpy [x, y, z].

    Returns:
        O ângulo em graus (entre 0 e 180).
    """
    # 1. Criar os vetores
    # Vetor BA (de B para A)
    vec_ba = a - b
    # Vetor BC (de B para C)
    vec_bc = c - b

    # 2. Calcular o produto escalar
    dot_product = np.dot(vec_ba, vec_bc)

    # 3. Calcular as magnitudes (normas) dos vetores
    norm_ba = np.linalg.norm(vec_ba)
    norm_bc = np.linalg.norm(vec_bc)
    
    # Prevenção de divisão por zero
    if norm_ba == 0 or norm_bc == 0:
        return 0.0

    # 4. Calcular o cosseno do ângulo
    cos_angle = dot_product / (norm_ba * norm_bc)
    
    # 5. Clamp o valor para evitar erros de domínio no acos devido a imprecisões de ponto flutuante
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    # 6. Calcular o ângulo em radianos e converter para graus
    angle_rad = math.acos(cos_angle)
    angle_deg = math.degrees(angle_rad)

    return angle_deg
       

# Inicializa MediaPipe Pose
mp_pose = mp.solutions.pose
pose = mp.solutions.pose.Pose()
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

try: #esse bloco está em implementação
    arduino = serial.Serial('COM7', 115200, timeout=1)
except Exception as e: #falha referente ao sistema
    print(f"arduino não encontrado! Erro: {e}")
    arduino = serial.Serial('/dev/ttyUSB0/', 115200, timeout=1)



while True:
    ret, frame = cap.read()
    if not ret:
        print("Cannot find a camera")
        break

    # Converte para RGB (MediaPipe usa RGB, não BGR)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb) #define a variável results como o processamento de mp.solutions.pose, que é definido como pose
    if results.pose_world_landmarks:
       # --- Extração de todos os 33 World Landmarks ---
        
        # Obtém a lista de landmarks
        landmarks = results.pose_world_landmarks.landmark
        
        # Braço Esquerdo (5)
        l_shoulder = np.array([landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y, landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].z])
        l_elbow = np.array([landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y, landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].z])
        l_wrist = np.array([landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y, landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].z])
        l_pinky = np.array([landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].y, landmarks[mp_pose.PoseLandmark.LEFT_PINKY.value].z])

        # Braço Direito (5)
        r_shoulder = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].z])
        r_elbow = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].z])
        r_wrist = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].z])
        r_pinky = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_PINKY.value].z])

        # Tronco (2)
        l_hip = np.array([landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y, landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].z])
        r_hip = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].z])

        # Perna Esquerda (4)
        l_knee = np.array([landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y, landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].z])
        l_ankle = np.array([landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].y, landmarks[mp_pose.PoseLandmark.LEFT_ANKLE.value].z])
        l_foot_index = np.array([landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].y, landmarks[mp_pose.PoseLandmark.LEFT_FOOT_INDEX.value].z])

        # Perna Direita (4)
        r_knee = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_KNEE.value].z])
        r_ankle = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_ANKLE.value].z])
        r_foot_index = np.array([landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].y, landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].z])
        # --- Fim da extração ---

    # Membros Esquerdos
        #Superiores Esquerdos
        ombEsqY = int(calculate_angle_3d(l_hip,l_shoulder,l_elbow))
        ombEsqX = int(calculate_angle_3d(r_shoulder,l_shoulder,l_elbow))
        ombEsqZ = int(calculate_angle_3d(l_elbow,l_wrist,l_pinky)) # realmente não é medido como os outros (teste)

        cotEsq = int(calculate_angle_3d(l_shoulder,l_elbow,l_wrist))

        #Inferiores Esquerdos
        coxEsqY = int(calculate_angle_3d(l_shoulder,l_hip,l_knee))
        coxEsqX = int(calculate_angle_3d(r_hip,l_hip,l_knee))

        joeEsq = int(calculate_angle_3d(l_hip, l_knee, l_ankle))
        tornEsq = int(calculate_angle_3d(l_knee,l_ankle,l_foot_index))

    # Membros Direitos:
        # Superiores Direitos
        ombDirY = int(calculate_angle_3d(r_hip, r_shoulder, r_elbow))
        ombDirX = int(calculate_angle_3d(l_shoulder, r_shoulder, r_elbow))
        ombDirZ = int(calculate_angle_3d(r_elbow, r_wrist, r_pinky))

        cotDir = int(calculate_angle_3d(r_shoulder, r_elbow, r_wrist))

        # Inferiores Direitos
        coxDirY = int(calculate_angle_3d(r_shoulder, r_hip, r_knee))
        coxDirX = int(calculate_angle_3d(l_hip, r_hip, r_knee))
        
        joeDir = int(calculate_angle_3d(r_hip, r_knee, r_ankle))
        tornDir = int(calculate_angle_3d(r_knee, r_ankle, r_foot_index))

        buffer = [
            # Superiores Esquerdos
            ombEsqY, ombEsqX, ombEsqZ, SUB_CHAR,
            cotEsq, SUB_CHAR, GRP_CHAR,
            # Inferiores Esquerdos  
            coxEsqY, coxEsqX, SUB_CHAR,
            joeEsq, SUB_CHAR,
            tornEsq, SUB_CHAR,GRP_CHAR, TP_CHAR,
            # Superiores Direitos
            ombDirY, ombDirX, ombDirZ, SUB_CHAR,
            cotDir, SUB_CHAR, GRP_CHAR,
            # Inferiores Direitos
            coxDirY, coxDirX, SUB_CHAR,
            joeDir, SUB_CHAR,
            tornDir, SUB_CHAR, GRP_CHAR, TP_CHAR
        ]

        print(f"{coxEsqX},{coxEsqY}\n")
        try:
            arduino.write(f"{buffer}".encode())
        except:
            print("arduino não encontrado")

    mp_draw.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    cv2.imshow("Pose Detection", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC para sair
        break

cap.release()
cv2.destroyAllWindows()
import pybullet as p    
import pybullet_data
import time
import torch
import torch.nn as nn
import math
from neural_network import init_net, alt_net

#Número de juntas do humanoide: 15
#Junta 0: root, tipo: 4
#Junta 1: chest, tipo: 2
#Junta 2: neck, tipo: 2
#Junta 3: right_shoulder, tipo: 2
#Junta 4: right_elbow, tipo: 0
#Junta 5: right_wrist, tipo: 4
#Junta 6: left_shoulder, tipo: 2
#Junta 7: left_elbow, tipo: 0
#Junta 8: left_wrist, tipo: 4
#Junta 9: right_hip, tipo: 2
#Junta 10: right_knee, tipo: 0
#Junta 11: right_ankle, tipo: 2
#Junta 12: left_hip, tipo: 2
#Junta 13: left_knee, tipo: 0
#Junta 14: left_ankle, tipo: 2

#junta tipo 0: REVOLUTE (ex: cotovelo, joelho)
#junta tipo 2: SPHERICAL (ex: ombro, quadril)
#junta tipo 4: FIXED (tudo que não se move)

#input
train_from_scratch = input("Treinar do zero? (s/n): ").strip().lower()
irender = input("Ativar renderização? (s/n): ").strip().lower()
fps_raw = input("Fps desejado? (0 = sem limite, 240 = tempo real): ").strip()
try:
    fps = float(fps_raw)
except ValueError:
    print("FPS inválido, usando 0 (sem limite).")
    fps = 0.0
if train_from_scratch == "s":
    tfs = True
else:
    tfs = False

if irender == "s":
    rfs = True
else:
    rfs = False

# define se vai ter GUI(renderização em janela) ou não
if rfs == True:
    physicsClient = p.connect(p.GUI)
else:
    physicsClient = p.connect(p.DIRECT)
# define as configurações
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# carrega o chão
planeId = p.loadURDF("plane.urdf")
# carrega o robô humanoide simples (bípede)
startPos = [0, 0, 10]
startOrientation = p.getQuaternionFromEuler([math.pi/2, 0, 0])  # gira 90° no X
robotId = 0
# define o número de juntas, nome e ID
num_joints = p.getNumJoints(robotId)
print("Número de juntas do humanoide:", num_joints)
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    print(f"Junta {i}: {info[1].decode('utf-8')}, tipo: {info[2]}")

#-----------------------------------------------------------------#
#-----------------------------FUNÇÕES-----------------------------#
#-----------------------------------------------------------------#

# le todos os angulos das juntas de um dado objeto Rid
def ler_angulos_juntas(Rid):
    numJ = p.getNumJoints(Rid)
    joint_angles = []
    for i in range(numJ):
        joint_info = p.getJointInfo(Rid, i)
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:
            if joint_type == p.JOINT_SPHERICAL:
                # getJointStateMultiDof returns position (quaternion) em joint_state[0]
                joint_state = p.getJointStateMultiDof(Rid, i)
                quat = joint_state[0]  # (x,y,z,w) provavelmente
                # converte quaternion para euler (roll, pitch, yaw)
                euler = p.getEulerFromQuaternion(quat)
                joint_angles.extend(list(euler))
            else:
                joint_state = p.getJointState(Rid, i)
                angle = joint_state[0]
                joint_angles.append(angle)    
    # retorna a lista joint_angles commo uma lista com todos os angulos
    return joint_angles

def calcular_inputs (Rid):
    # pega a posição e orientação do Rid, mas ignora a posição
    _, orn = p.getBasePositionAndOrientation(Rid)
    # transforma o quaternion em euler
    euler = list(p.getEulerFromQuaternion(orn))
    # le os angulos das juntas do Rid
    angulos = ler_angulos_juntas(Rid)
    # junta tudo em uma lista e retorna ela
    output = euler + angulos
    return output

def reset_simulation():
    # reseta a simulação
    p.resetSimulation()
    # define as configurações
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # carrega o chão
    planeId = p.loadURDF("plane.urdf")
    # carrega o robô humanoide simples (bípede)
    startPos = [0, 0, 3.5]
    startOrientation = p.getQuaternionFromEuler([math.pi/2, 0, 0])  # gira 90° no X
    robotId = p.loadURDF("quadruped/quadruped.urdf", startPos, startOrientation, useFixedBase=False)
    
    return robotId, planeId


def robot_dof_count(Rid):
    count = 0
    for i in range(p.getNumJoints(Rid)):
        jt = p.getJointInfo(Rid, i)[2]
        if jt == p.JOINT_FIXED:
            continue
        if jt == p.JOINT_SPHERICAL:
            count += 3   # vamos mapear 3 valores -> Euler -> quaternion
        else:
            count += 1
    return count

def output_to_robot(Rid, output, max_force=2):
    if torch.is_tensor(output):
        output = output.detach().cpu().numpy().flatten().tolist()

    expected = robot_dof_count(Rid)
    if len(output) != expected:
        raise ValueError(f"output length {len(output)} != expected {expected}")

    idx = 0
    for i in range(p.getNumJoints(Rid)):
        joint_info = p.getJointInfo(Rid, i)
        joint_type = joint_info[2]
        if joint_type == p.JOINT_FIXED:
            continue

        if joint_type == p.JOINT_SPHERICAL:
            euler = output[idx:idx+3]
            idx += 3
            quat = p.getQuaternionFromEuler(euler)
            p.setJointMotorControlMultiDof(
                bodyUniqueId=Rid,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=quat,
                force=[max_force]*4
            )
        else:
            target = output[idx]
            idx += 1
            p.setJointMotorControl2(
                bodyUniqueId=Rid,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                force=max_force
            )

def apenas_pes_no_chao(Rid, planeId):
    # lista com os índices dos links dos pés
    pes = [11, 14]
    
    # pega todos os contatos entre o robô e o chão
    contatos = p.getContactPoints(bodyA=Rid, bodyB=planeId)
    
    for contato in contatos:
        link = contato[3]  # linkIndexA
        if link not in pes:
            # algum link que não é pé está tocando
            return False
    
    # retorna True se há contato apenas nos pés ou nenhum contato
    return any(contato[3] in pes for contato in contatos)

def fitness_func(Rid, Pid):
    global fitness
    if apenas_pes_no_chao(Rid, Pid):
        fitness += 1
    else:
        fitness -= 1

# inicializa a rede neural
robotId, planeId = reset_simulation()
net = init_net(len(calcular_inputs(robotId)), [256, 160], len(ler_angulos_juntas(robotId)))
old_fitness = -10000000000000
geracao = 0
if tfs == False:
    try:
        net.load_state_dict(torch.load("best_net.pth"))
        print("Rede carregada de best_net.pth")
        with open("best_fitness.txt", "r") as f:
            old_fitness = float(f.read())
        print("old_fitness carregado!")
        with open("generation.txt", "r") as g:
            geracao = float(g.read())
        print("geração carregada!")
        print("todos os arquivos carregados com sucesso!")
    except FileNotFoundError:
        print("Nenhuma rede salva encontrada, começando do zero...")
        old_fitness = -10000000000000

fitness = 0
print("quantidade de inputs:", len(calcular_inputs(robotId)))

# loop que roda até o usuario fechar
running = True
while running:
    if rfs and p.getConnectionInfo().get('isConnected',0) == 0:
        running = False
    
    geracao += 1

    #reseta a simulação
    robotId, planeId = reset_simulation()
    if geracao < 100:
        lr = 0.2
    elif geracao < 200:
        lr = 0.05
    else:
        lr - 0.001
    new_net = alt_net(net, lr)

    lose = fitness - old_fitness
    print(f"geração: {int(geracao)}")
    print(f"fitness: {fitness}")
    print(f"old_fitness: {old_fitness}")
    print(f"diferença de {lose} pontos")

    fitness = 0

    # roda uma geração
    tamanho_simulacao = 1000
    for i in range(tamanho_simulacao):
        #roda um passo da rede neural
        with torch.no_grad():
            inputs = torch.tensor(calcular_inputs(robotId), dtype=torch.float32).unsqueeze(0)
            output = new_net(inputs)
        output_to_robot(robotId, output)

        #da um passo de simulação
        p.stepSimulation()

        #calcula o fitness
        fitness_func(robotId, planeId)
        
        # Pega posição do robô e faz a camera seguir ele
        pos, _ = p.getBasePositionAndOrientation(robotId)
        p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=pos)
    
        # limita o fps
        if fps != 0 and rfs == True:
            time.sleep(1./fps)
    
    if fitness > old_fitness:
        net = new_net
        print(f"Novo melhor fitness de: {fitness}!")
        old_fitness = fitness
        torch.save(net.state_dict(), "best_net.pth")
        with open("best_fitness.txt", "w") as f:
            f.write(str(old_fitness))
        with open("generation.txt", "w") as g:
            g.write(str(geracao))
    else:
        lose = fitness - old_fitness
        print(f"diferença de {lose} pontos")

# Finaliza a simulação
p.disconnect()

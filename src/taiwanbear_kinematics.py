
import numpy as np

# Calculate forward kinematics
def forward_kinematics(q, A = None):
    if (A is None):
        A = np.eye(4)
    effectors = []


    M_TaiwanBear = np.array([[7.549790126404332e-08,1.0,-4.371138828673793e-08,0.0],[0.0,-4.371138828673793e-08,-1.0,0.0],[-1.0,7.549790126404332e-08,-3.3001180777756647e-15,0.0],[0.0,0.0,0.0,1.0]])
    A_TaiwanBear = A.dot(M_TaiwanBear)


    M_RJoint_Back_Upper_XYZ_L = np.array([[1.0,0.0,0.0,0.03032732382416725],[0.0,-4.371138828673793e-08,1.0,0.45000001788139343],[0.0,-1.0,-4.371138828673793e-08,-0.23499634861946106],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Upper_XYZ_L = A_TaiwanBear.dot(M_RJoint_Back_Upper_XYZ_L)


    if "RJoint_Back_Upper_XYZ_L" in q:
        theta_RJoint_Back_Upper_XYZ_L = q["RJoint_Back_Upper_XYZ_L"]
        Rz_RJoint_Back_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_L[2]), - math.sin(theta_RJoint_Back_Upper_XYZ_L[2]), 0, 0 ], [ math.sin(theta_RJoint_Back_Upper_XYZ_L[2]), math.cos(theta_RJoint_Back_Upper_XYZ_L[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.dot(Rz_RJoint_Back_Upper_XYZ_L)     
        Ry_RJoint_Back_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_L[1]), 0, math.sin(theta_RJoint_Back_Upper_XYZ_L[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Back_Upper_XYZ_L[1]), math.cos(theta_RJoint_Back_Upper_XYZ_L[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.dot(Ry_RJoint_Back_Upper_XYZ_L)     
        Rx_RJoint_Back_Upper_XYZ_L = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Back_Upper_XYZ_L[0]), - math.sin(theta_RJoint_Back_Upper_XYZ_L[0]), 0 ], [ 0, math.sin(theta_RJoint_Back_Upper_XYZ_L[0]), math.cos(theta_RJoint_Back_Upper_XYZ_L[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.dot(Rx_RJoint_Back_Upper_XYZ_L)     


    M_RJoint_Back_Lower_Z_L = np.array([[1.0,1.5632423334315318e-14,-3.5762786865234375e-07,0.6865012049674988],[-1.0369608636003646e-43,1.0,4.3711427366588396e-08,0.0002469271421432495],[3.5762786865234375e-07,-4.3711427366588396e-08,1.0,0.17306020855903625],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Lower_Z_L = A_RJoint_Back_Upper_XYZ_L.dot(M_RJoint_Back_Lower_Z_L)


    if "RJoint_Back_Lower_Z_L" in q:
        theta_RJoint_Back_Lower_Z_L = q["RJoint_Back_Lower_Z_L"]
        Rz_RJoint_Back_Lower_Z_L = np.array([ [ math.cos(theta_RJoint_Back_Lower_Z_L), - math.sin(theta_RJoint_Back_Lower_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Back_Lower_Z_L), math.cos(theta_RJoint_Back_Lower_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Lower_Z_L = A_RJoint_Back_Lower_Z_L.dot(Rz_RJoint_Back_Lower_Z_L)     


    M_RJoint_Back_Ankle_Z_L = np.array([[1.0,5.2108128603027895e-15,-1.1920928955078125e-07,0.6337858438491821],[-5.210807778105106e-15,1.0,3.197441294480914e-14,2.558872154168057e-07],[1.1920928955078125e-07,-3.1974433273599875e-14,1.0,-3.826122352279526e-08],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Ankle_Z_L = A_RJoint_Back_Lower_Z_L.dot(M_RJoint_Back_Ankle_Z_L)


    if "RJoint_Back_Ankle_Z_L" in q:
        theta_RJoint_Back_Ankle_Z_L = q["RJoint_Back_Ankle_Z_L"]
        Rz_RJoint_Back_Ankle_Z_L = np.array([ [ math.cos(theta_RJoint_Back_Ankle_Z_L), - math.sin(theta_RJoint_Back_Ankle_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Back_Ankle_Z_L), math.cos(theta_RJoint_Back_Ankle_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Ankle_Z_L = A_RJoint_Back_Ankle_Z_L.dot(Rz_RJoint_Back_Ankle_Z_L)     


    M_Effector_Back_L = np.array([[7.549789415861596e-08,3.258413983076025e-07,1.0,0.24104450643062592],[1.7763556535541243e-14,-1.0,3.2584136988589307e-07,-0.1315731257200241],[1.0,-5.8906465859667745e-15,-7.549790836947068e-08,-0.00037578781484626234],[0.0,0.0,0.0,1.0]])
    A_Effector_Back_L = A_RJoint_Back_Ankle_Z_L.dot(M_Effector_Back_L)


    effectors.append(("Effector_Back_L", A_Effector_Back_L))


    M_RJoint_Back_Upper_XYZ_R = np.array([[1.0,-1.3200473158135606e-14,0.0,0.03032725676894188],[5.823351512373315e-22,-9.973857686418341e-07,-1.0,-0.44999995827674866],[1.3200472311102659e-14,1.0,-9.973857686418341e-07,-0.23499631881713867],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Upper_XYZ_R = A_TaiwanBear.dot(M_RJoint_Back_Upper_XYZ_R)


    if "RJoint_Back_Upper_XYZ_R" in q:
        theta_RJoint_Back_Upper_XYZ_R = q["RJoint_Back_Upper_XYZ_R"]
        Rz_RJoint_Back_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_R[2]), - math.sin(theta_RJoint_Back_Upper_XYZ_R[2]), 0, 0 ], [ math.sin(theta_RJoint_Back_Upper_XYZ_R[2]), math.cos(theta_RJoint_Back_Upper_XYZ_R[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.dot(Rz_RJoint_Back_Upper_XYZ_R)     
        Ry_RJoint_Back_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_R[1]), 0, math.sin(theta_RJoint_Back_Upper_XYZ_R[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Back_Upper_XYZ_R[1]), math.cos(theta_RJoint_Back_Upper_XYZ_R[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.dot(Ry_RJoint_Back_Upper_XYZ_R)     
        Rx_RJoint_Back_Upper_XYZ_R = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Back_Upper_XYZ_R[0]), - math.sin(theta_RJoint_Back_Upper_XYZ_R[0]), 0 ], [ 0, math.sin(theta_RJoint_Back_Upper_XYZ_R[0]), math.cos(theta_RJoint_Back_Upper_XYZ_R[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.dot(Rx_RJoint_Back_Upper_XYZ_R)     


    M_RJoint_Back_Lower_Z_R = np.array([[1.0,-6.814220140840405e-14,-6.357301884918343e-08,0.6865009069442749],[5.298521427925078e-14,1.0,-2.384185791015625e-07,-0.0002469192259013653],[6.357301884918343e-08,2.384185791015625e-07,1.0,0.17306026816368103],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Lower_Z_R = A_RJoint_Back_Upper_XYZ_R.dot(M_RJoint_Back_Lower_Z_R)


    if "RJoint_Back_Lower_Z_R" in q:
        theta_RJoint_Back_Lower_Z_R = q["RJoint_Back_Lower_Z_R"]
        Rz_RJoint_Back_Lower_Z_R = np.array([ [ math.cos(theta_RJoint_Back_Lower_Z_R), - math.sin(theta_RJoint_Back_Lower_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Back_Lower_Z_R), math.cos(theta_RJoint_Back_Lower_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Lower_Z_R = A_RJoint_Back_Lower_Z_R.dot(Rz_RJoint_Back_Lower_Z_R)     


    M_RJoint_Back_Ankle_Z_R = np.array([[1.0,0.0,0.0,0.633785605430603],[1.5741393721903615e-22,1.0,2.384185791015625e-07,-2.160634693382235e-07],[0.0,-2.384185791015625e-07,1.0,1.9789455052432459e-07],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Ankle_Z_R = A_RJoint_Back_Lower_Z_R.dot(M_RJoint_Back_Ankle_Z_R)


    if "RJoint_Back_Ankle_Z_R" in q:
        theta_RJoint_Back_Ankle_Z_R = q["RJoint_Back_Ankle_Z_R"]
        Rz_RJoint_Back_Ankle_Z_R = np.array([ [ math.cos(theta_RJoint_Back_Ankle_Z_R), - math.sin(theta_RJoint_Back_Ankle_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Back_Ankle_Z_R), math.cos(theta_RJoint_Back_Ankle_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Ankle_Z_R = A_RJoint_Back_Ankle_Z_R.dot(Rz_RJoint_Back_Ankle_Z_R)     


    M_Effector_Back_R = np.array([[-2.741932405569969e-07,-6.278329465203569e-07,1.0,0.2410445213317871],[-3.2560508765245514e-21,-1.0,-6.278329465203569e-07,0.13157300651073456],[1.0,-1.2117367385097028e-13,2.741932405569969e-07,-0.0003756207588594407],[0.0,0.0,0.0,1.0]])
    A_Effector_Back_R = A_RJoint_Back_Ankle_Z_R.dot(M_Effector_Back_R)


    effectors.append(("Effector_Back_R", A_Effector_Back_R))


    M_RJoint_Torso_XYZ_C = np.array([[2.3700034380218053e-14,7.549790126404332e-08,-1.0,0.007900208234786987],[3.5762786865234375e-07,1.0,7.549790126404332e-08,5.321260942992012e-08],[1.0,-3.5762786865234375e-07,-3.300117866017428e-15,0.8901089429855347],[0.0,0.0,0.0,1.0]])
    A_RJoint_Torso_XYZ_C = A_TaiwanBear.dot(M_RJoint_Torso_XYZ_C)


    if "RJoint_Torso_XYZ_C" in q:
        theta_RJoint_Torso_XYZ_C = q["RJoint_Torso_XYZ_C"]
        Rz_RJoint_Torso_XYZ_C = np.array([ [ math.cos(theta_RJoint_Torso_XYZ_C[2]), - math.sin(theta_RJoint_Torso_XYZ_C[2]), 0, 0 ], [ math.sin(theta_RJoint_Torso_XYZ_C[2]), math.cos(theta_RJoint_Torso_XYZ_C[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.dot(Rz_RJoint_Torso_XYZ_C)     
        Ry_RJoint_Torso_XYZ_C = np.array([ [ math.cos(theta_RJoint_Torso_XYZ_C[1]), 0, math.sin(theta_RJoint_Torso_XYZ_C[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Torso_XYZ_C[1]), math.cos(theta_RJoint_Torso_XYZ_C[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.dot(Ry_RJoint_Torso_XYZ_C)     
        Rx_RJoint_Torso_XYZ_C = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Torso_XYZ_C[0]), - math.sin(theta_RJoint_Torso_XYZ_C[0]), 0 ], [ 0, math.sin(theta_RJoint_Torso_XYZ_C[0]), math.cos(theta_RJoint_Torso_XYZ_C[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.dot(Rx_RJoint_Torso_XYZ_C)     


    M_RJoint_Front_Upper_XYZ_L = np.array([[2.3700034380218053e-14,-1.0,-4.3711395392165286e-08,0.6499999761581421],[7.549790126404332e-08,-4.3711395392165286e-08,1.0,0.4500000774860382],[-1.0,-2.7000152669751955e-14,7.549790126404332e-08,-0.02242732234299183],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Upper_XYZ_L = A_RJoint_Torso_XYZ_C.dot(M_RJoint_Front_Upper_XYZ_L)


    if "RJoint_Front_Upper_XYZ_L" in q:
        theta_RJoint_Front_Upper_XYZ_L = q["RJoint_Front_Upper_XYZ_L"]
        Rz_RJoint_Front_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_L[2]), - math.sin(theta_RJoint_Front_Upper_XYZ_L[2]), 0, 0 ], [ math.sin(theta_RJoint_Front_Upper_XYZ_L[2]), math.cos(theta_RJoint_Front_Upper_XYZ_L[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.dot(Rz_RJoint_Front_Upper_XYZ_L)     
        Ry_RJoint_Front_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_L[1]), 0, math.sin(theta_RJoint_Front_Upper_XYZ_L[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Front_Upper_XYZ_L[1]), math.cos(theta_RJoint_Front_Upper_XYZ_L[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.dot(Ry_RJoint_Front_Upper_XYZ_L)     
        Rx_RJoint_Front_Upper_XYZ_L = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Front_Upper_XYZ_L[0]), - math.sin(theta_RJoint_Front_Upper_XYZ_L[0]), 0 ], [ 0, math.sin(theta_RJoint_Front_Upper_XYZ_L[0]), math.cos(theta_RJoint_Front_Upper_XYZ_L[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.dot(Rx_RJoint_Front_Upper_XYZ_L)     


    M_RJoint_Front_Lower_Z_L = np.array([[1.0,1.6940658945086007e-21,0.0,0.8283848166465759],[-2.491518845517339e-22,1.0,4.3711366970455856e-08,0.00029831973370164633],[0.0,-4.3711366970455856e-08,1.0,0.05419759079813957],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Lower_Z_L = A_RJoint_Front_Upper_XYZ_L.dot(M_RJoint_Front_Lower_Z_L)


    if "RJoint_Front_Lower_Z_L" in q:
        theta_RJoint_Front_Lower_Z_L = q["RJoint_Front_Lower_Z_L"]
        Rz_RJoint_Front_Lower_Z_L = np.array([ [ math.cos(theta_RJoint_Front_Lower_Z_L), - math.sin(theta_RJoint_Front_Lower_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Front_Lower_Z_L), math.cos(theta_RJoint_Front_Lower_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Lower_Z_L = A_RJoint_Front_Lower_Z_L.dot(Rz_RJoint_Front_Lower_Z_L)     


    M_RJoint_Front_Ankle_Z_L = np.array([[1.0,0.0,0.0,0.5520808696746826],[-2.491518845517339e-22,1.0,-2.2876067581066115e-21,-0.00918277446180582],[0.0,-2.2876067581066115e-21,1.0,-3.109511510501761e-07],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Ankle_Z_L = A_RJoint_Front_Lower_Z_L.dot(M_RJoint_Front_Ankle_Z_L)


    if "RJoint_Front_Ankle_Z_L" in q:
        theta_RJoint_Front_Ankle_Z_L = q["RJoint_Front_Ankle_Z_L"]
        Rz_RJoint_Front_Ankle_Z_L = np.array([ [ math.cos(theta_RJoint_Front_Ankle_Z_L), - math.sin(theta_RJoint_Front_Ankle_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Front_Ankle_Z_L), math.cos(theta_RJoint_Front_Ankle_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Ankle_Z_L = A_RJoint_Front_Ankle_Z_L.dot(Rz_RJoint_Front_Ankle_Z_L)     


    M_Effector_Front_L = np.array([[1.5099581673894136e-07,8.026785849324369e-07,1.0,0.18844406306743622],[5.684341886080802e-14,-1.0,8.026785280890181e-07,-0.050207819789648056],[1.0,-6.06005453067035e-14,-1.5099580252808664e-07,0.007086289115250111],[0.0,0.0,0.0,1.0]])
    A_Effector_Front_L = A_RJoint_Front_Ankle_Z_L.dot(M_Effector_Front_L)


    effectors.append(("Effector_Front_L", A_Effector_Front_L))


    M_RJoint_Front_Upper_XYZ_R = np.array([[3.6900505844287765e-14,1.0,7.549789415861596e-08,0.6499999761581421],[7.549790126404332e-08,7.549789415861596e-08,-1.0,-0.45000001788139343],[-1.0,4.260044126811434e-14,-7.549790126404332e-08,-0.022427255287766457],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Upper_XYZ_R = A_RJoint_Torso_XYZ_C.dot(M_RJoint_Front_Upper_XYZ_R)


    if "RJoint_Front_Upper_XYZ_R" in q:
        theta_RJoint_Front_Upper_XYZ_R = q["RJoint_Front_Upper_XYZ_R"]
        Rz_RJoint_Front_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_R[2]), - math.sin(theta_RJoint_Front_Upper_XYZ_R[2]), 0, 0 ], [ math.sin(theta_RJoint_Front_Upper_XYZ_R[2]), math.cos(theta_RJoint_Front_Upper_XYZ_R[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.dot(Rz_RJoint_Front_Upper_XYZ_R)     
        Ry_RJoint_Front_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_R[1]), 0, math.sin(theta_RJoint_Front_Upper_XYZ_R[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Front_Upper_XYZ_R[1]), math.cos(theta_RJoint_Front_Upper_XYZ_R[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.dot(Ry_RJoint_Front_Upper_XYZ_R)     
        Rx_RJoint_Front_Upper_XYZ_R = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Front_Upper_XYZ_R[0]), - math.sin(theta_RJoint_Front_Upper_XYZ_R[0]), 0 ], [ 0, math.sin(theta_RJoint_Front_Upper_XYZ_R[0]), math.cos(theta_RJoint_Front_Upper_XYZ_R[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.dot(Rx_RJoint_Front_Upper_XYZ_R)     


    M_RJoint_Front_Lower_Z_R = np.array([[1.0,2.2799733224976824e-14,-6.357301884918343e-08,0.8283846378326416],[-3.795672204819599e-14,1.0,-2.3841856489070778e-07,-0.00029833876760676503],[6.357301884918343e-08,2.3841856489070778e-07,1.0,0.054197974503040314],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Lower_Z_R = A_RJoint_Front_Upper_XYZ_R.dot(M_RJoint_Front_Lower_Z_R)


    if "RJoint_Front_Lower_Z_R" in q:
        theta_RJoint_Front_Lower_Z_R = q["RJoint_Front_Lower_Z_R"]
        Rz_RJoint_Front_Lower_Z_R = np.array([ [ math.cos(theta_RJoint_Front_Lower_Z_R), - math.sin(theta_RJoint_Front_Lower_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Front_Lower_Z_R), math.cos(theta_RJoint_Front_Lower_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Lower_Z_R = A_RJoint_Front_Lower_Z_R.dot(Rz_RJoint_Front_Lower_Z_R)     


    M_RJoint_Front_Ankle_Z_R = np.array([[1.0,0.0,0.0,0.5520808696746826],[4.55698723209689e-23,1.0,-2.1472023911872446e-23,0.009183092042803764],[0.0,-2.1472020756428825e-23,1.0,-4.730811653530509e-08],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Ankle_Z_R = A_RJoint_Front_Lower_Z_R.dot(M_RJoint_Front_Ankle_Z_R)


    if "RJoint_Front_Ankle_Z_R" in q:
        theta_RJoint_Front_Ankle_Z_R = q["RJoint_Front_Ankle_Z_R"]
        Rz_RJoint_Front_Ankle_Z_R = np.array([ [ math.cos(theta_RJoint_Front_Ankle_Z_R), - math.sin(theta_RJoint_Front_Ankle_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Front_Ankle_Z_R), math.cos(theta_RJoint_Front_Ankle_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Ankle_Z_R = A_RJoint_Front_Ankle_Z_R.dot(Rz_RJoint_Front_Ankle_Z_R)     


    M_Effector_Front_R = np.array([[-8.742284052232208e-08,-6.278329465203569e-07,1.0,0.18844377994537354],[1.359421531208789e-22,-1.0,-6.278329465203569e-07,0.050207775086164474],[1.0,-6.433025499016226e-14,8.742283341689472e-08,0.007086239755153656],[0.0,0.0,0.0,1.0]])
    A_Effector_Front_R = A_RJoint_Front_Ankle_Z_R.dot(M_Effector_Front_R)


    effectors.append(("Effector_Front_R", A_Effector_Front_R))


    M_RJoint_Head_XYZ_C = np.array([[1.0,1.0890779494492707e-29,4.371138473402425e-08,1.0458284616470337],[6.3054852339690036e-30,1.0,7.549790126404332e-08,4.664957486966159e-08],[-4.371138473402425e-08,-7.549790126404332e-08,1.0,0.012383832596242428],[0.0,0.0,0.0,1.0]])
    A_RJoint_Head_XYZ_C = A_RJoint_Torso_XYZ_C.dot(M_RJoint_Head_XYZ_C)


    if "RJoint_Head_XYZ_C" in q:
        theta_RJoint_Head_XYZ_C = q["RJoint_Head_XYZ_C"]
        Rz_RJoint_Head_XYZ_C = np.array([ [ math.cos(theta_RJoint_Head_XYZ_C[2]), - math.sin(theta_RJoint_Head_XYZ_C[2]), 0, 0 ], [ math.sin(theta_RJoint_Head_XYZ_C[2]), math.cos(theta_RJoint_Head_XYZ_C[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.dot(Rz_RJoint_Head_XYZ_C)     
        Ry_RJoint_Head_XYZ_C = np.array([ [ math.cos(theta_RJoint_Head_XYZ_C[1]), 0, math.sin(theta_RJoint_Head_XYZ_C[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Head_XYZ_C[1]), math.cos(theta_RJoint_Head_XYZ_C[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.dot(Ry_RJoint_Head_XYZ_C)     
        Rx_RJoint_Head_XYZ_C = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Head_XYZ_C[0]), - math.sin(theta_RJoint_Head_XYZ_C[0]), 0 ], [ 0, math.sin(theta_RJoint_Head_XYZ_C[0]), math.cos(theta_RJoint_Head_XYZ_C[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.dot(Rx_RJoint_Head_XYZ_C)     


    M_Effector_Head_C = np.array([[-4.371136341774218e-08,-1.1920928955078125e-07,1.0,0.9635696411132812],[1.0,-2.6037946958593593e-14,4.37113598650285e-08,4.211892346006607e-08],[1.9894737170612214e-14,1.0,1.1920928955078125e-07,-0.44918084144592285],[0.0,0.0,0.0,1.0]])
    A_Effector_Head_C = A_RJoint_Head_XYZ_C.dot(M_Effector_Head_C)


    effectors.append(("Effector_Head_C", A_Effector_Head_C))


    return effectors

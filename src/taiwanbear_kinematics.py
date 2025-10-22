
import numpy as np

# Calculate forward kinematics
def forward_kinematics(q, A = None):
    if (A is None):
        A = np.eye(4)
    linkTransformations = []


    M_TaiwanBear = np.array([[1.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0],[0.0,0.0,1.0,0.0],[0.0,0.0,0.0,1.0]])
    A_TaiwanBear = A.dot(M_TaiwanBear)


    linkTransformations.append(("TaiwanBear", A_TaiwanBear))


    M_RJoint_Back_Upper_XYZ_L = np.array([[-4.371138828673793e-08,-4.371138828673793e-08,1.0,0.45000001788139343],[-1.0,1.910685676922942e-15,-4.371138828673793e-08,-0.03032730147242546],[0.0,-1.0,-4.371138828673793e-08,-0.23567399382591248],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Upper_XYZ_L = A_TaiwanBear.dot(M_RJoint_Back_Upper_XYZ_L)


    if "RJoint_Back_Upper_XYZ_L" in q:
        theta_RJoint_Back_Upper_XYZ_L = q["RJoint_Back_Upper_XYZ_L"]
        Rz_RJoint_Back_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_L[2]), - math.sin(theta_RJoint_Back_Upper_XYZ_L[2]), 0, 0 ], [ math.sin(theta_RJoint_Back_Upper_XYZ_L[2]), math.cos(theta_RJoint_Back_Upper_XYZ_L[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.dot(Rz_RJoint_Back_Upper_XYZ_L)     
        Ry_RJoint_Back_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_L[1]), 0, math.sin(theta_RJoint_Back_Upper_XYZ_L[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Back_Upper_XYZ_L[1]), math.cos(theta_RJoint_Back_Upper_XYZ_L[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.dot(Ry_RJoint_Back_Upper_XYZ_L)     
        Rx_RJoint_Back_Upper_XYZ_L = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Back_Upper_XYZ_L[0]), - math.sin(theta_RJoint_Back_Upper_XYZ_L[0]), 0 ], [ 0, math.sin(theta_RJoint_Back_Upper_XYZ_L[0]), math.cos(theta_RJoint_Back_Upper_XYZ_L[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.dot(Rx_RJoint_Back_Upper_XYZ_L)     


    linkTransformations.append(("RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L))


    M_Link_Back_Upper_L = np.array([[-4.371138828673793e-08,-7.549790126404332e-08,-1.0,0.28632402420043945],[-4.371138828673793e-08,1.0,-7.549790126404332e-08,0.009220348671078682],[1.0,4.371138473402425e-08,-4.371139183945161e-08,0.1939561367034912],[0.0,0.0,0.0,1.0]])
    A_Link_Back_Upper_L = A_RJoint_Back_Upper_XYZ_L.dot(M_Link_Back_Upper_L)


    linkTransformations.append(("Link_Back_Upper_L", A_Link_Back_Upper_L))


    M_RJoint_Back_Lower_Z_L = np.array([[1.9470718370939721e-07,3.1786502319164356e-08,1.0,-0.020895838737487793],[-7.105427357601002e-15,1.0,-3.1786502319164356e-08,-0.009650344029068947],[-1.0,0.0,1.9470718370939721e-07,-0.40017712116241455],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Lower_Z_L = A_Link_Back_Upper_L.dot(M_RJoint_Back_Lower_Z_L)


    if "RJoint_Back_Lower_Z_L" in q:
        theta_RJoint_Back_Lower_Z_L = q["RJoint_Back_Lower_Z_L"]
        Rz_RJoint_Back_Lower_Z_L = np.array([ [ math.cos(theta_RJoint_Back_Lower_Z_L), - math.sin(theta_RJoint_Back_Lower_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Back_Lower_Z_L), math.cos(theta_RJoint_Back_Lower_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Lower_Z_L = A_RJoint_Back_Lower_Z_L.dot(Rz_RJoint_Back_Lower_Z_L)     


    linkTransformations.append(("RJoint_Back_Lower_Z_L", A_RJoint_Back_Lower_Z_L))


    M_Link_Back_Lower_L = np.array([[1.9470718370939721e-07,-7.105427357601002e-15,-1.0,0.301618367433548],[3.1786502319164356e-08,1.0,-7.940933880509066e-23,0.009650426916778088],[1.0,-3.1786502319164356e-08,1.9470718370939721e-07,0.01826903410255909],[0.0,0.0,0.0,1.0]])
    A_Link_Back_Lower_L = A_RJoint_Back_Lower_Z_L.dot(M_Link_Back_Lower_L)


    linkTransformations.append(("Link_Back_Lower_L", A_Link_Back_Lower_L))


    M_RJoint_Back_Ankle_Z_L = np.array([[3.1391647326017846e-07,4.371142026116104e-08,1.0,-0.018269002437591553],[1.0331600464041912e-07,1.0,-4.371145223558415e-08,-0.009650138206779957],[-1.0,1.0331601885127384e-07,3.1391647326017846e-07,-0.33216753602027893],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Ankle_Z_L = A_Link_Back_Lower_L.dot(M_RJoint_Back_Ankle_Z_L)


    if "RJoint_Back_Ankle_Z_L" in q:
        theta_RJoint_Back_Ankle_Z_L = q["RJoint_Back_Ankle_Z_L"]
        Rz_RJoint_Back_Ankle_Z_L = np.array([ [ math.cos(theta_RJoint_Back_Ankle_Z_L), - math.sin(theta_RJoint_Back_Ankle_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Back_Ankle_Z_L), math.cos(theta_RJoint_Back_Ankle_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Ankle_Z_L = A_RJoint_Back_Ankle_Z_L.dot(Rz_RJoint_Back_Ankle_Z_L)     


    linkTransformations.append(("RJoint_Back_Ankle_Z_L", A_RJoint_Back_Ankle_Z_L))


    M_Link_Back_Foot_L = np.array([[6.715443419125222e-07,-5.684341886080802e-14,-1.0,0.23572823405265808],[4.3711427366588396e-08,1.0,-2.7000142505356588e-14,-0.13118453323841095],[1.0,-4.3711427366588396e-08,6.715443419125222e-07,-0.00037607140257023275],[0.0,0.0,0.0,1.0]])
    A_Link_Back_Foot_L = A_RJoint_Back_Ankle_Z_L.dot(M_Link_Back_Foot_L)


    linkTransformations.append(("Link_Back_Foot_L", A_Link_Back_Foot_L))


    M_Effector_Back_L = np.array([[-1.1920922560193503e-07,-1.0,9.218878176397993e-07,-6.081847203631696e-08],[-1.0,1.1920911191509731e-07,-1.1920946008103783e-07,-0.00038848756230436265],[1.192093321833454e-07,-9.218877607963805e-07,-1.0,-0.07693008333444595],[0.0,0.0,0.0,1.0]])
    A_Effector_Back_L = A_Link_Back_Foot_L.dot(M_Effector_Back_L)


    linkTransformations.append(("Effector_Back_L", A_Effector_Back_L))


    M_RJoint_Back_Upper_XYZ_R = np.array([[7.549790126404332e-08,7.549790126404332e-08,-1.0,-0.44999995827674866],[-1.0,-1.4610056261045272e-15,-7.549790126404332e-08,-0.03032730147242546],[-7.16093850883226e-15,1.0,7.549790126404332e-08,-0.2349960058927536],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Upper_XYZ_R = A_TaiwanBear.dot(M_RJoint_Back_Upper_XYZ_R)


    if "RJoint_Back_Upper_XYZ_R" in q:
        theta_RJoint_Back_Upper_XYZ_R = q["RJoint_Back_Upper_XYZ_R"]
        Rz_RJoint_Back_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_R[2]), - math.sin(theta_RJoint_Back_Upper_XYZ_R[2]), 0, 0 ], [ math.sin(theta_RJoint_Back_Upper_XYZ_R[2]), math.cos(theta_RJoint_Back_Upper_XYZ_R[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.dot(Rz_RJoint_Back_Upper_XYZ_R)     
        Ry_RJoint_Back_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Back_Upper_XYZ_R[1]), 0, math.sin(theta_RJoint_Back_Upper_XYZ_R[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Back_Upper_XYZ_R[1]), math.cos(theta_RJoint_Back_Upper_XYZ_R[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.dot(Ry_RJoint_Back_Upper_XYZ_R)     
        Rx_RJoint_Back_Upper_XYZ_R = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Back_Upper_XYZ_R[0]), - math.sin(theta_RJoint_Back_Upper_XYZ_R[0]), 0 ], [ 0, math.sin(theta_RJoint_Back_Upper_XYZ_R[0]), math.cos(theta_RJoint_Back_Upper_XYZ_R[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.dot(Rx_RJoint_Back_Upper_XYZ_R)     


    linkTransformations.append(("RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R))


    M_Link_Back_Upper_R = np.array([[7.549790126404332e-08,-1.3200473158135606e-14,-1.0,0.28632378578186035],[-1.0410971071905806e-06,-1.0,-6.540017279902527e-14,-0.00989750400185585],[-1.0,1.0410971071905806e-06,-7.549790126404332e-08,0.1939561367034912],[0.0,0.0,0.0,1.0]])
    A_Link_Back_Upper_R = A_RJoint_Back_Upper_XYZ_R.dot(M_Link_Back_Upper_R)


    linkTransformations.append(("Link_Back_Upper_R", A_Link_Back_Upper_R))


    M_RJoint_Back_Lower_Z_R = np.array([[1.6292057125610881e-07,1.1920927818209748e-06,-1.0,0.020895862951874733],[-8.742289026031358e-08,-0.9999999403953552,-1.1920928955078125e-06,-0.00965078640729189],[-0.9999999403953552,8.742308210685223e-08,-1.629204717801258e-07,-0.40017712116241455],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Lower_Z_R = A_Link_Back_Upper_R.dot(M_RJoint_Back_Lower_Z_R)


    if "RJoint_Back_Lower_Z_R" in q:
        theta_RJoint_Back_Lower_Z_R = q["RJoint_Back_Lower_Z_R"]
        Rz_RJoint_Back_Lower_Z_R = np.array([ [ math.cos(theta_RJoint_Back_Lower_Z_R), - math.sin(theta_RJoint_Back_Lower_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Back_Lower_Z_R), math.cos(theta_RJoint_Back_Lower_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Lower_Z_R = A_RJoint_Back_Lower_Z_R.dot(Rz_RJoint_Back_Lower_Z_R)     


    linkTransformations.append(("RJoint_Back_Lower_Z_R", A_RJoint_Back_Lower_Z_R))


    M_Link_Back_Lower_R = np.array([[1.1924797149731603e-08,7.54979225803254e-08,1.0,0.3016180992126465],[-6.198968094395241e-07,1.0,-7.549791547489804e-08,-0.009650622494518757],[-1.0,-6.198965820658486e-07,1.192486731582676e-08,0.018269259482622147],[0.0,0.0,0.0,1.0]])
    A_Link_Back_Lower_R = A_RJoint_Back_Lower_Z_R.dot(M_Link_Back_Lower_R)


    linkTransformations.append(("Link_Back_Lower_R", A_Link_Back_Lower_R))


    M_RJoint_Back_Ankle_Z_R = np.array([[4.37113598650285e-08,8.741883448237786e-07,-1.0,0.0182691290974617],[1.5099587358236022e-07,1.0,8.741883448237786e-07,0.009650207124650478],[1.0,-1.5099590200406965e-07,4.371127104718653e-08,0.33216753602027893],[0.0,0.0,0.0,1.0]])
    A_RJoint_Back_Ankle_Z_R = A_Link_Back_Lower_R.dot(M_RJoint_Back_Ankle_Z_R)


    if "RJoint_Back_Ankle_Z_R" in q:
        theta_RJoint_Back_Ankle_Z_R = q["RJoint_Back_Ankle_Z_R"]
        Rz_RJoint_Back_Ankle_Z_R = np.array([ [ math.cos(theta_RJoint_Back_Ankle_Z_R), - math.sin(theta_RJoint_Back_Ankle_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Back_Ankle_Z_R), math.cos(theta_RJoint_Back_Ankle_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Back_Ankle_Z_R = A_RJoint_Back_Ankle_Z_R.dot(Rz_RJoint_Back_Ankle_Z_R)     


    linkTransformations.append(("RJoint_Back_Ankle_Z_R", A_RJoint_Back_Ankle_Z_R))


    M_Link_Back_Foot_R = np.array([[1.1924874421254117e-08,1.5099581673894136e-07,1.0,0.22246618568897247],[-1.0410972208774183e-06,1.0,-1.5099580252808664e-07,-0.0004303620371501893],[-1.0,-1.0410969935037429e-06,1.192501297708759e-08,-0.0003757415688596666],[0.0,0.0,0.0,1.0]])
    A_Link_Back_Foot_R = A_RJoint_Back_Ankle_Z_R.dot(M_Link_Back_Foot_R)


    linkTransformations.append(("Link_Back_Foot_R", A_Link_Back_Foot_R))


    M_Effector_Back_R = np.array([[7.033305564618786e-07,-1.0,-6.357277726465327e-08,3.632777634265949e-09],[1.0,7.033304427750409e-07,4.7683693082944956e-07,0.0003885070327669382],[-4.7683684556432127e-07,-6.357311832516643e-08,1.0,0.07693040370941162],[0.0,0.0,0.0,1.0]])
    A_Effector_Back_R = A_Link_Back_Foot_R.dot(M_Effector_Back_R)


    linkTransformations.append(("Effector_Back_R", A_Effector_Back_R))


    M_RJoint_Torso_XYZ_C = np.array([[1.9470718370939721e-07,1.0,4.371138828673793e-08,1.4901161193847656e-08],[0.0,-4.371138828673793e-08,1.0,3.8907899124751566e-08],[1.0,-1.9470718370939721e-07,-8.510921832474271e-15,0.8901089429855347],[0.0,0.0,0.0,1.0]])
    A_RJoint_Torso_XYZ_C = A_TaiwanBear.dot(M_RJoint_Torso_XYZ_C)


    if "RJoint_Torso_XYZ_C" in q:
        theta_RJoint_Torso_XYZ_C = q["RJoint_Torso_XYZ_C"]
        Rz_RJoint_Torso_XYZ_C = np.array([ [ math.cos(theta_RJoint_Torso_XYZ_C[2]), - math.sin(theta_RJoint_Torso_XYZ_C[2]), 0, 0 ], [ math.sin(theta_RJoint_Torso_XYZ_C[2]), math.cos(theta_RJoint_Torso_XYZ_C[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.dot(Rz_RJoint_Torso_XYZ_C)     
        Ry_RJoint_Torso_XYZ_C = np.array([ [ math.cos(theta_RJoint_Torso_XYZ_C[1]), 0, math.sin(theta_RJoint_Torso_XYZ_C[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Torso_XYZ_C[1]), math.cos(theta_RJoint_Torso_XYZ_C[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.dot(Ry_RJoint_Torso_XYZ_C)     
        Rx_RJoint_Torso_XYZ_C = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Torso_XYZ_C[0]), - math.sin(theta_RJoint_Torso_XYZ_C[0]), 0 ], [ 0, math.sin(theta_RJoint_Torso_XYZ_C[0]), math.cos(theta_RJoint_Torso_XYZ_C[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.dot(Rx_RJoint_Torso_XYZ_C)     


    linkTransformations.append(("RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C))


    M_Link_UpperBody_C = np.array([[3.1391647326017846e-07,-1.0,0.0,-5.995204332975845e-15],[1.0,3.1391647326017846e-07,1.6571376115956498e-21,-2.8993822454026486e-08],[-1.0145808440972953e-21,0.0,1.0,-0.3840346932411194],[0.0,0.0,0.0,1.0]])
    A_Link_UpperBody_C = A_RJoint_Torso_XYZ_C.dot(M_Link_UpperBody_C)


    linkTransformations.append(("Link_UpperBody_C", A_Link_UpperBody_C))


    M_RJoint_Front_Upper_XYZ_L = np.array([[1.5099580252808664e-07,1.9106871592305995e-15,1.0,0.45000001788139343],[-4.371141670844736e-08,1.0,-1.4690727678941772e-21,-0.6500012278556824],[-1.0,-4.371141315573368e-08,1.5099580252808664e-07,0.3616074323654175],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Upper_XYZ_L = A_Link_UpperBody_C.dot(M_RJoint_Front_Upper_XYZ_L)


    if "RJoint_Front_Upper_XYZ_L" in q:
        theta_RJoint_Front_Upper_XYZ_L = q["RJoint_Front_Upper_XYZ_L"]
        Rz_RJoint_Front_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_L[2]), - math.sin(theta_RJoint_Front_Upper_XYZ_L[2]), 0, 0 ], [ math.sin(theta_RJoint_Front_Upper_XYZ_L[2]), math.cos(theta_RJoint_Front_Upper_XYZ_L[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.dot(Rz_RJoint_Front_Upper_XYZ_L)     
        Ry_RJoint_Front_Upper_XYZ_L = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_L[1]), 0, math.sin(theta_RJoint_Front_Upper_XYZ_L[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Front_Upper_XYZ_L[1]), math.cos(theta_RJoint_Front_Upper_XYZ_L[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.dot(Ry_RJoint_Front_Upper_XYZ_L)     
        Rx_RJoint_Front_Upper_XYZ_L = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Front_Upper_XYZ_L[0]), - math.sin(theta_RJoint_Front_Upper_XYZ_L[0]), 0 ], [ 0, math.sin(theta_RJoint_Front_Upper_XYZ_L[0]), math.cos(theta_RJoint_Front_Upper_XYZ_L[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.dot(Rx_RJoint_Front_Upper_XYZ_L)     


    linkTransformations.append(("RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L))


    M_Link_Front_Upper_L = np.array([[1.5099580252808664e-07,-4.371142026116104e-08,-1.0,0.4007798731327057],[4.3711398944878965e-08,1.0,-4.371141315573368e-08,0.009219011291861534],[1.0,-4.371138828673793e-08,1.5099580252808664e-07,0.06649638712406158],[0.0,0.0,0.0,1.0]])
    A_Link_Front_Upper_L = A_RJoint_Front_Upper_XYZ_L.dot(M_Link_Front_Upper_L)


    linkTransformations.append(("Link_Front_Upper_L", A_Link_Front_Upper_L))


    M_RJoint_Front_Lower_Z_L = np.array([[1.9470718370939721e-07,2.842170943040401e-14,1.0,-0.01229878794401884],[-4.9582090007067176e-15,1.0,-2.8421712818535796e-14,-0.009218773804605007],[-1.0,-1.4054908749832435e-15,1.9470718370939721e-07,-0.4276049733161926],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Lower_Z_L = A_Link_Front_Upper_L.dot(M_RJoint_Front_Lower_Z_L)


    if "RJoint_Front_Lower_Z_L" in q:
        theta_RJoint_Front_Lower_Z_L = q["RJoint_Front_Lower_Z_L"]
        Rz_RJoint_Front_Lower_Z_L = np.array([ [ math.cos(theta_RJoint_Front_Lower_Z_L), - math.sin(theta_RJoint_Front_Lower_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Front_Lower_Z_L), math.cos(theta_RJoint_Front_Lower_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Lower_Z_L = A_RJoint_Front_Lower_Z_L.dot(Rz_RJoint_Front_Lower_Z_L)     


    linkTransformations.append(("RJoint_Front_Lower_Z_L", A_RJoint_Front_Lower_Z_L))


    M_Link_Front_Lower_L = np.array([[1.5099581673894136e-07,-4.371141670844736e-08,-1.0,0.2915906012058258],[2.842170943040401e-14,1.0,-4.371142026116104e-08,-1.9945164808632398e-07],[1.0,-2.182147115727031e-14,1.5099581673894136e-07,0.011934522539377213],[0.0,0.0,0.0,1.0]])
    A_Link_Front_Lower_L = A_RJoint_Front_Lower_Z_L.dot(M_Link_Front_Lower_L)


    linkTransformations.append(("Link_Front_Lower_L", A_Link_Front_Lower_L))


    M_RJoint_Front_Ankle_Z_L = np.array([[2.702051062897226e-07,4.013392356227996e-07,1.0,-0.011934819631278515],[-4.371142381387472e-08,1.0,-4.013392356227996e-07,-0.0031993621960282326],[-1.0,-4.371131723246435e-08,2.702051062897226e-07,-0.26048988103866577],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Ankle_Z_L = A_Link_Front_Lower_L.dot(M_RJoint_Front_Ankle_Z_L)


    if "RJoint_Front_Ankle_Z_L" in q:
        theta_RJoint_Front_Ankle_Z_L = q["RJoint_Front_Ankle_Z_L"]
        Rz_RJoint_Front_Ankle_Z_L = np.array([ [ math.cos(theta_RJoint_Front_Ankle_Z_L), - math.sin(theta_RJoint_Front_Ankle_Z_L), 0, 0 ], [ math.sin(theta_RJoint_Front_Ankle_Z_L), math.cos(theta_RJoint_Front_Ankle_Z_L), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Ankle_Z_L = A_RJoint_Front_Ankle_Z_L.dot(Rz_RJoint_Front_Ankle_Z_L)     


    linkTransformations.append(("RJoint_Front_Ankle_Z_L", A_RJoint_Front_Ankle_Z_L))


    M_Link_Front_Foot_L = np.array([[1.5099580252808664e-07,-4.371142026116104e-08,-1.0,0.19831718504428864],[2.8421719594799374e-14,1.0,-4.371142026116104e-08,0.012381916865706444],[1.0,-1.6610637968176786e-14,1.5099580252808664e-07,0.0070861573331058025],[0.0,0.0,0.0,1.0]])
    A_Link_Front_Foot_L = A_RJoint_Front_Ankle_Z_L.dot(M_Link_Front_Foot_L)


    linkTransformations.append(("Link_Front_Foot_L", A_Link_Front_Foot_L))


    M_Effector_Front_L = np.array([[-4.013393208879279e-07,-1.0,1.1165950581926154e-06,-1.2849028507844196e-09],[-1.0,4.013392072010902e-07,-7.54984412765225e-08,-0.006353206932544708],[7.549799363459897e-08,-1.1165950581926154e-06,-1.0,-0.0586923286318779],[0.0,0.0,0.0,1.0]])
    A_Effector_Front_L = A_Link_Front_Foot_L.dot(M_Effector_Front_L)


    linkTransformations.append(("Effector_Front_L", A_Effector_Front_L))


    M_RJoint_Front_Upper_XYZ_R = np.array([[1.5099580252808664e-07,3.1786495213737e-08,-1.0,-0.4500000774860382],[-4.3711430919302074e-08,-1.0,-3.178649876645068e-08,-0.6499999165534973],[-1.0,4.3711430919302074e-08,-1.5099580252808664e-07,0.3616074323654175],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Upper_XYZ_R = A_Link_UpperBody_C.dot(M_RJoint_Front_Upper_XYZ_R)


    if "RJoint_Front_Upper_XYZ_R" in q:
        theta_RJoint_Front_Upper_XYZ_R = q["RJoint_Front_Upper_XYZ_R"]
        Rz_RJoint_Front_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_R[2]), - math.sin(theta_RJoint_Front_Upper_XYZ_R[2]), 0, 0 ], [ math.sin(theta_RJoint_Front_Upper_XYZ_R[2]), math.cos(theta_RJoint_Front_Upper_XYZ_R[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.dot(Rz_RJoint_Front_Upper_XYZ_R)     
        Ry_RJoint_Front_Upper_XYZ_R = np.array([ [ math.cos(theta_RJoint_Front_Upper_XYZ_R[1]), 0, math.sin(theta_RJoint_Front_Upper_XYZ_R[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Front_Upper_XYZ_R[1]), math.cos(theta_RJoint_Front_Upper_XYZ_R[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.dot(Ry_RJoint_Front_Upper_XYZ_R)     
        Rx_RJoint_Front_Upper_XYZ_R = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Front_Upper_XYZ_R[0]), - math.sin(theta_RJoint_Front_Upper_XYZ_R[0]), 0 ], [ 0, math.sin(theta_RJoint_Front_Upper_XYZ_R[0]), math.cos(theta_RJoint_Front_Upper_XYZ_R[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.dot(Rx_RJoint_Front_Upper_XYZ_R)     


    linkTransformations.append(("RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R))


    M_Link_Front_Upper_R = np.array([[1.5099580252808664e-07,-3.258413130424742e-07,1.0,0.40077969431877136],[3.178648810830964e-08,1.0,3.258413130424742e-07,8.611196449237468e-07],[-1.0,3.1786445475745495e-08,1.5099583094979607e-07,0.06649649143218994],[0.0,0.0,0.0,1.0]])
    A_Link_Front_Upper_R = A_RJoint_Front_Upper_XYZ_R.dot(M_Link_Front_Upper_R)


    linkTransformations.append(("Link_Front_Upper_R", A_Link_Front_Upper_R))


    M_RJoint_Front_Lower_Z_R = np.array([[1.1924875309432537e-08,-3.2584136988589307e-07,-1.0,0.012298567220568657],[-3.6955276527805836e-07,1.0,-3.2584136988589307e-07,0.009898408316075802],[1.0,3.6955276527805836e-07,1.1924753628989038e-08,0.42760491371154785],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Lower_Z_R = A_Link_Front_Upper_R.dot(M_RJoint_Front_Lower_Z_R)


    if "RJoint_Front_Lower_Z_R" in q:
        theta_RJoint_Front_Lower_Z_R = q["RJoint_Front_Lower_Z_R"]
        Rz_RJoint_Front_Lower_Z_R = np.array([ [ math.cos(theta_RJoint_Front_Lower_Z_R), - math.sin(theta_RJoint_Front_Lower_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Front_Lower_Z_R), math.cos(theta_RJoint_Front_Lower_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Lower_Z_R = A_RJoint_Front_Lower_Z_R.dot(Rz_RJoint_Front_Lower_Z_R)     


    linkTransformations.append(("RJoint_Front_Lower_Z_R", A_RJoint_Front_Lower_Z_R))


    M_Link_Front_Lower_R = np.array([[1.1924882414859894e-08,-3.2584136988589307e-07,1.0,0.29159072041511536],[1.5099578831723193e-07,1.0,3.2584136988589307e-07,0.0002992066147271544],[-1.0,1.5099580252808664e-07,1.1924933041029817e-08,0.011934686452150345],[0.0,0.0,0.0,1.0]])
    A_Link_Front_Lower_R = A_RJoint_Front_Lower_Z_R.dot(M_Link_Front_Lower_R)


    linkTransformations.append(("Link_Front_Lower_R", A_Link_Front_Lower_R))


    M_RJoint_Front_Ankle_Z_R = np.array([[1.1924882414859894e-08,-3.2584136988589307e-07,-1.0,0.011934665963053703],[-3.2584136988589307e-07,1.0,-3.2584136988589307e-07,-0.006699718534946442],[1.0,3.258413983076025e-07,1.192477760980637e-08,0.26048988103866577],[0.0,0.0,0.0,1.0]])
    A_RJoint_Front_Ankle_Z_R = A_Link_Front_Lower_R.dot(M_RJoint_Front_Ankle_Z_R)


    if "RJoint_Front_Ankle_Z_R" in q:
        theta_RJoint_Front_Ankle_Z_R = q["RJoint_Front_Ankle_Z_R"]
        Rz_RJoint_Front_Ankle_Z_R = np.array([ [ math.cos(theta_RJoint_Front_Ankle_Z_R), - math.sin(theta_RJoint_Front_Ankle_Z_R), 0, 0 ], [ math.sin(theta_RJoint_Front_Ankle_Z_R), math.cos(theta_RJoint_Front_Ankle_Z_R), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Front_Ankle_Z_R = A_RJoint_Front_Ankle_Z_R.dot(Rz_RJoint_Front_Ankle_Z_R)     


    linkTransformations.append(("RJoint_Front_Ankle_Z_R", A_RJoint_Front_Ankle_Z_R))


    M_Link_Front_Foot_R = np.array([[8.742277657347586e-08,-5.642599489874556e-07,1.0,0.19378723204135895],[-3.179046643708716e-07,1.0,5.642599489874556e-07,-0.022579794749617577],[-1.0,-3.1790469279258105e-07,8.742259183236456e-08,0.0070861950516700745],[0.0,0.0,0.0,1.0]])
    A_Link_Front_Foot_R = A_RJoint_Front_Ankle_Z_R.dot(M_Link_Front_Foot_R)


    linkTransformations.append(("Link_Front_Foot_R", A_Link_Front_Foot_R))


    M_Effector_Front_R = np.array([[-1.549839936387798e-07,-1.0,-5.523351092051598e-07,-7.089049347541732e-08],[1.0,-1.549837094216855e-07,-4.450507162800932e-07,0.007147040218114853],[4.450506310149649e-07,-5.523351660485787e-07,1.0,0.06602880358695984],[0.0,0.0,0.0,1.0]])
    A_Effector_Front_R = A_Link_Front_Foot_R.dot(M_Effector_Front_R)


    linkTransformations.append(("Effector_Front_R", A_Effector_Front_R))


    M_RJoint_Head_XYZ_C = np.array([[-4.3711398944878965e-08,1.0,-7.549790836947068e-08,3.047562202596055e-14],[-1.0,-4.3711395392165286e-08,4.371138828673793e-08,-1.0458284616470337],[4.371138473402425e-08,7.549790836947068e-08,1.0,0.39641857147216797],[0.0,0.0,0.0,1.0]])
    A_RJoint_Head_XYZ_C = A_Link_UpperBody_C.dot(M_RJoint_Head_XYZ_C)


    if "RJoint_Head_XYZ_C" in q:
        theta_RJoint_Head_XYZ_C = q["RJoint_Head_XYZ_C"]
        Rz_RJoint_Head_XYZ_C = np.array([ [ math.cos(theta_RJoint_Head_XYZ_C[2]), - math.sin(theta_RJoint_Head_XYZ_C[2]), 0, 0 ], [ math.sin(theta_RJoint_Head_XYZ_C[2]), math.cos(theta_RJoint_Head_XYZ_C[2]), 0, 0  ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.dot(Rz_RJoint_Head_XYZ_C)     
        Ry_RJoint_Head_XYZ_C = np.array([ [ math.cos(theta_RJoint_Head_XYZ_C[1]), 0, math.sin(theta_RJoint_Head_XYZ_C[1]), 0 ], [ 0, 1, 0, 0], [ - math.sin(theta_RJoint_Head_XYZ_C[1]), math.cos(theta_RJoint_Head_XYZ_C[1]), 0, 0  ], [0,0,0,1] ])
        A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.dot(Ry_RJoint_Head_XYZ_C)     
        Rx_RJoint_Head_XYZ_C = np.array([ [1, 0, 0, 0], [ 0, math.cos(theta_RJoint_Head_XYZ_C[0]), - math.sin(theta_RJoint_Head_XYZ_C[0]), 0 ], [ 0, math.sin(theta_RJoint_Head_XYZ_C[0]), math.cos(theta_RJoint_Head_XYZ_C[0]), 0 ], [ 0, 0, 1, 0 ], [0,0,0,1] ])
        A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.dot(Rx_RJoint_Head_XYZ_C)     


    linkTransformations.append(("RJoint_Head_XYZ_C", A_RJoint_Head_XYZ_C))


    M_Head_C = np.array([[3.1391647326017846e-07,-1.0,-4.371138473402425e-08,-1.0458284616470337],[1.0,3.1391647326017846e-07,-7.549790126404332e-08,-4.5714607210811664e-08],[7.549791547489804e-08,-4.37113598650285e-08,1.0,-0.39641866087913513],[0.0,0.0,0.0,1.0]])
    A_Head_C = A_RJoint_Head_XYZ_C.dot(M_Head_C)


    linkTransformations.append(("Head_C", A_Head_C))


    M_Effector_Head_C = np.array([[1.0,-2.0370159823247108e-14,2.2168405688070304e-14,-7.005406814641901e-07],[-9.379104345388564e-15,1.1920928955078125e-07,-1.0,-2.0093982219696045],[2.9367835068091694e-14,1.0,1.1920930376163597e-07,-0.05276213586330414],[0.0,0.0,0.0,1.0]])
    A_Effector_Head_C = A_Head_C.dot(M_Effector_Head_C)


    linkTransformations.append(("Effector_Head_C", A_Effector_Head_C))


    return linkTransformations

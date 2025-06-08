
import {Matrix4} from 'three';


type Rot3Angles = {x:number, y:number, z:number};

interface JointAngles {
  [key: string]: number | Rot3Angles;
}

type LinkTransformations = { [id:string] : Matrix4 };

function printMatrix(name:string, m:Matrix4) {
    console.log("Matrix", name);
    for(let i = 0; i < 4; i++) {
        console.log(m.elements[i+0*4].toFixed(2),m.elements[i+1*4].toFixed(2),m.elements[i+2*4].toFixed(2),m.elements[i+3*4].toFixed(2));
    }
    console.log("End of Matrix", name);
}


/**
 * Calculate forward kinematics.
 *
 * @param q A dictionary of joint values.
 *          For key "RJoint_???_???_XYZ_L" expect a number array of length 3.
 *          For key "RJoint_???_???_Z_L", expect a single number.
 * @param A An optional initial 4x4 transformation matrix.
 * @returns An array of tuples: [effectorName, transformationMatrix].
 */
function calcForwardKinematics(
  q: JointAngles,
  AIn?: Matrix4
): LinkTransformations {
     // Use identity if A is not provided.
    var A = new Matrix4( 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
    if (AIn !== undefined) {
        A = AIn;
    }
    
    //q = {
    //    "RJoint_Back_Upper_XYZ_L": {"x": 0.0/180.0*Math.PI, "y": 0.0/180.0*Math.PI, "z": 0.0/180.0*Math.PI },
    //    "RJoint_Back_Lower_Z_L": 20.0/180.0*Math.PI,
    //    "RJoint_Back_Ankle_Z_L": 30.0/180.0*Math.PI,
    //}

    var linkTransformations: LinkTransformations = {};


    const M_TaiwanBear = new Matrix4(1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0);
    var A_TaiwanBear =  new Matrix4();
    A_TaiwanBear.copy(A);
    //console.log("Parent matrix");
    //printMatrix("A", A);
    //console.log("Incoming matrix");
    //printMatrix("A_TaiwanBear",A_TaiwanBear); 
    A_TaiwanBear = A_TaiwanBear.multiply(M_TaiwanBear);
    //console.log(`M_TaiwanBear: ${M_TaiwanBear.elements[0+3*4]} ${M_TaiwanBear.elements[1+3*4]} ${M_TaiwanBear.elements[2+3*4]}`);
    //printMatrix("M_TaiwanBear", M_TaiwanBear);   
    //console.log(`A_TaiwanBear: ${A_TaiwanBear.elements[0+3*4]} ${A_TaiwanBear.elements[1+3*4]} ${A_TaiwanBear.elements[2+3*4]}`);
    //printMatrix("A_TaiwanBear", A_TaiwanBear);
    //console.log("Outgoing matrix");   


    linkTransformations["TaiwanBear"] = A_TaiwanBear;


    const M_RJoint_Back_Upper_XYZ_L = new Matrix4(1.0,0.0,0.0,0.03032732382416725,0.0,-4.371138828673793e-08,1.0,0.45000001788139343,0.0,-1.0,-4.371138828673793e-08,-0.23567400872707367,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Upper_XYZ_L =  new Matrix4();
    A_RJoint_Back_Upper_XYZ_L.copy(A_TaiwanBear);
    //console.log("Parent matrix");
    //printMatrix("A_TaiwanBear", A_TaiwanBear);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Back_Upper_XYZ_L",A_RJoint_Back_Upper_XYZ_L); 
    A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(M_RJoint_Back_Upper_XYZ_L);
    //console.log(`M_RJoint_Back_Upper_XYZ_L: ${M_RJoint_Back_Upper_XYZ_L.elements[0+3*4]} ${M_RJoint_Back_Upper_XYZ_L.elements[1+3*4]} ${M_RJoint_Back_Upper_XYZ_L.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Back_Upper_XYZ_L", M_RJoint_Back_Upper_XYZ_L);   
    //console.log(`A_RJoint_Back_Upper_XYZ_L: ${A_RJoint_Back_Upper_XYZ_L.elements[0+3*4]} ${A_RJoint_Back_Upper_XYZ_L.elements[1+3*4]} ${A_RJoint_Back_Upper_XYZ_L.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Upper_XYZ_L")) {
        const angles = q["RJoint_Back_Upper_XYZ_L"] as Rot3Angles;
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Back_Upper_XYZ_L = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(Rx_RJoint_Back_Upper_XYZ_L); 
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Back_Upper_XYZ_L = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(Ry_RJoint_Back_Upper_XYZ_L);     
        }
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Back_Upper_XYZ_L = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(Rz_RJoint_Back_Upper_XYZ_L);     
        }
        
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L);
    }


    linkTransformations["RJoint_Back_Upper_XYZ_L"] = A_RJoint_Back_Upper_XYZ_L;


    const M_Link_Back_Upper_L = new Matrix4(7.549790126404332e-08,-6.352747104407253e-22,-1.0,0.28632408380508423,4.3711398944878965e-08,1.0,3.3001182895339015e-15,0.00989803671836853,1.0,-4.3711398944878965e-08,7.549790126404332e-08,0.19395607709884644,0.0,0.0,0.0,1.0);
    var A_Link_Back_Upper_L =  new Matrix4();
    A_Link_Back_Upper_L.copy(A_RJoint_Back_Upper_XYZ_L);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Back_Upper_L",A_Link_Back_Upper_L); 
    A_Link_Back_Upper_L = A_Link_Back_Upper_L.multiply(M_Link_Back_Upper_L);
    //console.log(`M_Link_Back_Upper_L: ${M_Link_Back_Upper_L.elements[0+3*4]} ${M_Link_Back_Upper_L.elements[1+3*4]} ${M_Link_Back_Upper_L.elements[2+3*4]}`);
    //printMatrix("M_Link_Back_Upper_L", M_Link_Back_Upper_L);   
    //console.log(`A_Link_Back_Upper_L: ${A_Link_Back_Upper_L.elements[0+3*4]} ${A_Link_Back_Upper_L.elements[1+3*4]} ${A_Link_Back_Upper_L.elements[2+3*4]}`);
    //printMatrix("A_Link_Back_Upper_L", A_Link_Back_Upper_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Back_Upper_L"] = A_Link_Back_Upper_L;


    const M_RJoint_Back_Lower_Z_L = new Matrix4(5.52335052361741e-07,4.3711398944878965e-08,1.0,-0.020895827561616898,-2.0843219253959162e-14,1.0,-4.3711398944878965e-08,-0.00965034682303667,-1.0,3.3001182895339015e-15,5.52335052361741e-07,-0.40017712116241455,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Lower_Z_L =  new Matrix4();
    A_RJoint_Back_Lower_Z_L.copy(A_Link_Back_Upper_L);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Back_Upper_L", A_Link_Back_Upper_L);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Back_Lower_Z_L",A_RJoint_Back_Lower_Z_L); 
    A_RJoint_Back_Lower_Z_L = A_RJoint_Back_Lower_Z_L.multiply(M_RJoint_Back_Lower_Z_L);
    //console.log(`M_RJoint_Back_Lower_Z_L: ${M_RJoint_Back_Lower_Z_L.elements[0+3*4]} ${M_RJoint_Back_Lower_Z_L.elements[1+3*4]} ${M_RJoint_Back_Lower_Z_L.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Back_Lower_Z_L", M_RJoint_Back_Lower_Z_L);   
    //console.log(`A_RJoint_Back_Lower_Z_L: ${A_RJoint_Back_Lower_Z_L.elements[0+3*4]} ${A_RJoint_Back_Lower_Z_L.elements[1+3*4]} ${A_RJoint_Back_Lower_Z_L.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Back_Lower_Z_L", A_RJoint_Back_Lower_Z_L);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Lower_Z_L")) {
        const z = q["RJoint_Back_Lower_Z_L"] as number;
        const Rz_RJoint_Back_Lower_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Lower_Z_L.multiply(Rz_RJoint_Back_Lower_Z_L);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Back_Lower_Z_L", A_RJoint_Back_Lower_Z_L);
    }


    linkTransformations["RJoint_Back_Lower_Z_L"] = A_RJoint_Back_Lower_Z_L;


    const M_Link_Back_Lower_L = new Matrix4(1.9470718370939721e-07,1.0331601174584648e-07,-1.0,0.3016183376312256,-7.105431592765738e-15,1.0,1.0331601174584648e-07,0.009650424122810364,1.0,-1.2774402033139537e-14,1.9470718370939721e-07,0.01826903596520424,0.0,0.0,0.0,1.0);
    var A_Link_Back_Lower_L =  new Matrix4();
    A_Link_Back_Lower_L.copy(A_RJoint_Back_Lower_Z_L);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Back_Lower_Z_L", A_RJoint_Back_Lower_Z_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Back_Lower_L",A_Link_Back_Lower_L); 
    A_Link_Back_Lower_L = A_Link_Back_Lower_L.multiply(M_Link_Back_Lower_L);
    //console.log(`M_Link_Back_Lower_L: ${M_Link_Back_Lower_L.elements[0+3*4]} ${M_Link_Back_Lower_L.elements[1+3*4]} ${M_Link_Back_Lower_L.elements[2+3*4]}`);
    //printMatrix("M_Link_Back_Lower_L", M_Link_Back_Lower_L);   
    //console.log(`A_Link_Back_Lower_L: ${A_Link_Back_Lower_L.elements[0+3*4]} ${A_Link_Back_Lower_L.elements[1+3*4]} ${A_Link_Back_Lower_L.elements[2+3*4]}`);
    //printMatrix("A_Link_Back_Lower_L", A_Link_Back_Lower_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Back_Lower_L"] = A_Link_Back_Lower_L;


    const M_RJoint_Back_Ankle_Z_L = new Matrix4(3.1391647326017846e-07,4.371142026116104e-08,1.0,-0.018269008025527,1.0331601174584648e-07,1.0,-4.371145223558415e-08,-0.009650135412812233,-1.0,1.033160259567012e-07,3.1391647326017846e-07,-0.33216747641563416,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Ankle_Z_L =  new Matrix4();
    A_RJoint_Back_Ankle_Z_L.copy(A_Link_Back_Lower_L);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Back_Lower_L", A_Link_Back_Lower_L);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Back_Ankle_Z_L",A_RJoint_Back_Ankle_Z_L); 
    A_RJoint_Back_Ankle_Z_L = A_RJoint_Back_Ankle_Z_L.multiply(M_RJoint_Back_Ankle_Z_L);
    //console.log(`M_RJoint_Back_Ankle_Z_L: ${M_RJoint_Back_Ankle_Z_L.elements[0+3*4]} ${M_RJoint_Back_Ankle_Z_L.elements[1+3*4]} ${M_RJoint_Back_Ankle_Z_L.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Back_Ankle_Z_L", M_RJoint_Back_Ankle_Z_L);   
    //console.log(`A_RJoint_Back_Ankle_Z_L: ${A_RJoint_Back_Ankle_Z_L.elements[0+3*4]} ${A_RJoint_Back_Ankle_Z_L.elements[1+3*4]} ${A_RJoint_Back_Ankle_Z_L.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Back_Ankle_Z_L", A_RJoint_Back_Ankle_Z_L);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Ankle_Z_L")) {
        const z = q["RJoint_Back_Ankle_Z_L"] as number;
        const Rz_RJoint_Back_Ankle_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Ankle_Z_L.multiply(Rz_RJoint_Back_Ankle_Z_L);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Back_Ankle_Z_L", A_RJoint_Back_Ankle_Z_L);
    }


    linkTransformations["RJoint_Back_Ankle_Z_L"] = A_RJoint_Back_Ankle_Z_L;


    const M_Link_Back_Foot_L = new Matrix4(5.52335052361741e-07,-2.4143359566349692e-14,-1.0,0.2357282042503357,-1.7763565005870716e-14,1.0,-2.414336973074506e-14,-0.13118453323841095,1.0,1.776360396938629e-14,5.52335052361741e-07,-0.0003760427061934024,0.0,0.0,0.0,1.0);
    var A_Link_Back_Foot_L =  new Matrix4();
    A_Link_Back_Foot_L.copy(A_RJoint_Back_Ankle_Z_L);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Back_Ankle_Z_L", A_RJoint_Back_Ankle_Z_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Back_Foot_L",A_Link_Back_Foot_L); 
    A_Link_Back_Foot_L = A_Link_Back_Foot_L.multiply(M_Link_Back_Foot_L);
    //console.log(`M_Link_Back_Foot_L: ${M_Link_Back_Foot_L.elements[0+3*4]} ${M_Link_Back_Foot_L.elements[1+3*4]} ${M_Link_Back_Foot_L.elements[2+3*4]}`);
    //printMatrix("M_Link_Back_Foot_L", M_Link_Back_Foot_L);   
    //console.log(`A_Link_Back_Foot_L: ${A_Link_Back_Foot_L.elements[0+3*4]} ${A_Link_Back_Foot_L.elements[1+3*4]} ${A_Link_Back_Foot_L.elements[2+3*4]}`);
    //printMatrix("A_Link_Back_Foot_L", A_Link_Back_Foot_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Back_Foot_L"] = A_Link_Back_Foot_L;


    const M_Effector_Back_L = new Matrix4(-1.1920922560193503e-07,-1.0,9.218878176397993e-07,-2.4024869205163668e-08,-1.0,1.1920911191509731e-07,-1.1920946008103783e-07,-0.0003884907637257129,1.192093463942001e-07,-9.218878176397993e-07,-1.0,-0.07693012058734894,0.0,0.0,0.0,1.0);
    var A_Effector_Back_L =  new Matrix4();
    A_Effector_Back_L.copy(A_Link_Back_Foot_L);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Back_Foot_L", A_Link_Back_Foot_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Effector_Back_L",A_Effector_Back_L); 
    A_Effector_Back_L = A_Effector_Back_L.multiply(M_Effector_Back_L);
    //console.log(`M_Effector_Back_L: ${M_Effector_Back_L.elements[0+3*4]} ${M_Effector_Back_L.elements[1+3*4]} ${M_Effector_Back_L.elements[2+3*4]}`);
    //printMatrix("M_Effector_Back_L", M_Effector_Back_L);   
    //console.log(`A_Effector_Back_L: ${A_Effector_Back_L.elements[0+3*4]} ${A_Effector_Back_L.elements[1+3*4]} ${A_Effector_Back_L.elements[2+3*4]}`);
    //printMatrix("A_Effector_Back_L", A_Effector_Back_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Effector_Back_L"] = A_Effector_Back_L;


    const M_RJoint_Back_Upper_XYZ_R = new Matrix4(1.0,6.6002361555513294e-15,-3.178651297730539e-08,0.03032725676894188,-3.178651297730539e-08,1.3113415775478643e-07,-1.0,-0.44999995827674866,-2.431938741234924e-15,1.0,1.3113417196564114e-07,-0.2349959909915924,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Upper_XYZ_R =  new Matrix4();
    A_RJoint_Back_Upper_XYZ_R.copy(A_TaiwanBear);
    //console.log("Parent matrix");
    //printMatrix("A_TaiwanBear", A_TaiwanBear);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Back_Upper_XYZ_R",A_RJoint_Back_Upper_XYZ_R); 
    A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(M_RJoint_Back_Upper_XYZ_R);
    //console.log(`M_RJoint_Back_Upper_XYZ_R: ${M_RJoint_Back_Upper_XYZ_R.elements[0+3*4]} ${M_RJoint_Back_Upper_XYZ_R.elements[1+3*4]} ${M_RJoint_Back_Upper_XYZ_R.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Back_Upper_XYZ_R", M_RJoint_Back_Upper_XYZ_R);   
    //console.log(`A_RJoint_Back_Upper_XYZ_R: ${A_RJoint_Back_Upper_XYZ_R.elements[0+3*4]} ${A_RJoint_Back_Upper_XYZ_R.elements[1+3*4]} ${A_RJoint_Back_Upper_XYZ_R.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Upper_XYZ_R")) {
        const angles = q["RJoint_Back_Upper_XYZ_R"] as Rot3Angles;
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Back_Upper_XYZ_R = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(Rx_RJoint_Back_Upper_XYZ_R); 
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Back_Upper_XYZ_R = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(Ry_RJoint_Back_Upper_XYZ_R);     
        }
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Back_Upper_XYZ_R = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(Rz_RJoint_Back_Upper_XYZ_R);     
        }
        
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R);
    }


    linkTransformations["RJoint_Back_Upper_XYZ_R"] = A_RJoint_Back_Upper_XYZ_R;


    const M_Link_Back_Upper_R = new Matrix4(7.549790126404332e-08,1.5099581673894136e-07,1.0,0.28632378578186035,-1.0410971071905806e-06,1.0,-1.5099573147381307e-07,-0.009897496551275253,-1.0,-1.0410971071905806e-06,7.549805758344519e-08,0.19395612180233002,0.0,0.0,0.0,1.0);
    var A_Link_Back_Upper_R =  new Matrix4();
    A_Link_Back_Upper_R.copy(A_RJoint_Back_Upper_XYZ_R);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Back_Upper_R",A_Link_Back_Upper_R); 
    A_Link_Back_Upper_R = A_Link_Back_Upper_R.multiply(M_Link_Back_Upper_R);
    //console.log(`M_Link_Back_Upper_R: ${M_Link_Back_Upper_R.elements[0+3*4]} ${M_Link_Back_Upper_R.elements[1+3*4]} ${M_Link_Back_Upper_R.elements[2+3*4]}`);
    //printMatrix("M_Link_Back_Upper_R", M_Link_Back_Upper_R);   
    //console.log(`A_Link_Back_Upper_R: ${A_Link_Back_Upper_R.elements[0+3*4]} ${A_Link_Back_Upper_R.elements[1+3*4]} ${A_Link_Back_Upper_R.elements[2+3*4]}`);
    //printMatrix("A_Link_Back_Upper_R", A_Link_Back_Upper_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Back_Upper_R"] = A_Link_Back_Upper_R;


    const M_RJoint_Back_Lower_Z_R = new Matrix4(-5.591472245214391e-07,7.670207082810521e-07,-1.0,0.020895889028906822,-0.7071066498756409,0.7071068286895752,9.377424703416182e-07,0.00965085718780756,0.7071068286895752,0.7071066498756409,1.4698869676976756e-07,0.40017715096473694,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Lower_Z_R =  new Matrix4();
    A_RJoint_Back_Lower_Z_R.copy(A_Link_Back_Upper_R);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Back_Upper_R", A_Link_Back_Upper_R);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Back_Lower_Z_R",A_RJoint_Back_Lower_Z_R); 
    A_RJoint_Back_Lower_Z_R = A_RJoint_Back_Lower_Z_R.multiply(M_RJoint_Back_Lower_Z_R);
    //console.log(`M_RJoint_Back_Lower_Z_R: ${M_RJoint_Back_Lower_Z_R.elements[0+3*4]} ${M_RJoint_Back_Lower_Z_R.elements[1+3*4]} ${M_RJoint_Back_Lower_Z_R.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Back_Lower_Z_R", M_RJoint_Back_Lower_Z_R);   
    //console.log(`A_RJoint_Back_Lower_Z_R: ${A_RJoint_Back_Lower_Z_R.elements[0+3*4]} ${A_RJoint_Back_Lower_Z_R.elements[1+3*4]} ${A_RJoint_Back_Lower_Z_R.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Back_Lower_Z_R", A_RJoint_Back_Lower_Z_R);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Lower_Z_R")) {
        const z = q["RJoint_Back_Lower_Z_R"] as number;
        const Rz_RJoint_Back_Lower_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Lower_Z_R.multiply(Rz_RJoint_Back_Lower_Z_R);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Back_Lower_Z_R", A_RJoint_Back_Lower_Z_R);
    }


    linkTransformations["RJoint_Back_Lower_Z_R"] = A_RJoint_Back_Lower_Z_R;


    const M_Link_Back_Lower_R = new Matrix4(1.1925010312552331e-08,1.4900862765898637e-07,1.0,0.3016180992126465,-6.1989601363166e-07,1.0,-1.4901375777753856e-07,-0.009650563821196556,-1.0,-6.198962978487543e-07,1.1925003207124973e-08,0.018269240856170654,0.0,0.0,0.0,1.0);
    var A_Link_Back_Lower_R =  new Matrix4();
    A_Link_Back_Lower_R.copy(A_RJoint_Back_Lower_Z_R);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Back_Lower_Z_R", A_RJoint_Back_Lower_Z_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Back_Lower_R",A_Link_Back_Lower_R); 
    A_Link_Back_Lower_R = A_Link_Back_Lower_R.multiply(M_Link_Back_Lower_R);
    //console.log(`M_Link_Back_Lower_R: ${M_Link_Back_Lower_R.elements[0+3*4]} ${M_Link_Back_Lower_R.elements[1+3*4]} ${M_Link_Back_Lower_R.elements[2+3*4]}`);
    //printMatrix("M_Link_Back_Lower_R", M_Link_Back_Lower_R);   
    //console.log(`A_Link_Back_Lower_R: ${A_Link_Back_Lower_R.elements[0+3*4]} ${A_Link_Back_Lower_R.elements[1+3*4]} ${A_Link_Back_Lower_R.elements[2+3*4]}`);
    //printMatrix("A_Link_Back_Lower_R", A_Link_Back_Lower_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Back_Lower_R"] = A_Link_Back_Lower_R;


    const M_RJoint_Back_Ankle_Z_R = new Matrix4(4.3711366970455856e-08,8.741881174501032e-07,-1.0,0.018269071355462074,1.4900869871325995e-07,1.0,8.741880037632654e-07,0.00965025369077921,1.0,-1.4901583256232698e-07,4.3711338548746426e-08,0.3321675956249237,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Ankle_Z_R =  new Matrix4();
    A_RJoint_Back_Ankle_Z_R.copy(A_Link_Back_Lower_R);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Back_Lower_R", A_Link_Back_Lower_R);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Back_Ankle_Z_R",A_RJoint_Back_Ankle_Z_R); 
    A_RJoint_Back_Ankle_Z_R = A_RJoint_Back_Ankle_Z_R.multiply(M_RJoint_Back_Ankle_Z_R);
    //console.log(`M_RJoint_Back_Ankle_Z_R: ${M_RJoint_Back_Ankle_Z_R.elements[0+3*4]} ${M_RJoint_Back_Ankle_Z_R.elements[1+3*4]} ${M_RJoint_Back_Ankle_Z_R.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Back_Ankle_Z_R", M_RJoint_Back_Ankle_Z_R);   
    //console.log(`A_RJoint_Back_Ankle_Z_R: ${A_RJoint_Back_Ankle_Z_R.elements[0+3*4]} ${A_RJoint_Back_Ankle_Z_R.elements[1+3*4]} ${A_RJoint_Back_Ankle_Z_R.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Back_Ankle_Z_R", A_RJoint_Back_Ankle_Z_R);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Ankle_Z_R")) {
        const z = q["RJoint_Back_Ankle_Z_R"] as number;
        const Rz_RJoint_Back_Ankle_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Ankle_Z_R.multiply(Rz_RJoint_Back_Ankle_Z_R);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Back_Ankle_Z_R", A_RJoint_Back_Ankle_Z_R);
    }


    linkTransformations["RJoint_Back_Ankle_Z_R"] = A_RJoint_Back_Ankle_Z_R;


    const M_Link_Back_Foot_R = new Matrix4(1.1924726095458027e-08,1.4900611233770178e-07,1.0,0.22246640920639038,-1.0410968798169051e-06,1.0,-1.4901574729719869e-07,-0.00043041844037361443,-1.0,-1.041097334564256e-06,1.192495346913347e-08,-0.00037576310569420457,0.0,0.0,0.0,1.0);
    var A_Link_Back_Foot_R =  new Matrix4();
    A_Link_Back_Foot_R.copy(A_RJoint_Back_Ankle_Z_R);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Back_Ankle_Z_R", A_RJoint_Back_Ankle_Z_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Back_Foot_R",A_Link_Back_Foot_R); 
    A_Link_Back_Foot_R = A_Link_Back_Foot_R.multiply(M_Link_Back_Foot_R);
    //console.log(`M_Link_Back_Foot_R: ${M_Link_Back_Foot_R.elements[0+3*4]} ${M_Link_Back_Foot_R.elements[1+3*4]} ${M_Link_Back_Foot_R.elements[2+3*4]}`);
    //printMatrix("M_Link_Back_Foot_R", M_Link_Back_Foot_R);   
    //console.log(`A_Link_Back_Foot_R: ${A_Link_Back_Foot_R.elements[0+3*4]} ${A_Link_Back_Foot_R.elements[1+3*4]} ${A_Link_Back_Foot_R.elements[2+3*4]}`);
    //printMatrix("A_Link_Back_Foot_R", A_Link_Back_Foot_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Back_Foot_R"] = A_Link_Back_Foot_R;


    const M_Effector_Back_R = new Matrix4(7.033305564618786e-07,-1.0,1.748458089423366e-07,2.1013192963437177e-08,0.9999999403953552,7.033312385829049e-07,5.066346489002171e-07,0.0003884673351421952,-5.066459607405704e-07,1.748453541949857e-07,0.9999999403953552,0.07693018019199371,0.0,0.0,0.0,1.0);
    var A_Effector_Back_R =  new Matrix4();
    A_Effector_Back_R.copy(A_Link_Back_Foot_R);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Back_Foot_R", A_Link_Back_Foot_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Effector_Back_R",A_Effector_Back_R); 
    A_Effector_Back_R = A_Effector_Back_R.multiply(M_Effector_Back_R);
    //console.log(`M_Effector_Back_R: ${M_Effector_Back_R.elements[0+3*4]} ${M_Effector_Back_R.elements[1+3*4]} ${M_Effector_Back_R.elements[2+3*4]}`);
    //printMatrix("M_Effector_Back_R", M_Effector_Back_R);   
    //console.log(`A_Effector_Back_R: ${A_Effector_Back_R.elements[0+3*4]} ${A_Effector_Back_R.elements[1+3*4]} ${A_Effector_Back_R.elements[2+3*4]}`);
    //printMatrix("A_Effector_Back_R", A_Effector_Back_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Effector_Back_R"] = A_Effector_Back_R;


    const M_RJoint_Torso_XYZ_C = new Matrix4(2.3700034380218053e-14,7.549790126404332e-08,-1.0,1.1250063965327043e-15,3.5762786865234375e-07,1.0,7.549790126404332e-08,5.380906031859922e-08,1.0,-3.5762786865234375e-07,-3.300117866017428e-15,0.8901089429855347,0.0,0.0,0.0,1.0);
    var A_RJoint_Torso_XYZ_C =  new Matrix4();
    A_RJoint_Torso_XYZ_C.copy(A_TaiwanBear);
    //console.log("Parent matrix");
    //printMatrix("A_TaiwanBear", A_TaiwanBear);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Torso_XYZ_C",A_RJoint_Torso_XYZ_C); 
    A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(M_RJoint_Torso_XYZ_C);
    //console.log(`M_RJoint_Torso_XYZ_C: ${M_RJoint_Torso_XYZ_C.elements[0+3*4]} ${M_RJoint_Torso_XYZ_C.elements[1+3*4]} ${M_RJoint_Torso_XYZ_C.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Torso_XYZ_C", M_RJoint_Torso_XYZ_C);   
    //console.log(`A_RJoint_Torso_XYZ_C: ${A_RJoint_Torso_XYZ_C.elements[0+3*4]} ${A_RJoint_Torso_XYZ_C.elements[1+3*4]} ${A_RJoint_Torso_XYZ_C.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Torso_XYZ_C")) {
        const angles = q["RJoint_Torso_XYZ_C"] as Rot3Angles;
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Torso_XYZ_C = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(Rx_RJoint_Torso_XYZ_C); 
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Torso_XYZ_C = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(Ry_RJoint_Torso_XYZ_C);     
        }
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Torso_XYZ_C = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(Rz_RJoint_Torso_XYZ_C);     
        }
        
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    }


    linkTransformations["RJoint_Torso_XYZ_C"] = A_RJoint_Torso_XYZ_C;


    const M_Link_UpperBody_C = new Matrix4(3.1391647326017846e-07,-1.0,-1.4425274595939708e-22,-4.440892098500626e-15,1.0,3.1391647326017846e-07,-1.4425269547229914e-22,-2.899381001952861e-08,-1.4425274595939708e-22,-1.4425269547229914e-22,1.0,-0.3840346932411194,0.0,0.0,0.0,1.0);
    var A_Link_UpperBody_C =  new Matrix4();
    A_Link_UpperBody_C.copy(A_RJoint_Torso_XYZ_C);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_UpperBody_C",A_Link_UpperBody_C); 
    A_Link_UpperBody_C = A_Link_UpperBody_C.multiply(M_Link_UpperBody_C);
    //console.log(`M_Link_UpperBody_C: ${M_Link_UpperBody_C.elements[0+3*4]} ${M_Link_UpperBody_C.elements[1+3*4]} ${M_Link_UpperBody_C.elements[2+3*4]}`);
    //printMatrix("M_Link_UpperBody_C", M_Link_UpperBody_C);   
    //console.log(`A_Link_UpperBody_C: ${A_Link_UpperBody_C.elements[0+3*4]} ${A_Link_UpperBody_C.elements[1+3*4]} ${A_Link_UpperBody_C.elements[2+3*4]}`);
    //printMatrix("A_Link_UpperBody_C", A_Link_UpperBody_C);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_UpperBody_C"] = A_Link_UpperBody_C;


    const M_RJoint_Front_Upper_XYZ_L = new Matrix4(1.5099581673894136e-07,2.0384551512339162e-21,1.0,0.4500000476837158,-4.371141670844736e-08,1.0,3.3001180777756647e-15,-0.6500011682510376,-1.0,-4.371141670844736e-08,1.5099581673894136e-07,0.3616074323654175,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Upper_XYZ_L =  new Matrix4();
    A_RJoint_Front_Upper_XYZ_L.copy(A_Link_UpperBody_C);
    //console.log("Parent matrix");
    //printMatrix("A_Link_UpperBody_C", A_Link_UpperBody_C);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Front_Upper_XYZ_L",A_RJoint_Front_Upper_XYZ_L); 
    A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(M_RJoint_Front_Upper_XYZ_L);
    //console.log(`M_RJoint_Front_Upper_XYZ_L: ${M_RJoint_Front_Upper_XYZ_L.elements[0+3*4]} ${M_RJoint_Front_Upper_XYZ_L.elements[1+3*4]} ${M_RJoint_Front_Upper_XYZ_L.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Front_Upper_XYZ_L", M_RJoint_Front_Upper_XYZ_L);   
    //console.log(`A_RJoint_Front_Upper_XYZ_L: ${A_RJoint_Front_Upper_XYZ_L.elements[0+3*4]} ${A_RJoint_Front_Upper_XYZ_L.elements[1+3*4]} ${A_RJoint_Front_Upper_XYZ_L.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Upper_XYZ_L")) {
        const angles = q["RJoint_Front_Upper_XYZ_L"] as Rot3Angles;
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Front_Upper_XYZ_L = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(Rx_RJoint_Front_Upper_XYZ_L); 
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Front_Upper_XYZ_L = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(Ry_RJoint_Front_Upper_XYZ_L);     
        }
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Front_Upper_XYZ_L = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(Rz_RJoint_Front_Upper_XYZ_L);     
        }
        
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L);
    }


    linkTransformations["RJoint_Front_Upper_XYZ_L"] = A_RJoint_Front_Upper_XYZ_L;


    const M_Link_Front_Upper_L = new Matrix4(1.5099580252808664e-07,-4.371141670844736e-08,-1.0,0.4007798433303833,4.3711395392165286e-08,1.0,-4.371141315573368e-08,0.00921894982457161,1.0,-4.371139183945161e-08,1.5099581673894136e-07,0.0664963647723198,0.0,0.0,0.0,1.0);
    var A_Link_Front_Upper_L =  new Matrix4();
    A_Link_Front_Upper_L.copy(A_RJoint_Front_Upper_XYZ_L);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Front_Upper_L",A_Link_Front_Upper_L); 
    A_Link_Front_Upper_L = A_Link_Front_Upper_L.multiply(M_Link_Front_Upper_L);
    //console.log(`M_Link_Front_Upper_L: ${M_Link_Front_Upper_L.elements[0+3*4]} ${M_Link_Front_Upper_L.elements[1+3*4]} ${M_Link_Front_Upper_L.elements[2+3*4]}`);
    //printMatrix("M_Link_Front_Upper_L", M_Link_Front_Upper_L);   
    //console.log(`A_Link_Front_Upper_L: ${A_Link_Front_Upper_L.elements[0+3*4]} ${A_Link_Front_Upper_L.elements[1+3*4]} ${A_Link_Front_Upper_L.elements[2+3*4]}`);
    //printMatrix("A_Link_Front_Upper_L", A_Link_Front_Upper_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Front_Upper_L"] = A_Link_Front_Upper_L;


    const M_RJoint_Front_Lower_Z_L = new Matrix4(1.9470718370939721e-07,2.842170943040401e-14,1.0,-0.012298770248889923,-4.958207730157297e-15,1.0,-2.1316282072803006e-14,-0.009218715131282806,-1.0,-1.4054911926205988e-15,1.9470718370939721e-07,-0.4276049733161926,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Lower_Z_L =  new Matrix4();
    A_RJoint_Front_Lower_Z_L.copy(A_Link_Front_Upper_L);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Front_Upper_L", A_Link_Front_Upper_L);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Front_Lower_Z_L",A_RJoint_Front_Lower_Z_L); 
    A_RJoint_Front_Lower_Z_L = A_RJoint_Front_Lower_Z_L.multiply(M_RJoint_Front_Lower_Z_L);
    //console.log(`M_RJoint_Front_Lower_Z_L: ${M_RJoint_Front_Lower_Z_L.elements[0+3*4]} ${M_RJoint_Front_Lower_Z_L.elements[1+3*4]} ${M_RJoint_Front_Lower_Z_L.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Front_Lower_Z_L", M_RJoint_Front_Lower_Z_L);   
    //console.log(`A_RJoint_Front_Lower_Z_L: ${A_RJoint_Front_Lower_Z_L.elements[0+3*4]} ${A_RJoint_Front_Lower_Z_L.elements[1+3*4]} ${A_RJoint_Front_Lower_Z_L.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Front_Lower_Z_L", A_RJoint_Front_Lower_Z_L);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Lower_Z_L")) {
        const z = q["RJoint_Front_Lower_Z_L"] as number;
        const Rz_RJoint_Front_Lower_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Lower_Z_L.multiply(Rz_RJoint_Front_Lower_Z_L);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Front_Lower_Z_L", A_RJoint_Front_Lower_Z_L);
    }


    linkTransformations["RJoint_Front_Lower_Z_L"] = A_RJoint_Front_Lower_Z_L;


    const M_Link_Front_Lower_L = new Matrix4(1.5099581673894136e-07,-4.371141670844736e-08,-1.0,0.2915906608104706,2.4868995751603507e-14,1.0,-4.371142026116104e-08,-1.4177012985783222e-07,1.0,-1.9910783362765e-14,1.5099581673894136e-07,0.011934527195990086,0.0,0.0,0.0,1.0);
    var A_Link_Front_Lower_L =  new Matrix4();
    A_Link_Front_Lower_L.copy(A_RJoint_Front_Lower_Z_L);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Front_Lower_Z_L", A_RJoint_Front_Lower_Z_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Front_Lower_L",A_Link_Front_Lower_L); 
    A_Link_Front_Lower_L = A_Link_Front_Lower_L.multiply(M_Link_Front_Lower_L);
    //console.log(`M_Link_Front_Lower_L: ${M_Link_Front_Lower_L.elements[0+3*4]} ${M_Link_Front_Lower_L.elements[1+3*4]} ${M_Link_Front_Lower_L.elements[2+3*4]}`);
    //printMatrix("M_Link_Front_Lower_L", M_Link_Front_Lower_L);   
    //console.log(`A_Link_Front_Lower_L: ${A_Link_Front_Lower_L.elements[0+3*4]} ${A_Link_Front_Lower_L.elements[1+3*4]} ${A_Link_Front_Lower_L.elements[2+3*4]}`);
    //printMatrix("A_Link_Front_Lower_L", A_Link_Front_Lower_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Front_Lower_L"] = A_Link_Front_Lower_L;


    const M_RJoint_Front_Ankle_Z_L = new Matrix4(2.702051062897226e-07,4.013392356227996e-07,1.0,-0.011934826150536537,-4.371142381387472e-08,1.0,-4.013392356227996e-07,-0.0031993030570447445,-1.0,-4.371131723246435e-08,2.702051062897226e-07,-0.2604900002479553,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Ankle_Z_L =  new Matrix4();
    A_RJoint_Front_Ankle_Z_L.copy(A_Link_Front_Lower_L);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Front_Lower_L", A_Link_Front_Lower_L);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Front_Ankle_Z_L",A_RJoint_Front_Ankle_Z_L); 
    A_RJoint_Front_Ankle_Z_L = A_RJoint_Front_Ankle_Z_L.multiply(M_RJoint_Front_Ankle_Z_L);
    //console.log(`M_RJoint_Front_Ankle_Z_L: ${M_RJoint_Front_Ankle_Z_L.elements[0+3*4]} ${M_RJoint_Front_Ankle_Z_L.elements[1+3*4]} ${M_RJoint_Front_Ankle_Z_L.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Front_Ankle_Z_L", M_RJoint_Front_Ankle_Z_L);   
    //console.log(`A_RJoint_Front_Ankle_Z_L: ${A_RJoint_Front_Ankle_Z_L.elements[0+3*4]} ${A_RJoint_Front_Ankle_Z_L.elements[1+3*4]} ${A_RJoint_Front_Ankle_Z_L.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Front_Ankle_Z_L", A_RJoint_Front_Ankle_Z_L);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Ankle_Z_L")) {
        const z = q["RJoint_Front_Ankle_Z_L"] as number;
        const Rz_RJoint_Front_Ankle_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Ankle_Z_L.multiply(Rz_RJoint_Front_Ankle_Z_L);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Front_Ankle_Z_L", A_RJoint_Front_Ankle_Z_L);
    }


    linkTransformations["RJoint_Front_Ankle_Z_L"] = A_RJoint_Front_Ankle_Z_L;


    const M_Link_Front_Foot_L = new Matrix4(1.5099580252808664e-07,-4.371141670844736e-08,-1.0,0.19831721484661102,2.842172637106295e-14,1.0,-4.371142026116104e-08,0.01238197460770607,1.0,-1.469994000927611e-14,1.5099580252808664e-07,0.007086142897605896,0.0,0.0,0.0,1.0);
    var A_Link_Front_Foot_L =  new Matrix4();
    A_Link_Front_Foot_L.copy(A_RJoint_Front_Ankle_Z_L);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Front_Ankle_Z_L", A_RJoint_Front_Ankle_Z_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Front_Foot_L",A_Link_Front_Foot_L); 
    A_Link_Front_Foot_L = A_Link_Front_Foot_L.multiply(M_Link_Front_Foot_L);
    //console.log(`M_Link_Front_Foot_L: ${M_Link_Front_Foot_L.elements[0+3*4]} ${M_Link_Front_Foot_L.elements[1+3*4]} ${M_Link_Front_Foot_L.elements[2+3*4]}`);
    //printMatrix("M_Link_Front_Foot_L", M_Link_Front_Foot_L);   
    //console.log(`A_Link_Front_Foot_L: ${A_Link_Front_Foot_L.elements[0+3*4]} ${A_Link_Front_Foot_L.elements[1+3*4]} ${A_Link_Front_Foot_L.elements[2+3*4]}`);
    //printMatrix("A_Link_Front_Foot_L", A_Link_Front_Foot_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Front_Foot_L"] = A_Link_Front_Foot_L;


    const M_Effector_Front_L = new Matrix4(-4.013393208879279e-07,-1.0,1.1165950581926154e-06,-1.8681419078347972e-08,-1.0,4.013392072010902e-07,-7.54984412765225e-08,-0.006353148724883795,7.549799363459897e-08,-1.1165950581926154e-06,-1.0,-0.05869235098361969,0.0,0.0,0.0,1.0);
    var A_Effector_Front_L =  new Matrix4();
    A_Effector_Front_L.copy(A_Link_Front_Foot_L);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Front_Foot_L", A_Link_Front_Foot_L);
    //console.log("Incoming matrix");
    //printMatrix("A_Effector_Front_L",A_Effector_Front_L); 
    A_Effector_Front_L = A_Effector_Front_L.multiply(M_Effector_Front_L);
    //console.log(`M_Effector_Front_L: ${M_Effector_Front_L.elements[0+3*4]} ${M_Effector_Front_L.elements[1+3*4]} ${M_Effector_Front_L.elements[2+3*4]}`);
    //printMatrix("M_Effector_Front_L", M_Effector_Front_L);   
    //console.log(`A_Effector_Front_L: ${A_Effector_Front_L.elements[0+3*4]} ${A_Effector_Front_L.elements[1+3*4]} ${A_Effector_Front_L.elements[2+3*4]}`);
    //printMatrix("A_Effector_Front_L", A_Effector_Front_L);
    //console.log("Outgoing matrix");   


    linkTransformations["Effector_Front_L"] = A_Effector_Front_L;


    const M_RJoint_Front_Upper_XYZ_R = new Matrix4(1.5099581673894136e-07,3.178649876645068e-08,-1.0,-0.4500000476837158,-4.3711427366588396e-08,-1.0,-3.1786502319164356e-08,-0.6499999761581421,-1.0,4.3711430919302074e-08,-1.5099581673894136e-07,0.3616074323654175,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Upper_XYZ_R =  new Matrix4();
    A_RJoint_Front_Upper_XYZ_R.copy(A_Link_UpperBody_C);
    //console.log("Parent matrix");
    //printMatrix("A_Link_UpperBody_C", A_Link_UpperBody_C);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Front_Upper_XYZ_R",A_RJoint_Front_Upper_XYZ_R); 
    A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(M_RJoint_Front_Upper_XYZ_R);
    //console.log(`M_RJoint_Front_Upper_XYZ_R: ${M_RJoint_Front_Upper_XYZ_R.elements[0+3*4]} ${M_RJoint_Front_Upper_XYZ_R.elements[1+3*4]} ${M_RJoint_Front_Upper_XYZ_R.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Front_Upper_XYZ_R", M_RJoint_Front_Upper_XYZ_R);   
    //console.log(`A_RJoint_Front_Upper_XYZ_R: ${A_RJoint_Front_Upper_XYZ_R.elements[0+3*4]} ${A_RJoint_Front_Upper_XYZ_R.elements[1+3*4]} ${A_RJoint_Front_Upper_XYZ_R.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Upper_XYZ_R")) {
        const angles = q["RJoint_Front_Upper_XYZ_R"] as Rot3Angles;
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Front_Upper_XYZ_R = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(Rx_RJoint_Front_Upper_XYZ_R); 
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Front_Upper_XYZ_R = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(Ry_RJoint_Front_Upper_XYZ_R);     
        }
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Front_Upper_XYZ_R = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(Rz_RJoint_Front_Upper_XYZ_R);     
        }
        
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R);
    }


    linkTransformations["RJoint_Front_Upper_XYZ_R"] = A_RJoint_Front_Upper_XYZ_R;


    const M_Link_Front_Upper_R = new Matrix4(1.5099581673894136e-07,-3.2584134146418364e-07,1.0,0.40077972412109375,3.178649876645068e-08,1.0,3.2584134146418364e-07,7.995484452294477e-07,-1.0,3.1786445475745495e-08,1.5099581673894136e-07,0.06649642437696457,0.0,0.0,0.0,1.0);
    var A_Link_Front_Upper_R =  new Matrix4();
    A_Link_Front_Upper_R.copy(A_RJoint_Front_Upper_XYZ_R);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Front_Upper_R",A_Link_Front_Upper_R); 
    A_Link_Front_Upper_R = A_Link_Front_Upper_R.multiply(M_Link_Front_Upper_R);
    //console.log(`M_Link_Front_Upper_R: ${M_Link_Front_Upper_R.elements[0+3*4]} ${M_Link_Front_Upper_R.elements[1+3*4]} ${M_Link_Front_Upper_R.elements[2+3*4]}`);
    //printMatrix("M_Link_Front_Upper_R", M_Link_Front_Upper_R);   
    //console.log(`A_Link_Front_Upper_R: ${A_Link_Front_Upper_R.elements[0+3*4]} ${A_Link_Front_Upper_R.elements[1+3*4]} ${A_Link_Front_Upper_R.elements[2+3*4]}`);
    //printMatrix("A_Link_Front_Upper_R", A_Link_Front_Upper_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Front_Upper_R"] = A_Link_Front_Upper_R;


    const M_RJoint_Front_Lower_Z_R = new Matrix4(1.1924873533075697e-08,-3.2584136988589307e-07,-1.0,0.012298583984375,-3.6955276527805836e-07,1.0,-3.2584136988589307e-07,0.009898466989398003,1.0,3.6955276527805836e-07,1.1924752740810618e-08,0.42760491371154785,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Lower_Z_R =  new Matrix4();
    A_RJoint_Front_Lower_Z_R.copy(A_Link_Front_Upper_R);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Front_Upper_R", A_Link_Front_Upper_R);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Front_Lower_Z_R",A_RJoint_Front_Lower_Z_R); 
    A_RJoint_Front_Lower_Z_R = A_RJoint_Front_Lower_Z_R.multiply(M_RJoint_Front_Lower_Z_R);
    //console.log(`M_RJoint_Front_Lower_Z_R: ${M_RJoint_Front_Lower_Z_R.elements[0+3*4]} ${M_RJoint_Front_Lower_Z_R.elements[1+3*4]} ${M_RJoint_Front_Lower_Z_R.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Front_Lower_Z_R", M_RJoint_Front_Lower_Z_R);   
    //console.log(`A_RJoint_Front_Lower_Z_R: ${A_RJoint_Front_Lower_Z_R.elements[0+3*4]} ${A_RJoint_Front_Lower_Z_R.elements[1+3*4]} ${A_RJoint_Front_Lower_Z_R.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Front_Lower_Z_R", A_RJoint_Front_Lower_Z_R);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Lower_Z_R")) {
        const z = q["RJoint_Front_Lower_Z_R"] as number;
        const Rz_RJoint_Front_Lower_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Lower_Z_R.multiply(Rz_RJoint_Front_Lower_Z_R);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Front_Lower_Z_R", A_RJoint_Front_Lower_Z_R);
    }


    linkTransformations["RJoint_Front_Lower_Z_R"] = A_RJoint_Front_Lower_Z_R;


    const M_Link_Front_Lower_R = new Matrix4(1.1924880638503055e-08,-3.2584136988589307e-07,1.0,0.29159075021743774,1.5099578831723193e-07,1.0,3.2584136988589307e-07,0.0002992642985191196,-1.0,1.5099578831723193e-07,1.1924929488316138e-08,0.011934682726860046,0.0,0.0,0.0,1.0);
    var A_Link_Front_Lower_R =  new Matrix4();
    A_Link_Front_Lower_R.copy(A_RJoint_Front_Lower_Z_R);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Front_Lower_Z_R", A_RJoint_Front_Lower_Z_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Front_Lower_R",A_Link_Front_Lower_R); 
    A_Link_Front_Lower_R = A_Link_Front_Lower_R.multiply(M_Link_Front_Lower_R);
    //console.log(`M_Link_Front_Lower_R: ${M_Link_Front_Lower_R.elements[0+3*4]} ${M_Link_Front_Lower_R.elements[1+3*4]} ${M_Link_Front_Lower_R.elements[2+3*4]}`);
    //printMatrix("M_Link_Front_Lower_R", M_Link_Front_Lower_R);   
    //console.log(`A_Link_Front_Lower_R: ${A_Link_Front_Lower_R.elements[0+3*4]} ${A_Link_Front_Lower_R.elements[1+3*4]} ${A_Link_Front_Lower_R.elements[2+3*4]}`);
    //printMatrix("A_Link_Front_Lower_R", A_Link_Front_Lower_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Front_Lower_R"] = A_Link_Front_Lower_R;


    const M_RJoint_Front_Ankle_Z_R = new Matrix4(1.1924880638503055e-08,-3.2584136988589307e-07,-1.0,0.011934718117117882,-3.2584136988589307e-07,1.0,-3.2584136988589307e-07,-0.006699659395962954,1.0,3.2584136988589307e-07,1.192477405709269e-08,0.2604898512363434,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Ankle_Z_R =  new Matrix4();
    A_RJoint_Front_Ankle_Z_R.copy(A_Link_Front_Lower_R);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Front_Lower_R", A_Link_Front_Lower_R);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Front_Ankle_Z_R",A_RJoint_Front_Ankle_Z_R); 
    A_RJoint_Front_Ankle_Z_R = A_RJoint_Front_Ankle_Z_R.multiply(M_RJoint_Front_Ankle_Z_R);
    //console.log(`M_RJoint_Front_Ankle_Z_R: ${M_RJoint_Front_Ankle_Z_R.elements[0+3*4]} ${M_RJoint_Front_Ankle_Z_R.elements[1+3*4]} ${M_RJoint_Front_Ankle_Z_R.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Front_Ankle_Z_R", M_RJoint_Front_Ankle_Z_R);   
    //console.log(`A_RJoint_Front_Ankle_Z_R: ${A_RJoint_Front_Ankle_Z_R.elements[0+3*4]} ${A_RJoint_Front_Ankle_Z_R.elements[1+3*4]} ${A_RJoint_Front_Ankle_Z_R.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Front_Ankle_Z_R", A_RJoint_Front_Ankle_Z_R);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Ankle_Z_R")) {
        const z = q["RJoint_Front_Ankle_Z_R"] as number;
        const Rz_RJoint_Front_Ankle_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Ankle_Z_R.multiply(Rz_RJoint_Front_Ankle_Z_R);
        //console.log("After applying Z rotation");  
        //printMatrix("A_RJoint_Front_Ankle_Z_R", A_RJoint_Front_Ankle_Z_R);
    }


    linkTransformations["RJoint_Front_Ankle_Z_R"] = A_RJoint_Front_Ankle_Z_R;


    const M_Link_Front_Foot_R = new Matrix4(8.742277657347586e-08,-5.642599489874556e-07,1.0,0.19378721714019775,-3.179046643708716e-07,1.0,5.642599489874556e-07,-0.02257973700761795,-1.0,-3.179047212142905e-07,8.742259183236456e-08,0.007086207624524832,0.0,0.0,0.0,1.0);
    var A_Link_Front_Foot_R =  new Matrix4();
    A_Link_Front_Foot_R.copy(A_RJoint_Front_Ankle_Z_R);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Front_Ankle_Z_R", A_RJoint_Front_Ankle_Z_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Link_Front_Foot_R",A_Link_Front_Foot_R); 
    A_Link_Front_Foot_R = A_Link_Front_Foot_R.multiply(M_Link_Front_Foot_R);
    //console.log(`M_Link_Front_Foot_R: ${M_Link_Front_Foot_R.elements[0+3*4]} ${M_Link_Front_Foot_R.elements[1+3*4]} ${M_Link_Front_Foot_R.elements[2+3*4]}`);
    //printMatrix("M_Link_Front_Foot_R", M_Link_Front_Foot_R);   
    //console.log(`A_Link_Front_Foot_R: ${A_Link_Front_Foot_R.elements[0+3*4]} ${A_Link_Front_Foot_R.elements[1+3*4]} ${A_Link_Front_Foot_R.elements[2+3*4]}`);
    //printMatrix("A_Link_Front_Foot_R", A_Link_Front_Foot_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Link_Front_Foot_R"] = A_Link_Front_Foot_R;


    const M_Effector_Front_R = new Matrix4(-1.549839936387798e-07,-1.0,-5.523351092051598e-07,-8.732385481380334e-08,1.0,-1.5498373784339492e-07,-4.450507447018026e-07,0.007147098891437054,4.450506310149649e-07,-5.523351660485787e-07,1.0,0.0660289004445076,0.0,0.0,0.0,1.0);
    var A_Effector_Front_R =  new Matrix4();
    A_Effector_Front_R.copy(A_Link_Front_Foot_R);
    //console.log("Parent matrix");
    //printMatrix("A_Link_Front_Foot_R", A_Link_Front_Foot_R);
    //console.log("Incoming matrix");
    //printMatrix("A_Effector_Front_R",A_Effector_Front_R); 
    A_Effector_Front_R = A_Effector_Front_R.multiply(M_Effector_Front_R);
    //console.log(`M_Effector_Front_R: ${M_Effector_Front_R.elements[0+3*4]} ${M_Effector_Front_R.elements[1+3*4]} ${M_Effector_Front_R.elements[2+3*4]}`);
    //printMatrix("M_Effector_Front_R", M_Effector_Front_R);   
    //console.log(`A_Effector_Front_R: ${A_Effector_Front_R.elements[0+3*4]} ${A_Effector_Front_R.elements[1+3*4]} ${A_Effector_Front_R.elements[2+3*4]}`);
    //printMatrix("A_Effector_Front_R", A_Effector_Front_R);
    //console.log("Outgoing matrix");   


    linkTransformations["Effector_Front_R"] = A_Effector_Front_R;


    const M_RJoint_Head_XYZ_C = new Matrix4(-4.3711395392165286e-08,1.0,-7.549790836947068e-08,0.0,-1.0,-4.3711395392165286e-08,4.371138828673793e-08,-1.0458284616470337,4.371138473402425e-08,7.549790836947068e-08,1.0,0.39641857147216797,0.0,0.0,0.0,1.0);
    var A_RJoint_Head_XYZ_C =  new Matrix4();
    A_RJoint_Head_XYZ_C.copy(A_Link_UpperBody_C);
    //console.log("Parent matrix");
    //printMatrix("A_Link_UpperBody_C", A_Link_UpperBody_C);
    //console.log("Incoming matrix");
    //printMatrix("A_RJoint_Head_XYZ_C",A_RJoint_Head_XYZ_C); 
    A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(M_RJoint_Head_XYZ_C);
    //console.log(`M_RJoint_Head_XYZ_C: ${M_RJoint_Head_XYZ_C.elements[0+3*4]} ${M_RJoint_Head_XYZ_C.elements[1+3*4]} ${M_RJoint_Head_XYZ_C.elements[2+3*4]}`);
    //printMatrix("M_RJoint_Head_XYZ_C", M_RJoint_Head_XYZ_C);   
    //console.log(`A_RJoint_Head_XYZ_C: ${A_RJoint_Head_XYZ_C.elements[0+3*4]} ${A_RJoint_Head_XYZ_C.elements[1+3*4]} ${A_RJoint_Head_XYZ_C.elements[2+3*4]}`);
    //printMatrix("A_RJoint_Head_XYZ_C", A_RJoint_Head_XYZ_C);
    //console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Head_XYZ_C")) {
        const angles = q["RJoint_Head_XYZ_C"] as Rot3Angles;
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Head_XYZ_C = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(Rx_RJoint_Head_XYZ_C); 
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Head_XYZ_C = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(Ry_RJoint_Head_XYZ_C);     
        }
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Head_XYZ_C = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(Rz_RJoint_Head_XYZ_C);     
        }
        
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Head_XYZ_C", A_RJoint_Head_XYZ_C);
    }


    linkTransformations["RJoint_Head_XYZ_C"] = A_RJoint_Head_XYZ_C;


    const M_Head_C = new Matrix4(3.1391647326017846e-07,-1.0,-4.371138473402425e-08,-1.0458284616470337,1.0,3.1391647326017846e-07,-7.549791547489804e-08,-4.5714639185234773e-08,7.54979225803254e-08,-4.37113598650285e-08,1.0,-0.39641866087913513,0.0,0.0,0.0,1.0);
    var A_Head_C =  new Matrix4();
    A_Head_C.copy(A_RJoint_Head_XYZ_C);
    //console.log("Parent matrix");
    //printMatrix("A_RJoint_Head_XYZ_C", A_RJoint_Head_XYZ_C);
    //console.log("Incoming matrix");
    //printMatrix("A_Head_C",A_Head_C); 
    A_Head_C = A_Head_C.multiply(M_Head_C);
    //console.log(`M_Head_C: ${M_Head_C.elements[0+3*4]} ${M_Head_C.elements[1+3*4]} ${M_Head_C.elements[2+3*4]}`);
    //printMatrix("M_Head_C", M_Head_C);   
    //console.log(`A_Head_C: ${A_Head_C.elements[0+3*4]} ${A_Head_C.elements[1+3*4]} ${A_Head_C.elements[2+3*4]}`);
    //printMatrix("A_Head_C", A_Head_C);
    //console.log("Outgoing matrix");   


    linkTransformations["Head_C"] = A_Head_C;


    const M_Effector_Head_C = new Matrix4(1.0,-3.1264812787312746e-14,-2.5642845052215035e-14,-7.005405109339335e-07,-4.1622179200407824e-14,1.1920927533992653e-07,-1.0,-2.0093982219696045,2.5578595909099902e-14,1.0,1.1920927533992653e-07,-0.05276213586330414,0.0,0.0,0.0,1.0);
    var A_Effector_Head_C =  new Matrix4();
    A_Effector_Head_C.copy(A_Head_C);
    //console.log("Parent matrix");
    //printMatrix("A_Head_C", A_Head_C);
    //console.log("Incoming matrix");
    //printMatrix("A_Effector_Head_C",A_Effector_Head_C); 
    A_Effector_Head_C = A_Effector_Head_C.multiply(M_Effector_Head_C);
    //console.log(`M_Effector_Head_C: ${M_Effector_Head_C.elements[0+3*4]} ${M_Effector_Head_C.elements[1+3*4]} ${M_Effector_Head_C.elements[2+3*4]}`);
    //printMatrix("M_Effector_Head_C", M_Effector_Head_C);   
    //console.log(`A_Effector_Head_C: ${A_Effector_Head_C.elements[0+3*4]} ${A_Effector_Head_C.elements[1+3*4]} ${A_Effector_Head_C.elements[2+3*4]}`);
    //printMatrix("A_Effector_Head_C", A_Effector_Head_C);
    //console.log("Outgoing matrix");   


    linkTransformations["Effector_Head_C"] = A_Effector_Head_C;


    return linkTransformations;
}

export {calcForwardKinematics};

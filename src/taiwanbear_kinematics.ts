
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
    
    // q = {
    // //   "RJoint_Back_Upper_XYZ_L": {"x": 0.0/180.0*Math.PI, "y": 0.0/180.0*Math.PI, "z": 0.0/180.0*Math.PI },
    // //   "RJoint_Back_Lower_Z_L": 20.0/180.0*Math.PI,
    // //   "RJoint_Back_Ankle_Z_L": 30.0/180.0*Math.PI,
    // }

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


    const M_RJoint_Back_Upper_XYZ_L = new Matrix4(1.0,-1.0587911840678754e-22,0.0,0.03032732382416725,0.0,-6.137454278132282e-08,1.0,0.45000001788139343,0.0,-1.0,-6.137454278132282e-08,-0.23499634861946106,0.0,0.0,0.0,1.0);
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
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Back_Upper_XYZ_L = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(Rz_RJoint_Back_Upper_XYZ_L);     
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Back_Upper_XYZ_L = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(Ry_RJoint_Back_Upper_XYZ_L);     
        }
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Back_Upper_XYZ_L = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(Rx_RJoint_Back_Upper_XYZ_L); 
        }
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L);
    }


    linkTransformations["RJoint_Back_Upper_XYZ_L"] = A_RJoint_Back_Upper_XYZ_L;


    const M_Link_Back_Upper_L = new Matrix4(7.549790126404332e-08,-7.411538288475128e-22,-1.0,0.2863241136074066,4.3711398944878965e-08,1.0,3.3001185012921383e-15,0.0006768949097022414,1.0,-4.3711398944878965e-08,7.549790126404332e-08,0.6893821954727173,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Back_Lower_Z_L = new Matrix4(4.331257628109597e-07,9.47181888477644e-10,1.0,-0.5163219571113586,-1.5632414863985845e-14,1.0,-9.47181888477644e-10,-0.0004299516440369189,-1.0,-1.5222164620378592e-14,4.331257628109597e-07,-0.40017712116241455,0.0,0.0,0.0,1.0);
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


    const M_Link_Back_Lower_L = new Matrix4(1.9470718370939721e-07,1.0331601174584648e-07,-1.0,0.3016185462474823,-7.105432439798685e-15,1.0,1.0331601174584648e-07,0.00043000810546800494,1.0,-1.6327125876335405e-14,1.9470718370939721e-07,0.5222921967506409,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Back_Ankle_Z_L = new Matrix4(3.1391647326017846e-07,-4.0853258553852356e-08,1.0,-0.5222921371459961,1.0331601174584648e-07,1.0,4.085322302671557e-08,-0.0004297221021261066,-1.0,1.0331599753499177e-07,3.1391647326017846e-07,-0.332167387008667,0.0,0.0,0.0,1.0);
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


    const M_Link_Back_Foot_L = new Matrix4(5.52335052361741e-07,-2.4143361260415587e-14,-1.0,-0.3321671187877655,-1.4210854715202004e-14,1.0,-2.414337311887685e-14,-0.025352489203214645,1.0,1.4210842009707795e-14,5.52335052361741e-07,0.5163219571113586,0.0,0.0,0.0,1.0);
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


    const M_Effector_Back_L = new Matrix4(1.0,7.105427357601002e-14,1.1920926112907182e-07,-0.5166976451873779,7.10542481650216e-15,-1.0,5.32473393377586e-07,-0.10622052103281021,1.1920928955078125e-07,-5.32473393377586e-07,-1.0,-0.5732117891311646,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Back_Upper_XYZ_R = new Matrix4(1.0,-1.3200473158135606e-14,0.0,0.03032725676894188,5.823351512373315e-22,-9.973857686418341e-07,-1.0,-0.44999995827674866,1.3200472311102659e-14,1.0,-9.973857686418341e-07,-0.23499631881713867,0.0,0.0,0.0,1.0);
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
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Back_Upper_XYZ_R = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(Rz_RJoint_Back_Upper_XYZ_R);     
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Back_Upper_XYZ_R = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(Ry_RJoint_Back_Upper_XYZ_R);     
        }
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Back_Upper_XYZ_R = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(Rx_RJoint_Back_Upper_XYZ_R); 
        }
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R);
    }


    linkTransformations["RJoint_Back_Upper_XYZ_R"] = A_RJoint_Back_Upper_XYZ_R;


    const M_Link_Back_Upper_R = new Matrix4(7.549790126404332e-08,1.509957598955225e-07,1.0,0.28632375597953796,-3.2584136988589307e-07,1.0,-1.5099573147381307e-07,-0.0006767123122699559,-1.0,-3.2584136988589307e-07,7.549795100203482e-08,0.6893821954727173,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Back_Lower_Z_R = new Matrix4(1.1924880638503055e-08,-5.642599489874556e-07,-1.0,0.5163218975067139,1.5099578831723193e-07,1.0,-5.642599489874556e-07,0.0004300202417653054,1.0,-1.5099578831723193e-07,1.1924965903631346e-08,0.40017712116241455,0.0,0.0,0.0,1.0);
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


    const M_Link_Back_Lower_R = new Matrix4(1.1924880638503055e-08,1.5099578831723193e-07,1.0,0.30161821842193604,-6.198961841619166e-07,1.0,-1.5099578831723193e-07,-0.0004296250990591943,-1.0,-6.198961841619166e-07,1.1924973897237123e-08,0.5222923159599304,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Back_Ankle_Z_R = new Matrix4(9.934765898833575e-08,-3.814776050603541e-07,-1.0,0.5222920775413513,1.5099578831723193e-07,1.0,-3.814775766386447e-07,0.0004297832492738962,1.0,-1.509957598955225e-07,9.934771583175461e-08,0.33216729760169983,0.0,0.0,0.0,1.0);
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


    const M_Link_Back_Foot_R = new Matrix4(1.1924882414859894e-08,1.5099580252808664e-07,1.0,-0.332167387008667,-4.3711384023481514e-07,1.0,-1.5099578831723193e-07,0.025352755561470985,-1.0,-4.371138118131057e-07,1.1924947251884532e-08,0.5163217186927795,0.0,0.0,0.0,1.0);
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


    const M_Effector_Back_R = new Matrix4(-1.0,4.371137833913963e-07,1.7484583736404602e-07,0.5166975259780884,-4.371138118131057e-07,-1.0,-4.7683712978141557e-07,0.10622056573629379,1.7484563841208e-07,-4.768372150465439e-07,1.0,0.5732117295265198,0.0,0.0,0.0,1.0);
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
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Torso_XYZ_C = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(Rz_RJoint_Torso_XYZ_C);     
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Torso_XYZ_C = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(Ry_RJoint_Torso_XYZ_C);     
        }
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Torso_XYZ_C = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(Rx_RJoint_Torso_XYZ_C); 
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


    const M_RJoint_Front_Upper_XYZ_L = new Matrix4(1.5099581673894136e-07,2.0384551512339162e-21,1.0,0.4500000476837158,-4.371141670844736e-08,1.0,3.3001180777756647e-15,-0.6499999761581421,-1.0,-4.371141670844736e-08,1.5099581673894136e-07,0.3616074323654175,0.0,0.0,0.0,1.0);
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
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Front_Upper_XYZ_L = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(Rz_RJoint_Front_Upper_XYZ_L);     
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Front_Upper_XYZ_L = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(Ry_RJoint_Front_Upper_XYZ_L);     
        }
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Front_Upper_XYZ_L = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(Rx_RJoint_Front_Upper_XYZ_L); 
        }
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L);
    }


    linkTransformations["RJoint_Front_Upper_XYZ_L"] = A_RJoint_Front_Upper_XYZ_L;


    const M_Link_Front_Upper_L = new Matrix4(1.5099580252808664e-07,-4.371141670844736e-08,-1.0,0.40077993273735046,4.3711395392165286e-08,1.0,-4.371141315573368e-08,0.000677127274684608,1.0,-4.371139183945161e-08,1.5099581673894136e-07,0.5705195069313049,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Front_Lower_Z_L = new Matrix4(1.5099580252808664e-07,3.197442310920451e-14,1.0,-0.5163218379020691,-4.371142026116104e-08,1.0,-2.1316282072803006e-14,-0.00037886915379203856,-1.0,-4.371142026116104e-08,1.5099581673894136e-07,-0.42760491371154785,0.0,0.0,0.0,1.0);
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


    const M_Link_Front_Lower_L = new Matrix4(1.5099580252808664e-07,-4.371141670844736e-08,-1.0,0.23914949595928192,2.842170943040401e-14,1.0,-4.371142026116104e-08,0.00032782857306301594,1.0,-2.48689974456694e-14,1.5099581673894136e-07,0.5163219571113586,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Front_Ankle_Z_L = new Matrix4(1.5099580252808664e-07,4.0133926404450904e-07,1.0,-0.5163222551345825,-4.371142026116104e-08,1.0,-4.0133926404450904e-07,-0.00951077789068222,-1.0,-4.371135631231482e-08,1.5099581673894136e-07,-0.3129314184188843,0.0,0.0,0.0,1.0);
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


    const M_Link_Front_Foot_L = new Matrix4(1.5099581673894136e-07,-4.371141670844736e-08,-1.0,-0.2458752542734146,5.684341886080802e-14,1.0,-4.371142026116104e-08,0.009510694071650505,1.0,-5.6843412084544437e-14,1.5099580252808664e-07,0.516322135925293,0.0,0.0,0.0,1.0);
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


    const M_Effector_Front_L = new Matrix4(1.0,-4.013392072010902e-07,7.549823521912913e-08,-0.5092357993125916,-4.0133926404450904e-07,-1.0,7.589671326968528e-07,-0.0597185455262661,7.54979225803254e-08,-7.589671895402716e-07,-1.0,-0.4343193769454956,0.0,0.0,0.0,1.0);
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
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Front_Upper_XYZ_R = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(Rz_RJoint_Front_Upper_XYZ_R);     
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Front_Upper_XYZ_R = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(Ry_RJoint_Front_Upper_XYZ_R);     
        }
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Front_Upper_XYZ_R = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(Rx_RJoint_Front_Upper_XYZ_R); 
        }
        //console.log("After applying XYZ rotation");  
        //printMatrix("A_RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R);
    }


    linkTransformations["RJoint_Front_Upper_XYZ_R"] = A_RJoint_Front_Upper_XYZ_R;


    const M_Link_Front_Upper_R = new Matrix4(1.5099581673894136e-07,-3.2584134146418364e-07,1.0,0.4007796347141266,3.178649876645068e-08,1.0,3.2584134146418364e-07,-0.0006771596963517368,-1.0,3.1786445475745495e-08,1.5099581673894136e-07,0.5705196261405945,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Front_Lower_Z_R = new Matrix4(8.742279078433057e-08,-2.066320661242571e-07,-1.0,0.5163217782974243,-3.2584136988589307e-07,1.0,-2.0663209454596654e-07,0.0003785327135119587,1.0,3.2584136988589307e-07,8.742272683548435e-08,0.42760491371154785,0.0,0.0,0.0,1.0);
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


    const M_Link_Front_Lower_R = new Matrix4(1.1924884191216734e-08,-3.2584136988589307e-07,1.0,0.23914937674999237,1.5099580252808664e-07,1.0,3.2584136988589307e-07,-0.0003275431226938963,-1.0,1.5099580252808664e-07,1.192492682378088e-08,0.516322135925293,0.0,0.0,0.0,1.0);
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


    const M_RJoint_Front_Ankle_Z_R = new Matrix4(8.742277657347586e-08,-2.622683723529917e-07,-1.0,0.5163221955299377,-3.2584136988589307e-07,1.0,-2.6226840077470115e-07,0.009510775096714497,1.0,3.2584136988589307e-07,8.742269841377492e-08,0.3129313588142395,0.0,0.0,0.0,1.0);
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


    const M_Link_Front_Foot_R = new Matrix4(8.742276236262114e-08,-3.2584136988589307e-07,1.0,-0.24587546288967133,-3.1790460752745275e-07,1.0,3.2584136988589307e-07,-0.009510650299489498,-1.0,-3.179046359491622e-07,8.74226699920655e-08,0.5163220167160034,0.0,0.0,0.0,1.0);
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


    const M_Effector_Front_R = new Matrix4(-1.0,3.1790460752745275e-07,1.291335338356528e-13,0.5092357993125916,-3.1790460752745275e-07,-1.0,-9.5367431640625e-07,0.05971841514110565,-1.651355376226793e-13,-9.5367431640625e-07,1.0,0.43431922793388367,0.0,0.0,0.0,1.0);
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
        
        if (angles.hasOwnProperty("z")) { 
            const z = angles["z"];
            const Rz_RJoint_Head_XYZ_C = new Matrix4( Math.cos(z), - Math.sin(z), 0, 0,   Math.sin(z), Math.cos(z), 0, 0,   0, 0, 1, 0,   0, 0, 0, 1);
            A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(Rz_RJoint_Head_XYZ_C);     
        }
                
        if (angles.hasOwnProperty("y")) { 
            const y = angles["y"];
            const Ry_RJoint_Head_XYZ_C = new Matrix4(Math.cos(y), 0, Math.sin(y), 0,   0, 1, 0, 0,   - Math.sin(y), 0, Math.cos(y),   0, 0, 0, 0, 1);
            A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(Ry_RJoint_Head_XYZ_C);     
        }
        
        if (angles.hasOwnProperty("x")) {         
            const x = angles["x"];
            const Rx_RJoint_Head_XYZ_C = new Matrix4(1, 0, 0, 0,   0, Math.cos(x), - Math.sin(x), 0,   0, Math.sin(x), Math.cos(x), 0,   0, 0, 0, 1);
            A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(Rx_RJoint_Head_XYZ_C); 
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

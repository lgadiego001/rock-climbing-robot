
import {Matrix4} from 'three';


type Rot3Angles = {x:number, y:number, z:number};

interface JointAngles {
  [key: string]: number | Rot3Angles;
}

type Effectors = { [id:string] : Matrix4 };

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
): Effectors {
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

    var effectors: Effectors = {};


    const M_TaiwanBear = new Matrix4(7.549790126404332e-08,1.0,-4.371138828673793e-08,0.0,0.0,-4.371138828673793e-08,-1.0,0.0,-1.0,7.549790126404332e-08,-3.3001180777756647e-15,0.0,0.0,0.0,0.0,1.0);
    var A_TaiwanBear =  new Matrix4();
    A_TaiwanBear.copy(A);
    console.log("Parent matrix");
    printMatrix("A", A);
    console.log("Incoming matrix");
    printMatrix("A_TaiwanBear",A_TaiwanBear); 
    A_TaiwanBear = A_TaiwanBear.multiply(M_TaiwanBear);
    //console.log(`M_TaiwanBear: ${M_TaiwanBear.elements[0+3*4]} ${M_TaiwanBear.elements[1+3*4]} ${M_TaiwanBear.elements[2+3*4]}`);
    printMatrix("M_TaiwanBear", M_TaiwanBear);   
    //console.log(`A_TaiwanBear: ${A_TaiwanBear.elements[0+3*4]} ${A_TaiwanBear.elements[1+3*4]} ${A_TaiwanBear.elements[2+3*4]}`);
    printMatrix("A_TaiwanBear", A_TaiwanBear);
    console.log("Outgoing matrix");   


    const M_RJoint_Back_Upper_XYZ_L = new Matrix4(1.0,-1.0587911840678754e-22,0.0,0.03032732382416725,0.0,-6.137454278132282e-08,1.0,0.45000001788139343,0.0,-1.0,-6.137454278132282e-08,-0.23499634861946106,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Upper_XYZ_L =  new Matrix4();
    A_RJoint_Back_Upper_XYZ_L.copy(A_TaiwanBear);
    console.log("Parent matrix");
    printMatrix("A_TaiwanBear", A_TaiwanBear);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Back_Upper_XYZ_L",A_RJoint_Back_Upper_XYZ_L); 
    A_RJoint_Back_Upper_XYZ_L = A_RJoint_Back_Upper_XYZ_L.multiply(M_RJoint_Back_Upper_XYZ_L);
    //console.log(`M_RJoint_Back_Upper_XYZ_L: ${M_RJoint_Back_Upper_XYZ_L.elements[0+3*4]} ${M_RJoint_Back_Upper_XYZ_L.elements[1+3*4]} ${M_RJoint_Back_Upper_XYZ_L.elements[2+3*4]}`);
    printMatrix("M_RJoint_Back_Upper_XYZ_L", M_RJoint_Back_Upper_XYZ_L);   
    //console.log(`A_RJoint_Back_Upper_XYZ_L: ${A_RJoint_Back_Upper_XYZ_L.elements[0+3*4]} ${A_RJoint_Back_Upper_XYZ_L.elements[1+3*4]} ${A_RJoint_Back_Upper_XYZ_L.elements[2+3*4]}`);
    printMatrix("A_RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L);
    console.log("Outgoing matrix");   


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
        console.log("After applying XYZ rotation");  
        printMatrix("A_RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L);
    }


    const M_RJoint_Back_Lower_Z_L = new Matrix4(1.0,1.529367452991759e-14,-3.5762786865234375e-07,0.6865012049674988,-1.0587911840678754e-22,1.0,4.276421705640132e-08,0.00024691614089533687,3.5762786865234375e-07,-4.276421705640132e-08,1.0,0.17306022346019745,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Lower_Z_L =  new Matrix4();
    A_RJoint_Back_Lower_Z_L.copy(A_RJoint_Back_Upper_XYZ_L);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Back_Upper_XYZ_L", A_RJoint_Back_Upper_XYZ_L);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Back_Lower_Z_L",A_RJoint_Back_Lower_Z_L); 
    A_RJoint_Back_Lower_Z_L = A_RJoint_Back_Lower_Z_L.multiply(M_RJoint_Back_Lower_Z_L);
    //console.log(`M_RJoint_Back_Lower_Z_L: ${M_RJoint_Back_Lower_Z_L.elements[0+3*4]} ${M_RJoint_Back_Lower_Z_L.elements[1+3*4]} ${M_RJoint_Back_Lower_Z_L.elements[2+3*4]}`);
    printMatrix("M_RJoint_Back_Lower_Z_L", M_RJoint_Back_Lower_Z_L);   
    //console.log(`A_RJoint_Back_Lower_Z_L: ${A_RJoint_Back_Lower_Z_L.elements[0+3*4]} ${A_RJoint_Back_Lower_Z_L.elements[1+3*4]} ${A_RJoint_Back_Lower_Z_L.elements[2+3*4]}`);
    printMatrix("A_RJoint_Back_Lower_Z_L", A_RJoint_Back_Lower_Z_L);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Lower_Z_L")) {
        const z = q["RJoint_Back_Lower_Z_L"] as number;
        const Rz_RJoint_Back_Lower_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Lower_Z_L.multiply(Rz_RJoint_Back_Lower_Z_L);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Back_Lower_Z_L", A_RJoint_Back_Lower_Z_L);
    }


    const M_RJoint_Back_Ankle_Z_L = new Matrix4(1.0,9.157625558472299e-15,-1.1920928955078125e-07,0.6337858438491821,-4.2875384774661296e-15,1.0,4.0853251448425e-08,2.454721936828719e-07,1.1920928955078125e-07,-4.0853251448425e-08,1.0,-3.432890594012861e-08,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Ankle_Z_L =  new Matrix4();
    A_RJoint_Back_Ankle_Z_L.copy(A_RJoint_Back_Lower_Z_L);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Back_Lower_Z_L", A_RJoint_Back_Lower_Z_L);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Back_Ankle_Z_L",A_RJoint_Back_Ankle_Z_L); 
    A_RJoint_Back_Ankle_Z_L = A_RJoint_Back_Ankle_Z_L.multiply(M_RJoint_Back_Ankle_Z_L);
    //console.log(`M_RJoint_Back_Ankle_Z_L: ${M_RJoint_Back_Ankle_Z_L.elements[0+3*4]} ${M_RJoint_Back_Ankle_Z_L.elements[1+3*4]} ${M_RJoint_Back_Ankle_Z_L.elements[2+3*4]}`);
    printMatrix("M_RJoint_Back_Ankle_Z_L", M_RJoint_Back_Ankle_Z_L);   
    //console.log(`A_RJoint_Back_Ankle_Z_L: ${A_RJoint_Back_Ankle_Z_L.elements[0+3*4]} ${A_RJoint_Back_Ankle_Z_L.elements[1+3*4]} ${A_RJoint_Back_Ankle_Z_L.elements[2+3*4]}`);
    printMatrix("A_RJoint_Back_Ankle_Z_L", A_RJoint_Back_Ankle_Z_L);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Ankle_Z_L")) {
        const z = q["RJoint_Back_Ankle_Z_L"] as number;
        const Rz_RJoint_Back_Ankle_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Ankle_Z_L.multiply(Rz_RJoint_Back_Ankle_Z_L);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Back_Ankle_Z_L", A_RJoint_Back_Ankle_Z_L);
    }


    const M_Effector_Back_L = new Matrix4(4.331257628109597e-07,5.324734502210049e-07,1.0,0.24104438722133636,-7.105433286831633e-15,-1.0,5.324734502210049e-07,-0.13157302141189575,1.0,-2.372602982136929e-13,-4.3312579123266914e-07,-0.00037602026714012027,0.0,0.0,0.0,1.0);
    var A_Effector_Back_L =  new Matrix4();
    A_Effector_Back_L.copy(A_RJoint_Back_Ankle_Z_L);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Back_Ankle_Z_L", A_RJoint_Back_Ankle_Z_L);
    console.log("Incoming matrix");
    printMatrix("A_Effector_Back_L",A_Effector_Back_L); 
    A_Effector_Back_L = A_Effector_Back_L.multiply(M_Effector_Back_L);
    //console.log(`M_Effector_Back_L: ${M_Effector_Back_L.elements[0+3*4]} ${M_Effector_Back_L.elements[1+3*4]} ${M_Effector_Back_L.elements[2+3*4]}`);
    printMatrix("M_Effector_Back_L", M_Effector_Back_L);   
    //console.log(`A_Effector_Back_L: ${A_Effector_Back_L.elements[0+3*4]} ${A_Effector_Back_L.elements[1+3*4]} ${A_Effector_Back_L.elements[2+3*4]}`);
    printMatrix("A_Effector_Back_L", A_Effector_Back_L);
    console.log("Outgoing matrix");   


    effectors["Effector_Back_L"] = A_Effector_Back_L;


    const M_RJoint_Back_Upper_XYZ_R = new Matrix4(1.0,-1.3200473158135606e-14,0.0,0.03032725676894188,5.823351512373315e-22,-9.973857686418341e-07,-1.0,-0.44999995827674866,1.3200472311102659e-14,1.0,-9.973857686418341e-07,-0.23499631881713867,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Upper_XYZ_R =  new Matrix4();
    A_RJoint_Back_Upper_XYZ_R.copy(A_TaiwanBear);
    console.log("Parent matrix");
    printMatrix("A_TaiwanBear", A_TaiwanBear);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Back_Upper_XYZ_R",A_RJoint_Back_Upper_XYZ_R); 
    A_RJoint_Back_Upper_XYZ_R = A_RJoint_Back_Upper_XYZ_R.multiply(M_RJoint_Back_Upper_XYZ_R);
    //console.log(`M_RJoint_Back_Upper_XYZ_R: ${M_RJoint_Back_Upper_XYZ_R.elements[0+3*4]} ${M_RJoint_Back_Upper_XYZ_R.elements[1+3*4]} ${M_RJoint_Back_Upper_XYZ_R.elements[2+3*4]}`);
    printMatrix("M_RJoint_Back_Upper_XYZ_R", M_RJoint_Back_Upper_XYZ_R);   
    //console.log(`A_RJoint_Back_Upper_XYZ_R: ${A_RJoint_Back_Upper_XYZ_R.elements[0+3*4]} ${A_RJoint_Back_Upper_XYZ_R.elements[1+3*4]} ${A_RJoint_Back_Upper_XYZ_R.elements[2+3*4]}`);
    printMatrix("A_RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R);
    console.log("Outgoing matrix");   


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
        console.log("After applying XYZ rotation");  
        printMatrix("A_RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R);
    }


    const M_RJoint_Back_Lower_Z_R = new Matrix4(1.0,-6.814220140840405e-14,-6.357301884918343e-08,0.6865009069442749,5.298521427925078e-14,1.0,-2.384185791015625e-07,-0.0002469192259013653,6.357301884918343e-08,2.384185791015625e-07,1.0,0.17306026816368103,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Lower_Z_R =  new Matrix4();
    A_RJoint_Back_Lower_Z_R.copy(A_RJoint_Back_Upper_XYZ_R);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Back_Upper_XYZ_R", A_RJoint_Back_Upper_XYZ_R);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Back_Lower_Z_R",A_RJoint_Back_Lower_Z_R); 
    A_RJoint_Back_Lower_Z_R = A_RJoint_Back_Lower_Z_R.multiply(M_RJoint_Back_Lower_Z_R);
    //console.log(`M_RJoint_Back_Lower_Z_R: ${M_RJoint_Back_Lower_Z_R.elements[0+3*4]} ${M_RJoint_Back_Lower_Z_R.elements[1+3*4]} ${M_RJoint_Back_Lower_Z_R.elements[2+3*4]}`);
    printMatrix("M_RJoint_Back_Lower_Z_R", M_RJoint_Back_Lower_Z_R);   
    //console.log(`A_RJoint_Back_Lower_Z_R: ${A_RJoint_Back_Lower_Z_R.elements[0+3*4]} ${A_RJoint_Back_Lower_Z_R.elements[1+3*4]} ${A_RJoint_Back_Lower_Z_R.elements[2+3*4]}`);
    printMatrix("A_RJoint_Back_Lower_Z_R", A_RJoint_Back_Lower_Z_R);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Lower_Z_R")) {
        const z = q["RJoint_Back_Lower_Z_R"] as number;
        const Rz_RJoint_Back_Lower_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Lower_Z_R.multiply(Rz_RJoint_Back_Lower_Z_R);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Back_Lower_Z_R", A_RJoint_Back_Lower_Z_R);
    }


    const M_RJoint_Back_Ankle_Z_R = new Matrix4(1.0,3.417218222799556e-14,8.742277657347586e-08,0.6337854862213135,-5.501539978788883e-14,1.0,2.384185791015625e-07,-2.160634693382235e-07,-8.742277657347586e-08,-2.384185791015625e-07,1.0,1.9789453631346987e-07,0.0,0.0,0.0,1.0);
    var A_RJoint_Back_Ankle_Z_R =  new Matrix4();
    A_RJoint_Back_Ankle_Z_R.copy(A_RJoint_Back_Lower_Z_R);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Back_Lower_Z_R", A_RJoint_Back_Lower_Z_R);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Back_Ankle_Z_R",A_RJoint_Back_Ankle_Z_R); 
    A_RJoint_Back_Ankle_Z_R = A_RJoint_Back_Ankle_Z_R.multiply(M_RJoint_Back_Ankle_Z_R);
    //console.log(`M_RJoint_Back_Ankle_Z_R: ${M_RJoint_Back_Ankle_Z_R.elements[0+3*4]} ${M_RJoint_Back_Ankle_Z_R.elements[1+3*4]} ${M_RJoint_Back_Ankle_Z_R.elements[2+3*4]}`);
    printMatrix("M_RJoint_Back_Ankle_Z_R", M_RJoint_Back_Ankle_Z_R);   
    //console.log(`A_RJoint_Back_Ankle_Z_R: ${A_RJoint_Back_Ankle_Z_R.elements[0+3*4]} ${A_RJoint_Back_Ankle_Z_R.elements[1+3*4]} ${A_RJoint_Back_Ankle_Z_R.elements[2+3*4]}`);
    printMatrix("A_RJoint_Back_Ankle_Z_R", A_RJoint_Back_Ankle_Z_R);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Back_Ankle_Z_R")) {
        const z = q["RJoint_Back_Ankle_Z_R"] as number;
        const Rz_RJoint_Back_Ankle_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Back_Ankle_Z_R.multiply(Rz_RJoint_Back_Ankle_Z_R);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Back_Ankle_Z_R", A_RJoint_Back_Ankle_Z_R);
    }


    const M_Effector_Back_R = new Matrix4(1.6292068494294654e-07,-6.278330033637758e-07,1.0,0.24104434251785278,1.2218333295634789e-20,-1.0,-6.278330033637758e-07,0.13157300651073456,1.0,5.131309331433964e-14,-1.6292068494294654e-07,-0.000375779636669904,0.0,0.0,0.0,1.0);
    var A_Effector_Back_R =  new Matrix4();
    A_Effector_Back_R.copy(A_RJoint_Back_Ankle_Z_R);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Back_Ankle_Z_R", A_RJoint_Back_Ankle_Z_R);
    console.log("Incoming matrix");
    printMatrix("A_Effector_Back_R",A_Effector_Back_R); 
    A_Effector_Back_R = A_Effector_Back_R.multiply(M_Effector_Back_R);
    //console.log(`M_Effector_Back_R: ${M_Effector_Back_R.elements[0+3*4]} ${M_Effector_Back_R.elements[1+3*4]} ${M_Effector_Back_R.elements[2+3*4]}`);
    printMatrix("M_Effector_Back_R", M_Effector_Back_R);   
    //console.log(`A_Effector_Back_R: ${A_Effector_Back_R.elements[0+3*4]} ${A_Effector_Back_R.elements[1+3*4]} ${A_Effector_Back_R.elements[2+3*4]}`);
    printMatrix("A_Effector_Back_R", A_Effector_Back_R);
    console.log("Outgoing matrix");   


    effectors["Effector_Back_R"] = A_Effector_Back_R;


    const M_RJoint_Torso_XYZ_C = new Matrix4(2.3700034380218053e-14,7.549790126404332e-08,-1.0,0.007900208234786987,3.5762786865234375e-07,1.0,7.549790126404332e-08,5.321260942992012e-08,1.0,-3.5762786865234375e-07,-3.300117866017428e-15,0.8901089429855347,0.0,0.0,0.0,1.0);
    var A_RJoint_Torso_XYZ_C =  new Matrix4();
    A_RJoint_Torso_XYZ_C.copy(A_TaiwanBear);
    console.log("Parent matrix");
    printMatrix("A_TaiwanBear", A_TaiwanBear);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Torso_XYZ_C",A_RJoint_Torso_XYZ_C); 
    A_RJoint_Torso_XYZ_C = A_RJoint_Torso_XYZ_C.multiply(M_RJoint_Torso_XYZ_C);
    //console.log(`M_RJoint_Torso_XYZ_C: ${M_RJoint_Torso_XYZ_C.elements[0+3*4]} ${M_RJoint_Torso_XYZ_C.elements[1+3*4]} ${M_RJoint_Torso_XYZ_C.elements[2+3*4]}`);
    printMatrix("M_RJoint_Torso_XYZ_C", M_RJoint_Torso_XYZ_C);   
    //console.log(`A_RJoint_Torso_XYZ_C: ${A_RJoint_Torso_XYZ_C.elements[0+3*4]} ${A_RJoint_Torso_XYZ_C.elements[1+3*4]} ${A_RJoint_Torso_XYZ_C.elements[2+3*4]}`);
    printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    console.log("Outgoing matrix");   


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
        console.log("After applying XYZ rotation");  
        printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    }


    const M_RJoint_Front_Upper_XYZ_L = new Matrix4(2.3700034380218053e-14,-1.0,-4.3711395392165286e-08,0.6499999761581421,7.549790126404332e-08,-4.3711395392165286e-08,1.0,0.4500000774860382,-1.0,-2.7000152669751955e-14,7.549790126404332e-08,-0.02242732234299183,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Upper_XYZ_L =  new Matrix4();
    A_RJoint_Front_Upper_XYZ_L.copy(A_RJoint_Torso_XYZ_C);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Front_Upper_XYZ_L",A_RJoint_Front_Upper_XYZ_L); 
    A_RJoint_Front_Upper_XYZ_L = A_RJoint_Front_Upper_XYZ_L.multiply(M_RJoint_Front_Upper_XYZ_L);
    //console.log(`M_RJoint_Front_Upper_XYZ_L: ${M_RJoint_Front_Upper_XYZ_L.elements[0+3*4]} ${M_RJoint_Front_Upper_XYZ_L.elements[1+3*4]} ${M_RJoint_Front_Upper_XYZ_L.elements[2+3*4]}`);
    printMatrix("M_RJoint_Front_Upper_XYZ_L", M_RJoint_Front_Upper_XYZ_L);   
    //console.log(`A_RJoint_Front_Upper_XYZ_L: ${A_RJoint_Front_Upper_XYZ_L.elements[0+3*4]} ${A_RJoint_Front_Upper_XYZ_L.elements[1+3*4]} ${A_RJoint_Front_Upper_XYZ_L.elements[2+3*4]}`);
    printMatrix("A_RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L);
    console.log("Outgoing matrix");   


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
        console.log("After applying XYZ rotation");  
        printMatrix("A_RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L);
    }


    const M_RJoint_Front_Lower_Z_L = new Matrix4(1.0,1.6940658945086007e-21,0.0,0.8283848166465759,-2.491518845517339e-22,1.0,4.3711366970455856e-08,0.00029831973370164633,0.0,-4.3711366970455856e-08,1.0,0.05419759079813957,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Lower_Z_L =  new Matrix4();
    A_RJoint_Front_Lower_Z_L.copy(A_RJoint_Front_Upper_XYZ_L);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Front_Upper_XYZ_L", A_RJoint_Front_Upper_XYZ_L);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Front_Lower_Z_L",A_RJoint_Front_Lower_Z_L); 
    A_RJoint_Front_Lower_Z_L = A_RJoint_Front_Lower_Z_L.multiply(M_RJoint_Front_Lower_Z_L);
    //console.log(`M_RJoint_Front_Lower_Z_L: ${M_RJoint_Front_Lower_Z_L.elements[0+3*4]} ${M_RJoint_Front_Lower_Z_L.elements[1+3*4]} ${M_RJoint_Front_Lower_Z_L.elements[2+3*4]}`);
    printMatrix("M_RJoint_Front_Lower_Z_L", M_RJoint_Front_Lower_Z_L);   
    //console.log(`A_RJoint_Front_Lower_Z_L: ${A_RJoint_Front_Lower_Z_L.elements[0+3*4]} ${A_RJoint_Front_Lower_Z_L.elements[1+3*4]} ${A_RJoint_Front_Lower_Z_L.elements[2+3*4]}`);
    printMatrix("A_RJoint_Front_Lower_Z_L", A_RJoint_Front_Lower_Z_L);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Lower_Z_L")) {
        const z = q["RJoint_Front_Lower_Z_L"] as number;
        const Rz_RJoint_Front_Lower_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Lower_Z_L.multiply(Rz_RJoint_Front_Lower_Z_L);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Front_Lower_Z_L", A_RJoint_Front_Lower_Z_L);
    }


    const M_RJoint_Front_Ankle_Z_L = new Matrix4(1.0,-2.491518845517339e-22,0.0,0.5520808696746826,-2.491518845517339e-22,1.0,-4.013392356227996e-07,-0.00918277446180582,0.0,4.013392356227996e-07,1.0,-3.109511510501761e-07,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Ankle_Z_L =  new Matrix4();
    A_RJoint_Front_Ankle_Z_L.copy(A_RJoint_Front_Lower_Z_L);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Front_Lower_Z_L", A_RJoint_Front_Lower_Z_L);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Front_Ankle_Z_L",A_RJoint_Front_Ankle_Z_L); 
    A_RJoint_Front_Ankle_Z_L = A_RJoint_Front_Ankle_Z_L.multiply(M_RJoint_Front_Ankle_Z_L);
    //console.log(`M_RJoint_Front_Ankle_Z_L: ${M_RJoint_Front_Ankle_Z_L.elements[0+3*4]} ${M_RJoint_Front_Ankle_Z_L.elements[1+3*4]} ${M_RJoint_Front_Ankle_Z_L.elements[2+3*4]}`);
    printMatrix("M_RJoint_Front_Ankle_Z_L", M_RJoint_Front_Ankle_Z_L);   
    //console.log(`A_RJoint_Front_Ankle_Z_L: ${A_RJoint_Front_Ankle_Z_L.elements[0+3*4]} ${A_RJoint_Front_Ankle_Z_L.elements[1+3*4]} ${A_RJoint_Front_Ankle_Z_L.elements[2+3*4]}`);
    printMatrix("A_RJoint_Front_Ankle_Z_L", A_RJoint_Front_Ankle_Z_L);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Ankle_Z_L")) {
        const z = q["RJoint_Front_Ankle_Z_L"] as number;
        const Rz_RJoint_Front_Ankle_Z_L = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Ankle_Z_L.multiply(Rz_RJoint_Front_Ankle_Z_L);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Front_Ankle_Z_L", A_RJoint_Front_Ankle_Z_L);
    }


    const M_Effector_Front_L = new Matrix4(7.549790126404332e-08,8.026785280890181e-07,1.0,0.18844406306743622,-4.013392356227996e-07,-1.0,8.026785849324369e-07,-0.05020785331726074,1.0,-4.0133929246621847e-07,-7.549758151981223e-08,0.007086275611072779,0.0,0.0,0.0,1.0);
    var A_Effector_Front_L =  new Matrix4();
    A_Effector_Front_L.copy(A_RJoint_Front_Ankle_Z_L);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Front_Ankle_Z_L", A_RJoint_Front_Ankle_Z_L);
    console.log("Incoming matrix");
    printMatrix("A_Effector_Front_L",A_Effector_Front_L); 
    A_Effector_Front_L = A_Effector_Front_L.multiply(M_Effector_Front_L);
    //console.log(`M_Effector_Front_L: ${M_Effector_Front_L.elements[0+3*4]} ${M_Effector_Front_L.elements[1+3*4]} ${M_Effector_Front_L.elements[2+3*4]}`);
    printMatrix("M_Effector_Front_L", M_Effector_Front_L);   
    //console.log(`A_Effector_Front_L: ${A_Effector_Front_L.elements[0+3*4]} ${A_Effector_Front_L.elements[1+3*4]} ${A_Effector_Front_L.elements[2+3*4]}`);
    printMatrix("A_Effector_Front_L", A_Effector_Front_L);
    console.log("Outgoing matrix");   


    effectors["Effector_Front_L"] = A_Effector_Front_L;


    const M_RJoint_Front_Upper_XYZ_R = new Matrix4(3.6900505844287765e-14,1.0,7.549789415861596e-08,0.6499999761581421,7.549790126404332e-08,7.549789415861596e-08,-1.0,-0.45000001788139343,-1.0,4.260044126811434e-14,-7.549790126404332e-08,-0.022427255287766457,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Upper_XYZ_R =  new Matrix4();
    A_RJoint_Front_Upper_XYZ_R.copy(A_RJoint_Torso_XYZ_C);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Front_Upper_XYZ_R",A_RJoint_Front_Upper_XYZ_R); 
    A_RJoint_Front_Upper_XYZ_R = A_RJoint_Front_Upper_XYZ_R.multiply(M_RJoint_Front_Upper_XYZ_R);
    //console.log(`M_RJoint_Front_Upper_XYZ_R: ${M_RJoint_Front_Upper_XYZ_R.elements[0+3*4]} ${M_RJoint_Front_Upper_XYZ_R.elements[1+3*4]} ${M_RJoint_Front_Upper_XYZ_R.elements[2+3*4]}`);
    printMatrix("M_RJoint_Front_Upper_XYZ_R", M_RJoint_Front_Upper_XYZ_R);   
    //console.log(`A_RJoint_Front_Upper_XYZ_R: ${A_RJoint_Front_Upper_XYZ_R.elements[0+3*4]} ${A_RJoint_Front_Upper_XYZ_R.elements[1+3*4]} ${A_RJoint_Front_Upper_XYZ_R.elements[2+3*4]}`);
    printMatrix("A_RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R);
    console.log("Outgoing matrix");   


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
        console.log("After applying XYZ rotation");  
        printMatrix("A_RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R);
    }


    const M_RJoint_Front_Lower_Z_R = new Matrix4(1.0,2.2799733224976824e-14,-6.357301884918343e-08,0.8283846378326416,-3.795672204819599e-14,1.0,-2.3841856489070778e-07,-0.00029833876760676503,6.357301884918343e-08,2.3841856489070778e-07,1.0,0.054197974503040314,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Lower_Z_R =  new Matrix4();
    A_RJoint_Front_Lower_Z_R.copy(A_RJoint_Front_Upper_XYZ_R);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Front_Upper_XYZ_R", A_RJoint_Front_Upper_XYZ_R);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Front_Lower_Z_R",A_RJoint_Front_Lower_Z_R); 
    A_RJoint_Front_Lower_Z_R = A_RJoint_Front_Lower_Z_R.multiply(M_RJoint_Front_Lower_Z_R);
    //console.log(`M_RJoint_Front_Lower_Z_R: ${M_RJoint_Front_Lower_Z_R.elements[0+3*4]} ${M_RJoint_Front_Lower_Z_R.elements[1+3*4]} ${M_RJoint_Front_Lower_Z_R.elements[2+3*4]}`);
    printMatrix("M_RJoint_Front_Lower_Z_R", M_RJoint_Front_Lower_Z_R);   
    //console.log(`A_RJoint_Front_Lower_Z_R: ${A_RJoint_Front_Lower_Z_R.elements[0+3*4]} ${A_RJoint_Front_Lower_Z_R.elements[1+3*4]} ${A_RJoint_Front_Lower_Z_R.elements[2+3*4]}`);
    printMatrix("A_RJoint_Front_Lower_Z_R", A_RJoint_Front_Lower_Z_R);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Lower_Z_R")) {
        const z = q["RJoint_Front_Lower_Z_R"] as number;
        const Rz_RJoint_Front_Lower_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Lower_Z_R.multiply(Rz_RJoint_Front_Lower_Z_R);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Front_Lower_Z_R", A_RJoint_Front_Lower_Z_R);
    }


    const M_RJoint_Front_Ankle_Z_R = new Matrix4(1.0,1.2093716192330614e-14,7.549790126404332e-08,0.5520807504653931,1.63279915440075e-14,1.0,-4.132641322485142e-07,0.009183330461382866,-7.549790126404332e-08,4.132641322485142e-07,1.0,1.2296554885438127e-08,0.0,0.0,0.0,1.0);
    var A_RJoint_Front_Ankle_Z_R =  new Matrix4();
    A_RJoint_Front_Ankle_Z_R.copy(A_RJoint_Front_Lower_Z_R);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Front_Lower_Z_R", A_RJoint_Front_Lower_Z_R);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Front_Ankle_Z_R",A_RJoint_Front_Ankle_Z_R); 
    A_RJoint_Front_Ankle_Z_R = A_RJoint_Front_Ankle_Z_R.multiply(M_RJoint_Front_Ankle_Z_R);
    //console.log(`M_RJoint_Front_Ankle_Z_R: ${M_RJoint_Front_Ankle_Z_R.elements[0+3*4]} ${M_RJoint_Front_Ankle_Z_R.elements[1+3*4]} ${M_RJoint_Front_Ankle_Z_R.elements[2+3*4]}`);
    printMatrix("M_RJoint_Front_Ankle_Z_R", M_RJoint_Front_Ankle_Z_R);   
    //console.log(`A_RJoint_Front_Ankle_Z_R: ${A_RJoint_Front_Ankle_Z_R.elements[0+3*4]} ${A_RJoint_Front_Ankle_Z_R.elements[1+3*4]} ${A_RJoint_Front_Ankle_Z_R.elements[2+3*4]}`);
    printMatrix("A_RJoint_Front_Ankle_Z_R", A_RJoint_Front_Ankle_Z_R);
    console.log("Outgoing matrix");   


    if (q.hasOwnProperty("RJoint_Front_Ankle_Z_R")) {
        const z = q["RJoint_Front_Ankle_Z_R"] as number;
        const Rz_RJoint_Front_Ankle_Z_R = new Matrix4(Math.cos(z), - Math.sin(z), 0, 0, Math.sin(z), Math.cos(z), 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1);
        A_RJoint_Front_Ankle_Z_R.multiply(Rz_RJoint_Front_Ankle_Z_R);
        console.log("After applying Z rotation");  
        printMatrix("A_RJoint_Front_Ankle_Z_R", A_RJoint_Front_Ankle_Z_R);
    }


    const M_Effector_Front_R = new Matrix4(-8.742284052232208e-08,-6.278329465203569e-07,1.0,0.18844373524188995,-1.0231872608106513e-27,-1.0,-6.278330033637758e-07,0.050207749009132385,1.0,-5.488690488979035e-14,8.742283341689472e-08,0.0070862360298633575,0.0,0.0,0.0,1.0);
    var A_Effector_Front_R =  new Matrix4();
    A_Effector_Front_R.copy(A_RJoint_Front_Ankle_Z_R);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Front_Ankle_Z_R", A_RJoint_Front_Ankle_Z_R);
    console.log("Incoming matrix");
    printMatrix("A_Effector_Front_R",A_Effector_Front_R); 
    A_Effector_Front_R = A_Effector_Front_R.multiply(M_Effector_Front_R);
    //console.log(`M_Effector_Front_R: ${M_Effector_Front_R.elements[0+3*4]} ${M_Effector_Front_R.elements[1+3*4]} ${M_Effector_Front_R.elements[2+3*4]}`);
    printMatrix("M_Effector_Front_R", M_Effector_Front_R);   
    //console.log(`A_Effector_Front_R: ${A_Effector_Front_R.elements[0+3*4]} ${A_Effector_Front_R.elements[1+3*4]} ${A_Effector_Front_R.elements[2+3*4]}`);
    printMatrix("A_Effector_Front_R", A_Effector_Front_R);
    console.log("Outgoing matrix");   


    effectors["Effector_Front_R"] = A_Effector_Front_R;


    const M_RJoint_Head_XYZ_C = new Matrix4(1.0,1.0890779494492707e-29,4.371138473402425e-08,1.0458284616470337,6.3054852339690036e-30,1.0,7.549790126404332e-08,4.664957486966159e-08,-4.371138473402425e-08,-7.549790126404332e-08,1.0,0.012383832596242428,0.0,0.0,0.0,1.0);
    var A_RJoint_Head_XYZ_C =  new Matrix4();
    A_RJoint_Head_XYZ_C.copy(A_RJoint_Torso_XYZ_C);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Torso_XYZ_C", A_RJoint_Torso_XYZ_C);
    console.log("Incoming matrix");
    printMatrix("A_RJoint_Head_XYZ_C",A_RJoint_Head_XYZ_C); 
    A_RJoint_Head_XYZ_C = A_RJoint_Head_XYZ_C.multiply(M_RJoint_Head_XYZ_C);
    //console.log(`M_RJoint_Head_XYZ_C: ${M_RJoint_Head_XYZ_C.elements[0+3*4]} ${M_RJoint_Head_XYZ_C.elements[1+3*4]} ${M_RJoint_Head_XYZ_C.elements[2+3*4]}`);
    printMatrix("M_RJoint_Head_XYZ_C", M_RJoint_Head_XYZ_C);   
    //console.log(`A_RJoint_Head_XYZ_C: ${A_RJoint_Head_XYZ_C.elements[0+3*4]} ${A_RJoint_Head_XYZ_C.elements[1+3*4]} ${A_RJoint_Head_XYZ_C.elements[2+3*4]}`);
    printMatrix("A_RJoint_Head_XYZ_C", A_RJoint_Head_XYZ_C);
    console.log("Outgoing matrix");   


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
        console.log("After applying XYZ rotation");  
        printMatrix("A_RJoint_Head_XYZ_C", A_RJoint_Head_XYZ_C);
    }


    const M_Effector_Head_C = new Matrix4(-4.371136341774218e-08,-1.1920928955078125e-07,1.0,0.9635696411132812,1.0,-2.6037946958593593e-14,4.37113598650285e-08,-6.584216976079915e-07,1.9894737170612214e-14,1.0,1.1920928955078125e-07,-0.44918084144592285,0.0,0.0,0.0,1.0);
    var A_Effector_Head_C =  new Matrix4();
    A_Effector_Head_C.copy(A_RJoint_Head_XYZ_C);
    console.log("Parent matrix");
    printMatrix("A_RJoint_Head_XYZ_C", A_RJoint_Head_XYZ_C);
    console.log("Incoming matrix");
    printMatrix("A_Effector_Head_C",A_Effector_Head_C); 
    A_Effector_Head_C = A_Effector_Head_C.multiply(M_Effector_Head_C);
    //console.log(`M_Effector_Head_C: ${M_Effector_Head_C.elements[0+3*4]} ${M_Effector_Head_C.elements[1+3*4]} ${M_Effector_Head_C.elements[2+3*4]}`);
    printMatrix("M_Effector_Head_C", M_Effector_Head_C);   
    //console.log(`A_Effector_Head_C: ${A_Effector_Head_C.elements[0+3*4]} ${A_Effector_Head_C.elements[1+3*4]} ${A_Effector_Head_C.elements[2+3*4]}`);
    printMatrix("A_Effector_Head_C", A_Effector_Head_C);
    console.log("Outgoing matrix");   


    effectors["Effector_Head_C"] = A_Effector_Head_C;


    return effectors;
}

export {calcForwardKinematics};

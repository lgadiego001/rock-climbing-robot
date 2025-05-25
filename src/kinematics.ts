import { calcForwardKinematics } from "./taiwanbear_kinematics";
import { Matrix4, Object3D } from "three";

type Rot3Angles = {x:number, y:number, z:number};

interface JointAngles {
  [key: string]: number | Rot3Angles;
}

calcForwardKinematics( {}, new Matrix4() ); 

class Kinematics {
    root: Object3D;

    constructor(root: Object3D) {
        this.root = root;
    };

    forwardKinematics(q:JointAngles, A: Matrix4) {
        return calcForwardKinematics(q, A);
    }

    inverseKinematics(q:JointAngles, AIn: Matrix4) {
        // Implement inverse kinematics logic here
        // This is a placeholder implementation
        const AOut = new Matrix4();
        AOut.copy(AIn);
        return AOut;
    }

    getCurrentState() {
        const q: JointAngles = {};
        this.root.traverse((child) => {
            if (child instanceof Object3D) {
                const name = child.name;
                console.log(`child name: ${name}`);
                if (child.name.startsWith("RJoint_")) {
                    if (child.rotation && child.rotation.isEuler) {
                        if (child.name.includes("_XYZ_")) {
                            q[name] = { "x": child.rotation.x, "y": child.rotation.y, "z": child.rotation.z };
                            console.log(`child rotation xyz: ${child.rotation.x}, ${child.rotation.y}, ${child.rotation.z}`);
                        } else if (child.name.includes("_Z_")) {
                            q[name] = child.rotation.z;
                            console.log(`child rotation z: ${child.rotation.z}`);
                        }
                    }
                }
            }
        });
        console.log(`getCurrentState q: ${JSON.stringify(q)}`);
        return q;
    }
}

export { Kinematics };
export type { JointAngles };
export type { Rot3Angles };

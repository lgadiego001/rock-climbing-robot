import { calcForwardKinematics } from "./taiwanbear_kinematics";
import { Matrix4, Object3D } from "three";

type Rot3Angles = { x: number, y: number, z: number };

interface JointAngles {
    [key: string]: number | Rot3Angles;
}

interface NameChildMap {
    [key: string]: Object3D;
}

class Kinematics {
    root: Object3D;
    map: NameChildMap;
    currentQ: JointAngles;

    constructor(root: Object3D) {
        this.root = root;
        this.map = this.getNameChildMap(this.root);
        this.currentQ = this.getCurrentState();
    };

    forwardKinematics(q: JointAngles, A: Matrix4) {
        return calcForwardKinematics(q, A);
    }

    inverseKinematics(q: JointAngles, AIn: Matrix4) {
        // Implement inverse kinematics logic here
        // This is a placeholder implementation
        const AOut = new Matrix4();
        AOut.copy(AIn);
        return AOut;
    }

    getNameChildMap(obj: Object3D): NameChildMap {
        const map: NameChildMap = {};
        obj.traverse((child) => {
            if (child instanceof Object3D) {
                const name = child.name;
                console.log(`child name: ${name}`);
                map[name] = child;
            }
        });
        return map;
    }

    getCurrentState() {
        const q: JointAngles = {};
        for (let name in this.map) {
            console.log(`child name: ${name}`);
            if (name.startsWith("RJoint_")) {
                let child = this.map[name];
                if (child.rotation && child.rotation.isEuler) {
                    if (name.includes("_XYZ_")) {
                        q[name] = { "x": child.rotation.x, "y": child.rotation.y, "z": child.rotation.z };
                        console.log(`child rotation xyz: ${child.rotation.x}, ${child.rotation.y}, ${child.rotation.z}`);
                    } else if (name.includes("_Z_")) {
                        q[name] = child.rotation.z;
                        console.log(`child rotation z: ${child.rotation.z}`);
                    }
                }
            }
        }
        console.log(`getCurrentState q: ${JSON.stringify(q)}`);
        return q;
    }

    updateJointZ(child: Object3D, val: number) {
        child.rotation.z = val;
    }

    updateJointXYZ(child: Object3D, val: Rot3Angles) {
        child.rotation.x = val["x"];
        child.rotation.y = val["y"];
        child.rotation.z = val["z"];       
    }

    updateState(q : JointAngles) {
        for(let joint in q) {
            if (joint.includes("_Z_")) {
                this.updateJointZ(this.map[joint], q[joint]);
            } else if (joint.includes("_XYZ_")) {

            }
        }
    }
}

export { Kinematics };
export type { JointAngles };
export type { Rot3Angles };

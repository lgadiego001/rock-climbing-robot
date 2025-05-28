import { calcForwardKinematics } from "./taiwanbear_kinematics";
import { Matrix4, Vector3, Object3D } from "three";

type Rot3Angles = { x: number, y: number, z: number };

interface JointAngles {
    [key: string]: number | Rot3Angles;
}

interface NameChildMap {
    [key: string]: Object3D;
}

function sortObjectKeys<T extends Record<string, any>>(obj: T): T {
    return Object.keys(obj)
      .sort()
      .reduce((sortedObj, key) => {
        (sortedObj as any)[key] = obj[key];
        return sortedObj;
      }, {} as T);
}
    
function isDictionary(obj: unknown): obj is Record<string, unknown> {
    return typeof obj === 'object' && obj !== null && !Array.isArray(obj);
}

class Kinematics {
    root: Object3D;
    map: NameChildMap;
    homePositionConfig: JointAngles;
    qConfigIndex: {[key:string]: number};
    qConfigIndexReversed: string[];
    qConfigLength: number;

    constructor(root: Object3D) {
        this.root = root;
        this.map = sortObjectKeys(this.getNameChildMap(this.root));
        const [ind, indRev ] = this.calcQConfigIndex(this.map);
        this.qConfigIndex = ind; 
        this.qConfigIndexReversed = indRev;  
        this.qConfigLength = this.qConfigIndexReversed.length;
        this.homePositionConfig = this.getCurrentStateConfig();
    }
    
    calcQConfigIndex(map : NameChildMap) : [{ [key:string] : number }, string[]] {
        const qConfigIndex : { [key:string] : number } = {};
        const qConfigIndexReversed : string[] = [];

        let count = 0;
        for(let joint in map) {
            if (joint.includes("_XYZ_")) {
                qConfigIndex[joint + "/" + "x"] = count;
                qConfigIndexReversed.push(joint + "/" + "x");
                count = count + 1;

                qConfigIndex[joint + "/" + "y"] = count; 
                qConfigIndexReversed.push(joint + "/" + "y");
                count = count + 1;
                
                qConfigIndex[joint + "/" + "z"] = count; 
                qConfigIndexReversed.push(joint + "/" + "z");
                count = count + 1;
            } else if (joint.includes("_Z_")){
                qConfigIndex[joint] = count;
                qConfigIndexReversed.push(joint);
                count = count + 1;
            }
        }

        console.log("qConfigIndex", JSON.stringify(qConfigIndex));
        console.log("qConfigIndexReversed", qConfigIndexReversed.length, JSON.stringify(qConfigIndexReversed));

        return [qConfigIndex, qConfigIndexReversed];
    }

    forwardKinematics(q: JointAngles, A: Matrix4) {
        console.log(`forwardKinematics q: ${JSON.stringify(q)}`);
        const dq = q;

        // for (let name in q) {
        //     if (name.startsWith("RJoint_")) {
        //         if (name.includes("_XYZ_")) {
        //             dq[name] = { "x": q[name]["x"] - this.homePosition[name]["x"], "y": q[name]["y"]  - this.homePosition[name]["y"], "z": q[name]["z"] + this.homePosition[name]["z"] };
        //         } else if (name.includes("_Z_")) {
        //             dq[name] = q[name] - this.homePosition[name];
        //         }
        //     }
        // }

        return calcForwardKinematics(dq, A);
    }

    qConfigStringToIndex(label: string) {
        return this.qConfigIndex[label];
    }

    qConfigToVector(q:JointAngles) {
        const vector : number[] = new Array(this.qConfigLength).fill(-1000);

        for(let joint in q) {
            let qc = q[joint];
            if (isDictionary(qc)) {
                vector[this.qConfigIndex[joint + "/" + "x"]] = qc["x"];
                vector[this.qConfigIndex[joint + "/" + "y"]] = qc["y"];
                vector[this.qConfigIndex[joint + "/" + "z"]] = qc["z"];
            } else {
                vector[this.qConfigIndex[joint]] = qc;
            }
        }

        return vector;
    }

    calcJacobian(joint: string, q : JointAngles, dq : number, A : Matrix4 ) {
        const fwd1 = this.forwardKinematics(q,A);
        const m = fwd1[joint];
        const pos = new Vector3(m.elements[3*4 + 0], m.elements[3*4 + 1], m.elements[3*4 + 2]);
        const cVec = this.configToVector(q);

        for(let i = 0; i < this.qConfigLength; i++) {
            const vec = [ ...cVec ];
            vec[i] = vec[i] + dq;
            const q2 = VectorToConfig(vec);
            const fwd2 = this.forwardKinematics(q,A);
            const m2 = fwd1[joint];
            const pos2 = new Vector3(m2.elements[3*4 + 0], m2.elements[3*4 + 1], m2.elements[3*4 + 2]);        
        }
    }

    inverseKinematics(link: string, target : Vector3, 
            A: Matrix4, 
            alpha : number = 0.5, 
            limit : number = 0.01,
            maxIterations : number = 50) {
        const dErr = new Vector3(1000,1000,1000);
        let count = 0;

        while( ( dErr.lengthSq() >= limit ) && (count++ < maxIterations)) {
            const jac = this.calcJacobian(q,A);

        }
        return [0,dErr];
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

    configToVector(q : JointAngles) {
        const vec : number[] = new Array(this.qConfigLength).fill(-1000);
        for (let joint in q) {
            let qc = q[joint];
            if (isDictionary(qc)) {
                vec[this.qConfigIndex[joint + "/" + "x"]] = qc["x"];
                vec[this.qConfigIndex[joint + "/" + "y"]] = qc["y"];
                vec[this.qConfigIndex[joint + "/" + "z"]] = qc["z"];
            } else {
                vec[this.qConfigIndex[joint]] = qc;
            }
        }

        return vec;
    }

    vectorToConfig(vec: number[] ) {
        const q : JointAngles = {};

        for(let i = 0; i < vec.length; i++) {
            let jLabel = this.qConfigIndexReversed[i];
            if (jLabel.endsWith("/x")) {
                if (q[jLabel.substring(0,jLabel.length-2)] === undefined) {
                    q[jLabel.substring(0,jLabel.length-2)] = {} as Rot3Angles;
                }
                (q[jLabel.substring(0,jLabel.length-2)] as Rot3Angles)["x"] = vec[i];
            } else if (jLabel.endsWith("/y")) {
                if (q[jLabel.substring(0,jLabel.length-2)] === undefined) {
                    q[jLabel.substring(0,jLabel.length-2)] = {} as Rot3Angles;
                }
                (q[jLabel.substring(0,jLabel.length-2)] as Rot3Angles)["y"] = vec[i];
            } else if (jLabel.endsWith("/z")) {
                if (q[jLabel.substring(0,jLabel.length-2)] === undefined) {
                    q[jLabel.substring(0,jLabel.length-2)] = {} as Rot3Angles;
                }
                (q[jLabel.substring(0,jLabel.length-2)] as Rot3Angles)["z"] = vec[i];
            } else {
                q[jLabel] = vec[i];
            }
        }
    }
    getCurrentStateVector() {
        return this.configToVector(this.getCurrentStateConfig());
    }

    getCurrentStateConfig() {
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

    setConfiguration(q : JointAngles) {
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

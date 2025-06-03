import { calcForwardKinematics } from "./taiwanbear_kinematics";
import { Matrix4, Vector3, Object3D } from "three";

type Rot3Angles = { x: number, y: number, z: number };

interface JointAngles {
    [key: string]: number | Rot3Angles;
}

interface NameChildMap {
    [key: string]: Object3D;
}

type LinkTransformations = { [id:string] : Matrix4 };

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

        this.root.translateX(-1);
        this.root.translateZ(4);
        this.root.rotateX(Math.PI);

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

    forwardKinematics(q: JointAngles, A: Matrix4) : LinkTransformations {

        console.log(`forwardKinematics q: ${JSON.stringify(q)}`);
        const q2 = q;

        return calcForwardKinematics(q2, A);
    }

    qConfigStringToIndex(label: string) {
        return this.qConfigIndex[label];
    }

    calcJacobian(q : JointAngles, joint: string, pos: Vector3, dq : number, A : Matrix4 ) {
        const cVec = this.configToVector(q);
        const jac : Array<Vector3> = new Array<Vector3>();

        for(let i = 0; i < this.qConfigLength; i++) {
            const vec = [ ...cVec ];
            vec[i] = vec[i] + dq;
            const q2 = this.vectorToConfig(vec);
            const fwd2 = this.forwardKinematics(q2,A);
            const m2 = fwd2[joint];
            const pos2 = this.getPosition(m2);
            
            const diff = pos2.sub(pos);
            const partDeriv = diff.divideScalar(dq)
            
            jac.push(partDeriv);
        }

        return jac;
    }

    getPosition( m : Matrix4 ) {
        const me = m.elements;

        return new Vector3(me[0+3*4], me[1+3*4], me[2+3*4]);
    }

    inverseKinematics(q: JointAngles, joint: string, target : Vector3, 
            A: Matrix4, 
            alpha : number = 0.5, 
            limit : number = 0.01,
            maxIterations : number = 100) {
        var dErr = new Vector3(1000,1000,1000);
        const dq = 0.1;
        let count = 0;
        const vec = this.configToVector(q);

        while( ( dErr.lengthSq() >= limit ) && (count++ < maxIterations)) {
            console.log(`count ${count}, dErr ${dErr.lengthSq()}`);
            const q2 = this.vectorToConfig(vec)
            const fwd2 = this.forwardKinematics(q2, A);
            
            const pos = this.getPosition( fwd2[joint] );
            dErr = new Vector3( target.x - pos.x, target.y - pos.y, target.z - pos.z);

            const jac = this.calcJacobian(q2, joint, pos, dq, A);
            console.log("Jacobian", JSON.stringify(jac));

            for(let i = 0; i < this.qConfigLength; i++) {
                const jj = jac[i];
                const delta = jj.x * dErr.x + jj.y * dErr.y + jj.z * dErr.z;
                if (delta != 0.0) {
                    vec[i] = vec[i] + alpha * delta; 
                }
            }
        }
        return [this.vectorToConfig(vec),dErr];
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
        return q;
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

    setConfiguration(q : JointAngles) {
        for(let joint in q) {
            const child = this.map[joint];
            const qc = q[joint];

            if (isDictionary(qc)) {
                const qh = this.homePositionConfig[joint] as Rot3Angles;
                //child.rotation.set( qh["x"] - qc["x"], qh["y"] - qc["y"], qh["z"] - qc["z"], "XYZ");
                child.rotation.set( qc["x"], qc["y"], qc["z"], "XYZ");
                
            } else {
                const qh = this.homePositionConfig[joint] as number;
                //child.rotation.set( child.rotation.x, child.rotation.y, qh - qc, "XYZ");
                child.rotation.set( child.rotation.x, child.rotation.y, qc as number, "XYZ");
            }
        }
    }

}

export { Kinematics };
export type { JointAngles };
export type { Rot3Angles };

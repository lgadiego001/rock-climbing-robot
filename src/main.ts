import {
  AmbientLight,
  Clock,
  DirectionalLight,
  Mesh,
  MeshBasicMaterial,
  MeshToonMaterial,
  PlaneGeometry, 
  Vector2,
  Vector3,
  Matrix4, 
  Euler,
  Object3D,
} from 'three';

import camera from './core/camera';
import { fpsGraph, gui } from './core/gui';
import { controls } from './core/orbit-control';
import { renderer, scene } from './core/renderer';

import './style.css';
import { ThreeMFLoader } from 'three/examples/jsm/Addons.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { LinkTransformations, JointAngles, Kinematics, Rot3Angles } from './kinematics';
import { setupWall } from './wall';
import { update } from 'three/examples/jsm/libs/tween.module.js';


const loader = new GLTFLoader();

// Lights
const ambientLight = new AmbientLight(0xFFFFFF, 0.5)
scene.add(ambientLight)

const directionalLight = new DirectionalLight('#ffffff', 1)
directionalLight.castShadow = true
directionalLight.shadow.mapSize.set(1024, 1024)
directionalLight.shadow.camera.far = 15
directionalLight.shadow.normalBias = 0.05
directionalLight.position.set(0.25, 2, 2.25)

scene.add(directionalLight)

const [wall,holds] = await setupWall("./assets/climbing holds.glb", "/route1.txt", 1.0, 1.5);
scene.add(wall);

var bear = null;
var bearKinematics : Kinematics | null = null;


loader.load("./assets/taiwan bear.glb", (gltf) => {
  bear = gltf.scene.children[0];
  {
    //bear.position.set(0,-4,2);
    //bear.rotateX(Math.PI);
  }

  bear.updateMatrixWorld();
  bearKinematics = new Kinematics(bear);
  scene.add(bear);

  // Set the directions for turns

  {
    const q = bearKinematics.getCurrentStateConfig();
    q["RJoint_Back_Lower_Z_L"] = q["RJoint_Back_Lower_Z_L"] + Math.PI/4;
    q["RJoint_Back_Lower_Z_R"] = q["RJoint_Back_Lower_Z_R"] - Math.PI/4;
    q["RJoint_Front_Lower_Z_L"] = q["RJoint_Front_Lower_Z_L"] - Math.PI/8;
    q["RJoint_Front_Lower_Z_R"] = q["RJoint_Front_Lower_Z_R"] + Math.PI/8;
    
    bearKinematics.setConfiguration(q);  
  }

  // const JointFolder = gui.addFolder({
  //   title:"Joint Control"
  
  // })

  // let joints : Array<Object3D> = [bear];

  // while(joints.length > 0) {
  //   const it = joints.pop();
  //   console.log(`bear child ${it.name}`);
  //   if (it.name.substring(0,5) == "Joint") {
  //     console.log(`Adding joint ${it.name}`);
  //   } 
  //   for(const c of it.children) {
  //     joints.push(c);
  //   }
  
  //   // JointFolder.addBinding(
  //   //   directionalLight.position,
  //   //   key as keyof Vector3,
  //   //   {
  //   //     min: -100,
  //   //     max: 100,
  //   //     step: 1,
  //   //   },
  //   // )
  // }
})

const DirectionalLightFolder = gui.addFolder({
  title: 'Directional Light',
})

Object.keys(directionalLight.position).forEach((key) => {
  DirectionalLightFolder.addBinding(
    directionalLight.position,
    key as keyof Vector3,
    {
      min: -100,
      max: 100,
      step: 1,
    },
  )
})


  // const plane = new Mesh(
  //   new PlaneGeometry(10, 10, 10, 10),
  //   new MeshToonMaterial({ color: '#444' }),
  // )

  // plane.rotation.set(-Math.PI / 2, 0, 0)
  // plane.receiveShadow = true
  // scene.add(plane)

camera.position.set(0,0,20);
camera.lookAt(new Vector3(0, 0, 0));
camera.up.set(0, -1, 0); // Set the up direction to Y axis
camera.updateMatrixWorld();

camera.updateProjectionMatrix();
controls.update();

const clock = new Clock()

async function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function isDictionary(obj: unknown): obj is Record<string, unknown> {
    return typeof obj === 'object' && obj !== null && !Array.isArray(obj);
}

function printQConfig( name: string, q: JointAngles ) {
  console.log("Configuration"), name;
  for(let joint in q) {

    if (isDictionary(q[joint])) {
      const qc = q[joint] as Rot3Angles;
      console.log("Joint ", joint, (qc['x']/Math.PI * 180.0).toFixed(2), (qc['y']/Math.PI * 180.0).toFixed(2), (qc['z']/Math.PI * 180.0).toFixed(2) );
    } else {
      const qc = q[joint] as number;
      console.log("Joint ", joint, (qc/Math.PI * 180.0).toFixed(2) );
    }
  }
}

function printKinematics( name: string, fwd: LinkTransformations ) {
  console.log("Configuration", name);
  for(let joint in fwd) {
    const j = fwd[joint];
    console.log(joint, "x", j.elements[0+3*4].toFixed(2), "y", j.elements[1+3*4].toFixed(2), "z", j.elements[2+3*4].toFixed(2) );
  }
}

function printMatrix(name:string, m:Matrix4) {
    console.log("Matrix", name);
    for(let i = 0; i < 4; i++) {
        console.log(m.elements[i+0*4].toFixed(2),m.elements[i+1*4].toFixed(2),m.elements[i+2*4].toFixed(2),m.elements[i+3*4].toFixed(2));
    }
    console.log("End of Matrix", name);
}


function findClosestHold(pos : Vector3, holds: Object3D[] ) {
  var min = 1000000;
  var minHold : Object3D = holds[0];

  for(let h of holds) {
    const hPos = h.position;
    const dist = pos.distanceToSquared(hPos);

    console.log(pos.x, pos.y, pos.z, '<->', hPos.x, hPos.y, hPos.z, 'dist', dist);
    if (dist < min ) {
      min = dist;
      minHold = h;
    }
  }
  return minHold;
}

function initialPosition( bearKinematics: Kinematics, holds: Object3D[] ) {
  var init = false;

  if ( holds.length > 0 ) {
    for(let eff of ["Effector_Back_R" ]) { //"Effector_Front_R", "Effector_Front_L"]) {
      const q = bearKinematics.getCurrentStateConfig();
      const origin = bearKinematics.root.matrixWorld;
      const fwd = bearKinematics.forwardKinematics(q, origin);
      printKinematics('old Kin', fwd);
      //console.log(`initialPosition: current ${JSON.stringify(q)}\n\nkin ${JSON.stringify(fwd)}`);
  
      const m = fwd[eff];
      const current = new Vector3(m.elements[0 + 3*4], m.elements[1+3*4], m.elements[2+3*4]);

      console.log(`${eff} ${JSON.stringify(current)}`);

      const h = findClosestHold(current, holds);
      h.position.set(-current.x + 0.5,current.y,current.z);
      h.updateMatrixWorld();      

      const dist = current.distanceToSquared(h.position);
      console.log(`closest hold for ${eff} ${JSON.stringify(current)} ${h.name} ${JSON.stringify(h.position)} dist ${dist}`);

      const [newConfig, err, count, fwd2] = bearKinematics.inverseKinematics(q, eff, h.position, origin, 0.3);
      console.log('IK: err', err.lengthSq(), "vec", JSON.stringify(err), count);
      printQConfig('newConfig', newConfig);
      printKinematics('new Kin', fwd2);
      
      // const fwd2 = bearKinematics.forwardKinematics(newConfig as JointAngles, origin);
      // const mAfter = fwd2[eff];
      // const currentAfter = new Vector3(mAfter.elements[0 + 3*4], mAfter.elements[1+3*4], mAfter.elements[2+3*4]);
      // const distAfter = currentAfter.distanceToSquared(h.position);
      // console.log(`after ik closest hold for ${eff} ${JSON.stringify(currentAfter)} ${h.name} ${JSON.stringify(h.position)} dist ${distAfter}`);

      
      bearKinematics.setConfiguration(newConfig as JointAngles);

      init = true;
    }
  }

  return init;
}

var initialized = false;

const loop = async () => {
  const elapsedTime = clock.getElapsedTime()

  fpsGraph.begin()

  //controls.update();
  renderer.render(scene, camera);


  if (bearKinematics) {
    //const origin : Matrix4 = bearKinematics.root.matrixWorld;
    
    if (! initialized) {
      initialized = initialPosition(bearKinematics, holds);
    }

    let q = bearKinematics.getCurrentStateConfig();
     
    console.log(`q: ${JSON.stringify(q)}`);
    
    // const fwd = bearKinematics.forwardKinematics(q, origin);
    // console.log(`fwd: ${JSON.stringify(fwd)}`);
    // for (let effector in fwd) {
    //   const m = fwd[effector];
    //   console.log(effector,"x",m.slements[0 + 3*4], "y",m.elements[1+3*4], "z",m.elements[2+3*4]);
    // }
  
    {
      if (false) {
        const n = q['RJoint_Torso_XYZ_C'] as Rot3Angles;
        n["z"] = n["z"] + 0.2;
        q["RJoint_Torso_XYZ_C"] = n;
      }
    }

  // bearKinematics.map["RJoint_Back_Upper_XYZ_R"].rotation.z += 0.2;;
  
    bearKinematics.setConfiguration(q);

  
    //const joint = "Effector_Front_R";
    //const me = fwd["Effector_Front_R"].elements;
    //const pos = new Vector3( me[0+3*4], me[1+3*4], me[2+3*4] );

    // if (holds.length > 0) {
    //   const target = new Vector3();
    //   holds[0].getWorldPosition(target);

    //   const [newConfig, _] = bearKinematics.inverseKinematics(q, joint, target, origin, 0.3);
    //   console.log('newConfig', JSON.stringify(newConfig));
    //   bearKinematics.setConfiguration(newConfig as JointAngles);
    // }

    await sleep(100); 
  }

  fpsGraph.end()
  requestAnimationFrame(loop)
}

loop();

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
} from 'three'

import camera from './core/camera'
import { fpsGraph, gui } from './core/gui'
import { controls } from './core/orbit-control'
import { renderer, scene } from './core/renderer'

import './style.css'
import { ThreeMFLoader } from 'three/examples/jsm/Addons.js'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import { Kinematics, Rot3Angles } from './kinematics'


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

loader.load("./assets/climbing holds.glb", (gltf) => {
  const wall = gltf.scene.children[0];  // sword 3D object is loaded
  //holds.translateZ(-1);
  wall.scale.set(3,5,0.1);
  wall.position.set(0,0,-0.1)
  scene.add(wall);

  const getObject = (name:string) => {
    var obj : Object3D | null = null;
    for( const it of gltf.scene.children ) {
      if(it.name == name) {
        obj = it;
        break
      }
    }
    return obj;
  } 

  const jug1Center = getObject("Jug1Center");
  jug1Center?.setRotationFromEuler(new Euler(90.0/180.0*Math.PI, 0.0/180.0*Math.PI, 0.0/180.0*Math.PI));
  scene.add(jug1Center!);

  const jug1Left = getObject("Jug1Left");
  jug1Left?.setRotationFromEuler(new Euler(90.0/180.0*Math.PI, 45.0/180.0*Math.PI, 0.0/180.0*Math.PI));
  scene.add(jug1Left!);

  const jug1Right = getObject("Jug1Right");
  jug1Right?.setRotationFromEuler(new Euler(90.0/180.0*Math.PI, -45.0/180.0*Math.PI, 0.0/180.0*Math.PI));
  scene.add(jug1Right!);

  for(var i = 0; i < 10; i++ ) {
    const cp = jug1Center!!.clone();
    cp.translateX(0.2 * i);
    scene.add(cp);
  }
  
  console.log(`center ${jug1Center} left ${jug1Left} right ${jug1Right}`);
  // import fragmentShader from '/@/shaders/fragment.glsl'
  // // Shaders
  // import vertexShader from '/@/shaders/vertex.glsl'



  // const sphereMaterial = new ShaderMaterial({
  //   uniforms: {
  //     uTime: { value: 0 },
  //     uFrequency: { value: new Vector2(20, 15) },
  //   },
  //   vertexShader,
  //   fragmentShader,
  // })

  // const sphere = new Mesh(
  //   new SphereGeometry(1, 32, 32),
  //   sphereMaterial,
  // )

  // sphere.position.set(0, 2, 0)
  // sphere.castShadow = true
  // scene.add(sphere)

  // const geometry = new THREE.BoxGeometry(1, 0.2, 2);
  // const material = new THREE.MeshBasicMaterial({ color: 0xa02030 });
  // const cube = new THREE.Mesh(geometry, material);
  // scene.add(cube);
});

var bear = null;
var bearKinematics : Kinematics | null = null;


loader.load("./assets/taiwan bear.glb", (gltf) => {
  bear = gltf.scene.children[0];
  scene.add(bear);
  bearKinematics = new Kinematics(bear);

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

const clock = new Clock()

//camera.position = new Vector3(0.00, 2.6, -1);
camera.setRotationFromEuler( new Euler(-Math.PI/2, Math.PI/3, 0));
camera.updateMatrixWorld();
controls.update();

async function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function printMatrix(name:string, m:Matrix4) {
    console.log("Matrix", name);
    for(let i = 0; i < 4; i++) {
        console.log(m.elements[i+0*4].toFixed(2),m.elements[i+1*4].toFixed(2),m.elements[i+2*4].toFixed(2),m.elements[i+3*4].toFixed(2));
    }
    console.log("End of Matrix", name);
}

const loop = async () => {
  const elapsedTime = clock.getElapsedTime()

  fpsGraph.begin()

  controls.update()
  // cube.rotation.x += 1.0 / 180.0 * Math.PI
  // cube.rotation.z += 5.0 / 180.0 * Math.PI

  renderer.render(scene, camera)
  const ID4 = new Matrix4();
  printMatrix("ID4", ID4);

  if (bearKinematics) {
    let q = bearKinematics.vectorToConfig( new Array(bearKinematics.qConfigLength).fill(0) );
    console.log(`q: ${JSON.stringify(q)}`);

    const fwd = bearKinematics.forwardKinematics(q, ID4);
    console.log(`fwd: ${JSON.stringify(fwd)}`);
    for (let effector in fwd) {
      const m = fwd[effector];
      console.log(effector,"x",m.elements[0 + 3*4], "y",m.elements[1+3*4], "z",m.elements[2+3*4]);
    }

    //const n = q['RJoint_Torso_XYZ_C'] as Rot3Angles;
    //n["x"] = n["x"] + 0.2;
    //q["RJoint_Torso_XYZ_C"] = n;
    
    //bearKinematics.map["RJoint_Back_Upper_XYZ_R"].rotation.z += 0.2;;
    //bearKinematics.setConfiguration(q);
    const joint = "Effector_Front_L";
    const me = fwd["Effector_Front_L"].elements;
    const pos = new Vector3( me[0+3*4], me[1+3*4], me[2+3*4] );

    const [newConfig, _] = bearKinematics.inverseKinematics(q, joint, new Vector3(pos.x + 0.3, pos.y + 0.3, pos.z + 0.3), ID4, 0.3);
    bearKinematics.setConfiguration(newConfig);

    await sleep(100); 
  }

  fpsGraph.end()
  requestAnimationFrame(loop)
}

loop();

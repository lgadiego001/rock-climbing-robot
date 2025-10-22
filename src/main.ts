import type { Object3D } from 'three'

import type {
  JointAngles,
  LinkTransformations,
  Rot3Angles,
} from './kinematics'

import type {
  Route,
} from './wall'

import { AmbientLight, DirectionalLight, Matrix4, Vector3 } from 'three'

import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'

import camera from './core/camera'
import { fpsGraph, gui } from './core/gui'
import { controls } from './core/orbit-control'
import { renderer, scene } from './core/renderer'
import { Kinematics } from './kinematics'
import { setupWall, marker_on_off } from './wall'
import './style.css'

const loader = new GLTFLoader()

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

let wall: Object3D | null = null
let route: Route | null = null

setupWall(
  '/rock-climbing-robot/climbing holds yup.glb',
  '/rock-climbing-robot/route3.txt',
  1.0,
  1.5,
).then( (result) => {
  const [w, r] = result
  wall = w
  route = r
  scene.add(wall as Object3D)
})


let bear = null
let bearKinematics: Kinematics | null = null

loader.load('/rock-climbing-robot/taiwan bear.glb', (gltf) => {
  bear = gltf.scene.children[0]
  
  bearKinematics = new Kinematics(bear, false)
  scene.add(bear)

  {
    bear.position.set(0, 1.0, -6.5)
    //bear.rotation.set(-Math.PI/2,0,0)
    // bear.rotateX(Math.PI)
  }

  bear.updateMatrixWorld(true)

  // Set the directions for turns

  {
    const q = bearKinematics.getCurrentStateConfig()
    q.RJoint_Back_Lower_Z_L = (q.RJoint_Back_Lower_Z_L as number) + Math.PI / 4
    q.RJoint_Back_Lower_Z_R = (q.RJoint_Back_Lower_Z_R as number) - Math.PI / 4
    q.RJoint_Front_Lower_Z_L
      = (q.RJoint_Front_Lower_Z_L as number) - Math.PI / 8
    q.RJoint_Front_Lower_Z_R
      = (q.RJoint_Front_Lower_Z_R as number) + Math.PI / 8

    bearKinematics.setConfiguration(q)
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

camera.position.set(0, 20, -5)
// camera.lookAt(new Vector3(0, 0, 0))
// camera.up.set(0, 0, 1) // Set the up direction to Y axis
// camera.updateMatrixWorld()

camera.updateProjectionMatrix()
controls.update()

//const clock = new Clock()

async function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms))
}

function isDictionary(obj: unknown): obj is Record<string, unknown> {
  return typeof obj === 'object' && obj !== null && !Array.isArray(obj)
}

function printQConfig(name: string, q: JointAngles) {
  console.log('Configuration', name)
  for (const joint in q) {
    if (isDictionary(q[joint])) {
      const qc = q[joint] as Rot3Angles
      console.log(
        'Joint ',
        joint,
        ((qc.x / Math.PI) * 180.0).toFixed(2),
        ((qc.y / Math.PI) * 180.0).toFixed(2),
        ((qc.z / Math.PI) * 180.0).toFixed(2),
      )
    }
    else {
      const qc = q[joint] as number
      console.log('Joint ', joint, ((qc / Math.PI) * 180.0).toFixed(2))
    }
  }
}

function printKinematics(
  name: string,
  fwd: LinkTransformations,
  root: Object3D | null = null,
  showLinks = false,
) {
  console.log('Configuration', name)
  for (const joint in fwd) {
    if (showLinks || !joint.startsWith('Link')) {
      const j = fwd[joint]
      console.log(
        'kin',
        joint,
        'x',
        j.elements[0 + 3 * 4].toFixed(2),
        'y',
        j.elements[1 + 3 * 4].toFixed(2),
        'z',
        j.elements[2 + 3 * 4].toFixed(2),
      )
      if (root) {
        const obj = root.getObjectByName(joint)
        obj?.updateMatrixWorld(true)
        const pos = new Vector3()
        obj?.getWorldPosition(pos)
        console.log(
          'world',
          joint,
          'x',
          pos?.x.toFixed(2),
          'y',
          pos?.y.toFixed(2),
          'z',
          pos?.z.toFixed(2),
        )
      }
    }
  }
}

function printMatrix(name: string, m: Matrix4) {
  console.log('Matrix', name)
  for (let i = 0; i < 4; i++) {
    console.log(
      m.elements[i + 0 * 4].toFixed(2),
      m.elements[i + 1 * 4].toFixed(2),
      m.elements[i + 2 * 4].toFixed(2),
      m.elements[i + 3 * 4].toFixed(2),
    )
  }
  console.log('End of Matrix', name)
}

function findClosestHold(pos: Vector3, holds: Object3D[], coM: Vector3) {
  let min = 1000000
  let minHold: Object3D = holds[0]

  for (let h of holds) {
    const hPos = h.position
    const dist = pos.distanceToSquared(hPos)

    console.log(
      pos.x,
      pos.y,
      pos.z,
      '<->',
      hPos.x,
      hPos.y,
      hPos.z,
      'dist',
      dist,
    )
    if ((h.name.includes('Left') && (h.position.x >= coM.x)) || (h.name.includes('Right') && (h.position.x <= coM.x )) || (h.name.includes("Center") )) {
      console.log(`Legal hold ${h.name} ${h.position.x} ${coM.position}`)
      if (dist < min) {
        min = dist
        minHold = h
      }
    }
  }
  return minHold
}

function showKinematics(root: Object3D) {
  printMatrix(`M_${root.name}`, root.matrix)
  printMatrix(`A_${root.name}`, root.matrixWorld)
  for (const c of root.children) {
    showKinematics(c)
  }
}

function hang_on_wall(bearKinematics: Kinematics, holds: Object3D[], setMarkers = false) {
  let init = false

  if (holds.length > 0) {

    for( const h of holds) {
      marker_on_off(h, false)
    } 

    for (const eff of ['Effector_Back_L', 'Effector_Back_R', 'Effector_Front_L', 'Effector_Front_R']) {
      // "Effector_Front_R", "Effector_Front_L"]) {
      const q = bearKinematics.getCurrentStateConfig()
      const origin = bearKinematics.root.matrixWorld

      const fwdBlender = bearKinematics.forwardKinematics(q, new Matrix4())
      printKinematics('old Kin (Blender)', fwdBlender)

      const fwd = bearKinematics.forwardKinematics(q, origin)
      printKinematics('old Kin (world)', fwd, bearKinematics.root)

      showKinematics(bearKinematics.root)

      // console.log(`initialPosition: current ${JSON.stringify(q)}\n\nkin ${JSON.stringify(fwd)}`);

        // const m = fwd[eff]
        // const current = new Vector3(
        //   m.elements[0 + 3 * 4],
        //   m.elements[1 + 3 * 4],
        //   m.elements[2 + 3 * 4],
        // )
        // const h = findClosestHold(current, holds)
        // //h.updateMatrixWorld(true)
      
      const def_m = bearKinematics.homePositionFwd[eff]
      const m = new Matrix4()
      m.multiplyMatrices(origin,def_m)
      const current = new Vector3(
          m.elements[0 + 3 * 4],
          m.elements[1 + 3 * 4],
          m.elements[2 + 3 * 4],
        )
      console.log(`${eff} ${JSON.stringify(current)}`)

      const h = findClosestHold(current, holds, bearKinematics.root.position)
      //h.updateMatrixWorld(true)
    
      //h.position.set(current.x + 0.2, current.y + 1.3, current.z - 0.25)
      
      //const dist = current.distanceToSquared(h.position)

      const [newConfig, err, count, fwd2] = bearKinematics.inverseKinematics(
        q,
        eff,
        h.position,
        origin,
        0.1,
        0.05,
        10
      )
      console.log('IK: err', err.lengthSq(), 'vec', JSON.stringify(err), count)
      printQConfig('newConfig', newConfig)
      printKinematics('new Kin', fwd2)

      if (setMarkers) {
        if (err.lengthSq() <= 0.1) {
          marker_on_off(h, true)
        // } else {
        //   hold_on_off(h, false)
        }
      }
      // const fwd2 = bearKinematics.forwardKinematics(newConfig as JointAngles, origin);
      // const mAfter = fwd2[eff];
      // const currentAfter = new Vector3(mAfter.elements[0 + 3*4], mAfter.elements[1+3*4], mAfter.elements[2+3*4]);
      // const distAfter = currentAfter.distanceToSquared(h.position);
      // console.log(`after ik closest hold for ${eff} ${JSON.stringify(currentAfter)} ${h.name} ${JSON.stringify(h.position)} dist ${distAfter}`);

      bearKinematics.setConfiguration(newConfig as JointAngles)

      init = true
    }
  }

  return init
}

//let initialized = false

let pathCounter = 0

const loop = async () => {
  //const elapsedTime = clock.getElapsedTime()

  fpsGraph.begin()

  // controls.update();
  renderer.render(scene, camera)

  if ((bearKinematics) && (route)) {
    //const origin : Matrix4 = bearKinematics.root.matrixWorld

    //let q = bearKinematics.getCurrentStateConfig()

    // console.log(`q: ${JSON.stringify(q)}`)

    // {
    //   const fwd = bearKinematics.forwardKinematics(q, origin);
    //   printKinematics('main before rotation', fwd)
    // }
    // {
    //   if (true) {
    //     const n = q.RJoint_Back_Ankle_Z_L as number
    //     q.RJoint_Back_Ankle_Z_L = n + 0.1
    //   }
    // }
    // bearKinematics.setConfiguration(q)
    // {
    //   const fwd = bearKinematics.forwardKinematics(q, origin);
    //   printKinematics('main after rotation', fwd)
    // }

    // {
    //   q.RJoint_Back_Ankle_Z_L = q.RJoint_Back_Ankle_Z_L - 0.1
    //   bearKinematics.setConfiguration(q)
    //   const fwd4 = bearKinematics.forwardKinematics(q, origin);
    //   printKinematics('main after reset', fwd4)
    // }


    hang_on_wall(bearKinematics, route.holds, true)

    // bearKinematics.map["RJoint_Back_Upper_XYZ_R"].rotation.z += 0.2;;

    if (pathCounter < 20) {
      bearKinematics.root.translateZ(0.05)
    } else if (pathCounter < 30) {
      bearKinematics.root.translateZ(0.05)
      bearKinematics.root.translateX(0.1)
    } else if (pathCounter < 50) {
      bearKinematics.root.translateZ(0.05)
    } else if (pathCounter < 60) {
      bearKinematics.root.translateZ(0.1)
      bearKinematics.root.translateX(0.05)
    } else if (pathCounter < 80) {
      bearKinematics.root.translateZ(0.1)
    } else if (pathCounter < 120) {
      bearKinematics.root.translateZ(0.1)
      bearKinematics.root.translateX(-0.05)
    } else if (pathCounter < 130) {
      bearKinematics.root.translateZ(0.1)
    }
    bearKinematics.root.updateMatrixWorld(true)

    
    // const joint = "Effector_Front_R";
    // const me = fwd["Effector_Front_R"].elements;
    // const pos = new Vector3( me[0+3*4], me[1+3*4], me[2+3*4] );

    // if (holds.length > 0) {
    //   const target = new Vector3();
    //   holds[0].getWorldPosition(target);

    //   const [newConfig, _] = bearKinematics.inverseKinematics(q, joint, target, origin, 0.3);
    //   console.log('newConfig', JSON.stringify(newConfig));
    //   bearKinematics.setConfiguration(newConfig as JointAngles);
    // }
    pathCounter += 1
    await sleep(10)
  }

  fpsGraph.end()
  requestAnimationFrame(loop)
}

loop()

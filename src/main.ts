import type { Object3D } from 'three'

import type {
  JointAngles,
  LinkTransformations,
  Rot3Angles,
} from './kinematics'

import { AmbientLight, Clock, DirectionalLight, Matrix4, Vector3 } from 'three'

import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'

import camera from './core/camera'
import { fpsGraph, gui } from './core/gui'
import { controls } from './core/orbit-control'
import { renderer, scene } from './core/renderer'
import { Kinematics } from './kinematics'
import { setupWall } from './wall'
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

const [wall, route] = await setupWall(
  './assets/climbing holds yup.glb',
  '/route3.txt',
  1.0,
  1.5,
)

scene.add(wall)

let bear = null
let bearKinematics: Kinematics | null = null

loader.load('./assets/taiwan bear.glb', (gltf) => {
  bear = gltf.scene.children[0]
  {
    bear.position.set(-5.5, 1.0, -6.5)
    //bear.rotation.set(-Math.PI/2,0,0)
    // bear.rotateX(Math.PI)
  }

  bear.updateMatrixWorld(true)
  bearKinematics = new Kinematics(bear, false)
  scene.add(bear)

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

const clock = new Clock()

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

function findClosestHold(pos: Vector3, holds: Object3D[]) {
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
    if (dist < min) {
      min = dist
      minHold = h
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

function initialPosition(bearKinematics: Kinematics, holds: Object3D[]) {
  let init = false
  console.log(
    `holds ${holds} holds.length ${holds.length} ${holds.length > 0}`,
  )
  if (holds.length > 0) {
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

      const m = fwd[eff]
      const current = new Vector3(
        m.elements[0 + 3 * 4],
        m.elements[1 + 3 * 4],
        m.elements[2 + 3 * 4],
      )

      console.log(`${eff} ${JSON.stringify(current)}`)

      const h = findClosestHold(current, holds)
      //h.position.set(current.x + 0.2, current.y + 1.3, current.z - 0.25)
      h.updateMatrixWorld(true)

      const dist = current.distanceToSquared(h.position)
      console.log(
        `closest hold for ${eff} ${JSON.stringify(current)} ${h.name} ${JSON.stringify(h.position)} dist ${dist}`,
      )

      const [newConfig, err, count, fwd2] = bearKinematics.inverseKinematics(
        q,
        eff,
        h.position,
        origin,
        0.3,
      )
      console.log('IK: err', err.lengthSq(), 'vec', JSON.stringify(err), count)
      printQConfig('newConfig', newConfig)
      printKinematics('new Kin', fwd2)

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

let initialized = false

const loop = async () => {
  const _elapsedTime = clock.getElapsedTime()

  fpsGraph.begin()

  // controls.update();
  renderer.render(scene, camera)

  if (bearKinematics) {
    const origin : Matrix4 = bearKinematics.root.matrixWorld;

    let q = bearKinematics.getCurrentStateConfig()

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


    if (!initialized) {
      const holds = route.holds
      console.log(`loop ${holds.length}`)
      initialized = initialPosition(bearKinematics, holds)
    }

    // bearKinematics.map["RJoint_Back_Upper_XYZ_R"].rotation.z += 0.2;;

    
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

    await sleep(100)
  }

  fpsGraph.end()
  requestAnimationFrame(loop)
}

loop()

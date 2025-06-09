import type { Object3D } from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'

interface Route {
  level: number
  holds: Object3D[]
}

async function setupWall(wallURL: string, routeURL: string, dx = 0.75, dy = 1.0): Promise<[Object3D, Route]> {
  async function loadRoute(url: string): Promise<string> {
    try {
      const response = await fetch(url)
      if (!response.ok) {
        throw new Error(`HTTP error! Status: ${response.status}`)
      }
      const text = await response.text()
      return text
    }
    catch (error) {
      console.error('Error loading file:', error)
      throw error
    }
  }

  async function loadWallAssets(url: string): Promise<Object3D> {
    return new Promise((resolve, reject) => {
      const loader = new GLTFLoader()
      loader.load(url, (gltf) => {
        resolve(gltf.scene)
      }, undefined, (error) => {
        reject(error)
      })
    })
  }

  const [gltf, route] = await Promise.all([loadWallAssets(wallURL), loadRoute(routeURL)])

  const holds: Object3D[] = []

  const wall = gltf.getObjectByName('Wall1')
  wall!.children = []
  // wall.position.set(0, 0, 0);

  const jugCenter1 = gltf.getObjectByName('JugCenter1')

  const jugRight1 = gltf.getObjectByName('JugRight1')

  const jugLeft1 = gltf.getObjectByName('JugLeft1')

  const crimpCenter1 = gltf.getObjectByName('CrimpCenter1')

  const jugCenter2 = gltf.getObjectByName('JugCenter2')

  const lines = route.split('\n')
  const height = lines.length
  const width = lines[0].length
  const level = Number.parseInt(lines[0].trim(), 10)

  // wall!!.setRotationFromEuler(new Euler(0, 0, Math.PI));
  // wall!!.updateMatrixWorld();
  // wall!.visible = false;

  for (let i = 1; i < lines.length; i++) {
    const line = lines[i]
    if (line.length === 0 || line.startsWith('#')) {
      continue // Skip empty lines and comments
    }

    for (let j = 0; j < line.length; j++) {
      const char = line[j]
      let hold: Object3D = jugCenter1!.clone()
      let name = 'INVALID'
      if (char === 'U') {
        name = `JugCenter1_${i}_${j}`
        hold = jugCenter1!.clone()
      }
      else if (char === 'R') {
        name = `JugRight1_${i}_${j}`
        hold = jugRight1!.clone()
      }
      else if (char === 'L') {
        name = `JugLeft1_${i}_${j}`
        hold = jugLeft1!.clone()
      }
      else if (char === 'C') {
        name = `CrimpCenter1_${i}_${j}`
        hold = crimpCenter1!.clone()
      }
      else if (char === 'V') {
        name = `JugCenter2_${i}_${j}`
        hold = jugCenter2!.clone()
      }
      else {
        continue
      }
      hold.position.set((width - 1 - j - width / 2) * dx, 0, (height - 1 - i - height / 2) * dy)
      // hold.scale.set(1, 1, 1)
      hold.name = name
      hold.updateMatrixWorld(true)

      wall!.add(hold)
      holds.push(hold)
    }
  }

  return [wall!, { level, holds }]
}

export { Route, setupWall }

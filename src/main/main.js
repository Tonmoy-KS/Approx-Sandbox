import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

// --- Vector/Quaternion/Shape ---
class Vec3 {
  constructor(x=0, y=0, z=0) { this.x=x; this.y=y; this.z=z;}
  add(v) { return new Vec3(this.x+v.x, this.y+v.y, this.z+v.z);}
  sub(v) { return new Vec3(this.x-v.x, this.y-v.y, this.z-v.z);}
  mul(s) { return new Vec3(this.x*s, this.y*s, this.z*s);}
  dot(v) { return this.x*v.x + this.y*v.y + this.z*v.z;}
  length() { return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z);}
  normalize() { const l=this.length(); return l===0?new Vec3():this.mul(1/l);}
  copy() { return new Vec3(this.x,this.y,this.z);}
  set(x, y, z) {this.x=x;this.y=y;this.z=z;}
  static zero() { return new Vec3(0,0,0);}
}

class Quaternion { // Minimal placeholder for orientation
  constructor(x=0,y=0,z=0,w=1){this.x=x;this.y=y;this.z=z;this.w=w;}
}

const SHAPE_TYPE = { SPHERE:0, BOX:1, CYLINDER:2, CUSTOM:3 };
class SphereShape { constructor(radius) { this.type=SHAPE_TYPE.SPHERE; this.radius=radius; } }
class BoxShape { constructor(size) { this.type=SHAPE_TYPE.BOX; this.size=size; } }
class CylinderShape {
  constructor(radius=1, height=2) {
    this.type = SHAPE_TYPE.CYLINDER;
    this.radius = radius;
    this.height = height;
  }
}
class CompoundShape {
  constructor(shapes) {
    this.type = SHAPE_TYPE.CUSTOM;
    this.shapes = shapes; // array of {shape, offset}
  }
}

// --- Rigid Body ---
class RigidBody {
  constructor(position, shape, mass, mesh) {
    this.position = position.copy();
    this.velocity = Vec3.zero();
    this.force = Vec3.zero();
    this.shape = shape;
    this.mass = mass;
    this.invMass = mass>0 ? 1/mass : 0;
    this.mesh = mesh;
    this.restitution = 0.35;
    this.friction = 0.3;
    this.angularVelocity = Vec3.zero();
    this.orientation = new Quaternion();
    this.torque = Vec3.zero();
    // Inertia (demo-only for sphere/cylinder)
    if(shape.type === SHAPE_TYPE.SPHERE)
      this.inertia = (2/5)*mass*Math.pow(shape.radius,2);
    else if(shape.type === SHAPE_TYPE.CYLINDER)
      this.inertia = (1/2)*mass*Math.pow(shape.radius,2);
    else
      this.inertia = 1;
    this.invInertia = this.inertia > 0 ? 1/this.inertia : 0;
  }
  applyForce(f) { this.force = this.force.add(f); }
  applyTorque(t) { this.torque = this.torque.add(t); }
  integrate(dt) {
    if(this.invMass===0) return;
    // linear
    this.velocity = this.velocity.add(this.force.mul(dt*this.invMass));
    this.position = this.position.add(this.velocity.mul(dt));
    // angular
    this.angularVelocity = this.angularVelocity.add(this.torque.mul(dt*this.invInertia));
    this.torque = Vec3.zero();
    if(this.mesh) {
      this.mesh.position.set(this.position.x, this.position.y, this.position.z);
      if(this.angularVelocity.length() > 0.0001) {
        this.mesh.rotateX(this.angularVelocity.x * dt);
        this.mesh.rotateY(this.angularVelocity.y * dt);
        this.mesh.rotateZ(this.angularVelocity.z * dt);
      }
    }
    this.force = Vec3.zero();
  }
}

// --- Physics World ---
class PhysicsWorld {
  constructor() {
    this.bodies = [];
    this.gravity = new Vec3(0, -10, 0);
    this.bounds = {x:20,y:20,z:20};
    this.collisionCount = 0;
  }
  addBody(b) { this.bodies.push(b);}
  removeBody(b) { const i=this.bodies.indexOf(b); if(i>-1) this.bodies.splice(i,1);}
  update(dt) {
    this.collisionCount = 0;
    // Broadphase spatial hash
    let grid = {};
    for(const b of this.bodies) {
      let key = `${Math.floor(b.position.x)}:${Math.floor(b.position.y)}:${Math.floor(b.position.z)}`;
      if(!grid[key]) grid[key]=[];
      grid[key].push(b);
    }
    for(const b of this.bodies) {
      b.applyForce(this.gravity.mul(b.mass));
      b.integrate(dt);
    }

    // Ground and Bounds Collision
    for(const b of this.bodies) {
        let halfSize = 0;
        if (b.shape.type === SHAPE_TYPE.SPHERE) halfSize = b.shape.radius;
        else if (b.shape.type === SHAPE_TYPE.BOX) halfSize = b.shape.size / 2;
        else if (b.shape.type === SHAPE_TYPE.CYLINDER) halfSize = b.shape.height / 2;
        
        // Ground collision
        if(b.position.y < halfSize) {
          b.position.y = halfSize;
          b.velocity.y = -b.velocity.y * b.restitution;
          // Apply friction
          b.velocity.x *= (1 - b.friction);
          b.velocity.z *= (1 - b.friction);
        }

        // Wall/Ceiling collisions
        for(const axis of ['x','z']) {
            if(b.position[axis] < -this.bounds[axis]/2 + halfSize) {
              b.position[axis] = -this.bounds[axis]/2 + halfSize;
              b.velocity[axis] = -b.velocity[axis]*b.restitution;
            }
            if(b.position[axis] > this.bounds[axis]/2 - halfSize) {
              b.position[axis] = this.bounds[axis]/2 - halfSize;
              b.velocity[axis] = -b.velocity[axis]*b.restitution;
            }
        }
         if(b.position.y > this.bounds.y - halfSize) {
              b.position.y = this.bounds.y - halfSize;
              b.velocity.y = -b.velocity.y*b.restitution;
         }
    }

    // Object collisions (only within same grid cell)
    for(const cell in grid) {
      let cellBodies = grid[cell];
      for(let i=0;i<cellBodies.length;i++) {
        let a=cellBodies[i];
        for(let j=i+1;j<cellBodies.length;j++) {
          let b=cellBodies[j];
          if(a.shape.type===SHAPE_TYPE.SPHERE && b.shape.type===SHAPE_TYPE.SPHERE) {
            let d = a.position.sub(b.position);
            let dist = d.length();
            let minDist = a.shape.radius + b.shape.radius;
            if(dist < minDist && dist>0) {
              let norm = d.normalize();
              let penetration = minDist-dist;
              let relVel = a.velocity.sub(b.velocity).dot(norm);
              let impulse = (-(1+Math.min(a.restitution,b.restitution))*relVel)/
                            (a.invMass+b.invMass);
              let impulseVec = norm.mul(impulse);
              a.velocity = a.velocity.add(impulseVec.mul(a.invMass));
              b.velocity = b.velocity.sub(impulseVec.mul(b.invMass));
              a.position = a.position.add(norm.mul(penetration*a.invMass/(a.invMass+b.invMass)));
              b.position = b.position.sub(norm.mul(penetration*b.invMass/(a.invMass+b.invMass)));
              this.collisionCount++;
            }
          } else if(a.shape.type===SHAPE_TYPE.BOX && b.shape.type===SHAPE_TYPE.BOX) {
            if(boxCollides(a, b)) {
              let norm = a.position.sub(b.position).normalize();
              let relVel = a.velocity.sub(b.velocity).dot(norm);
              let impulse = (-(1+Math.min(a.restitution,b.restitution))*relVel)/
                            (a.invMass+b.invMass);
              let impulseVec = norm.mul(impulse);
              a.velocity = a.velocity.add(impulseVec.mul(a.invMass));
              b.velocity = b.velocity.sub(impulseVec.mul(b.invMass));
              a.position = a.position.add(norm.mul(0.05*a.invMass/(a.invMass+b.invMass)));
              b.position = b.position.sub(norm.mul(0.05*b.invMass/(a.invMass+b.invMass)));
              this.collisionCount++;
            }
          } else if(a.shape.type===SHAPE_TYPE.CYLINDER && b.shape.type===SHAPE_TYPE.CYLINDER) {
            if(cylinderCollides(a, b)) {
              let norm = a.position.sub(b.position).normalize();
              let relVel = a.velocity.sub(b.velocity).dot(norm);
              let impulse = (-(1+Math.min(a.restitution,b.restitution))*relVel)/
                            (a.invMass+b.invMass);
              let impulseVec = norm.mul(impulse);
              a.velocity = a.velocity.add(impulseVec.mul(a.invMass));
              b.velocity = b.velocity.sub(impulseVec.mul(b.invMass));
              a.position = a.position.add(norm.mul(0.05*a.invMass/(a.invMass+b.invMass)));
              b.position = b.position.sub(norm.mul(0.05*b.invMass/(a.invMass+b.invMass)));
              this.collisionCount++;
            }
          }
        }
      }
    }
  }
}

function boxCollides(a, b) {
  let sa = a.shape.size, sb = b.shape.size;
  return Math.abs(a.position.x-b.position.x) < (sa+sb)/2 &&
         Math.abs(a.position.y-b.position.y) < (sa+sb)/2 &&
         Math.abs(a.position.z-b.position.z) < (sa+sb)/2;
}
function cylinderCollides(a, b) {
  let dx = a.position.x - b.position.x;
  let dz = a.position.z - b.position.z;
  let dist = Math.sqrt(dx*dx + dz*dz);
  let minDist = a.shape.radius + b.shape.radius;
  let yOverlap = Math.abs(a.position.y-b.position.y) < (a.shape.height+b.shape.height)/2;
  return dist < minDist && yOverlap;
}

// --- Rendering & UI ---
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x181818);
const camera = new THREE.PerspectiveCamera(75, window.innerWidth/window.innerHeight, 0.1, 1000);
camera.position.set(0,8,16);

const renderer = new THREE.WebGLRenderer({antialias:true});
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.05;

// Ground plane
const groundGeo = new THREE.BoxGeometry(20, 0.4, 20);
const groundMat = new THREE.MeshStandardMaterial({color:0x222a33});
const ground = new THREE.Mesh(groundGeo, groundMat);
ground.position.y = -0.2;
ground.receiveShadow = true;
scene.add(ground);

// World bounding box
const boundsBox = new THREE.BoxHelper(new THREE.Mesh(
  new THREE.BoxGeometry(20, 20, 20)
), 0x00bfff);
boundsBox.position.set(0, 10, 0); // Centered at y=10
scene.add(boundsBox);

// Lights
const ambient = new THREE.AmbientLight(0xffffff, 0.55);
scene.add(ambient);
const dirLight = new THREE.DirectionalLight(0xffffff, 0.7);
dirLight.position.set(9, 20, 8);
dirLight.castShadow = true;
scene.add(dirLight);

// Physics setup
const world = new PhysicsWorld();

// Spawn shapes menu
const spawnShapes = [
  {name:'Sphere', type:SHAPE_TYPE.SPHERE},
  {name:'Box', type:SHAPE_TYPE.BOX},
  {name:'Cylinder', type:SHAPE_TYPE.CYLINDER}
];
const spawnShapeSelect = document.getElementById('spawn-shape');
function updateSpawnShapeOptions() {
  spawnShapeSelect.innerHTML = '';
  for(const s of spawnShapes) {
    let opt = document.createElement('option');
    opt.value = s.name;
    opt.textContent = s.name;
    spawnShapeSelect.appendChild(opt);
  }
}
updateSpawnShapeOptions();

// File/model loading
const gltfLoader = new GLTFLoader();
const loadedModels = [];
document.getElementById('model-input').addEventListener('change', e => {
  for(const file of e.target.files) {
    const url = URL.createObjectURL(file);
    gltfLoader.load(url, gltf => {
      loadedModels.push(gltf.scene);
      const li = document.createElement('li');
      li.textContent = file.name;
      document.getElementById('file-list').appendChild(li);
    });
  }
});

function spawnBody(shapeName, position = new Vec3(0,5,0)) {
  let shapeDef = spawnShapes.find(s => s.name===shapeName);
  if(!shapeDef) return;
  let mesh, shape, mass;
  if(shapeDef.type===SHAPE_TYPE.SPHERE) {
    let radius = 0.4 + Math.random()*0.3;
    shape = new SphereShape(radius);
    mesh = new THREE.Mesh(
      new THREE.SphereGeometry(radius, 24, 16),
      new THREE.MeshStandardMaterial({color:0x00bfff})
    );
    mass = 1;
  } else if(shapeDef.type===SHAPE_TYPE.BOX) {
    let size = 0.4 + Math.random()*0.4;
    shape = new BoxShape(size);
    mesh = new THREE.Mesh(
      new THREE.BoxGeometry(size, size, size),
      new THREE.MeshStandardMaterial({color:0xffa600})
    );
    mass = 1.5;
  } else if(shapeDef.type===SHAPE_TYPE.CYLINDER) {
    let radius = 0.3 + Math.random()*0.3, height = 0.8 + Math.random()*0.8;
    shape = new CylinderShape(radius, height);
    mesh = new THREE.Mesh(
      new THREE.CylinderGeometry(radius, radius, height, 20),
      new THREE.MeshStandardMaterial({color:0x2ecc40})
    );
    mass = 2;
  }
  mesh.position.set(position.x, position.y, position.z);
  mesh.castShadow = true;
  scene.add(mesh);
  let rb = new RigidBody(position, shape, mass, mesh);
  rb.angularVelocity = new Vec3(Math.random()*2-1, Math.random()*2-1, Math.random()*2-1);
  rb.friction = Number(document.getElementById('friction-slider').value);
  rb.restitution = Number(document.getElementById('restitution-slider').value);
  world.addBody(rb);
  return rb;
}

// --- UI Event Listeners ---
let gravitySlider = document.getElementById('gravity-slider');
gravitySlider.addEventListener('input', () => world.gravity.y = Number(gravitySlider.value));

let timescaleSlider = document.getElementById('timescale-slider');
let timeScale = 1;
timescaleSlider.addEventListener('input', () => timeScale = Number(timescaleSlider.value));

document.getElementById('shoot-btn').addEventListener('click', () => {
  let newBody = spawnBody(spawnShapeSelect.value, new Vec3(0, 5, 0));
  if(newBody) newBody.velocity = new Vec3(0,0,-12);
});

document.getElementById('reset-btn').addEventListener('click', () => {
  world.bodies.forEach(b => scene.remove(b.mesh));
  world.bodies = [];
  selectedBody = null;
  updateObjectInfo();
});

let paused = false;
document.getElementById('pause-btn').onclick = () => {
  paused = true;
  document.getElementById('pause-btn').style.display='none';
  document.getElementById('resume-btn').style.display='inline-block';
};
document.getElementById('resume-btn').onclick = () => {
  paused = false;
  document.getElementById('pause-btn').style.display='inline-block';
  document.getElementById('resume-btn').style.display='none';
};

document.getElementById('cam-front-btn').onclick = () => { camera.position.set(0,8,16); controls.target.set(0,0,0); };
document.getElementById('cam-top-btn').onclick = () => { camera.position.set(0,20,0.1); controls.target.set(0,0,0); };
document.getElementById('cam-side-btn').onclick = () => { camera.position.set(16,8,0); controls.target.set(0,0,0); };

document.getElementById('reset-object-btn').onclick = () => {
  if(selectedBody) {
    selectedBody.position = new Vec3(0,5,0);
    selectedBody.velocity = Vec3.zero();
    selectedBody.angularVelocity = Vec3.zero();
    selectedBody.mesh.position.set(0,5,0);
    selectedBody.mesh.rotation.set(0,0,0);
    updateObjectInfo();
  }
};

// --- Save/Load (unchanged) ---
document.getElementById('save-btn').addEventListener('click', () => {
  let data = world.bodies.map(b => ({
    type: b.shape.type, position: [b.position.x, b.position.y, b.position.z], velocity: [b.velocity.x, b.velocity.y, b.velocity.z],
    angularVelocity: [b.angularVelocity.x, b.angularVelocity.y, b.angularVelocity.z], mass: b.mass, friction: b.friction, restitution: b.restitution,
    size: b.shape.size || b.shape.radius, height: b.shape.height, color: b.mesh.material.color.getHex()
  }));
  let blob = new Blob([JSON.stringify(data)], {type:'application/json'});
  let a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = 'sandbox-scene.json'; a.click();
});

document.getElementById('load-btn').addEventListener('click', () => {
    // This is a placeholder for the original load functionality.
    // The original `loadScene` function was removed, you might want to re-add it if needed.
    alert("Load scene functionality needs to be connected.");
});

document.getElementById('import-btn').onclick = () => document.getElementById('import-scene-input').click();
document.getElementById('import-scene-input').addEventListener('change', e => {
      let file = e.target.files[0];
      if(file) {
        let reader = new FileReader();
        reader.onload = () => {
          let data = JSON.parse(reader.result);
          world.bodies.forEach(b => scene.remove(b.mesh));
          world.bodies = [];
          for(const obj of data) {
            let mesh, shape;
            if(obj.type===SHAPE_TYPE.SPHERE) {
              shape = new SphereShape(obj.size);
              mesh = new THREE.Mesh(
                new THREE.SphereGeometry(obj.size, 24, 16),
                new THREE.MeshStandardMaterial({color:obj.color})
              );
            } else if(obj.type===SHAPE_TYPE.BOX) {
              shape = new BoxShape(obj.size);
              mesh = new THREE.Mesh(
                new THREE.BoxGeometry(obj.size, obj.size, obj.size),
                new THREE.MeshStandardMaterial({color:obj.color})
              );
            } else if(obj.type===SHAPE_TYPE.CYLINDER) {
              shape = new CylinderShape(obj.size, obj.height || 2);
              mesh = new THREE.Mesh(
                new THREE.CylinderGeometry(obj.size, obj.size, obj.height||2, 20),
                new THREE.MeshStandardMaterial({color:obj.color})
              );
            }
            mesh.position.set(...obj.position);
            mesh.castShadow = true;
            scene.add(mesh);
            let rb = new RigidBody(new Vec3(...obj.position), shape, obj.mass, mesh);
            rb.velocity = new Vec3(...obj.velocity);
            rb.angularVelocity = new Vec3(...obj.angularVelocity);
            rb.friction = obj.friction;
            rb.restitution = obj.restitution;
            world.addBody(rb);
          }
          alert('Scene imported!');
        };
        reader.readAsText(file);
      }
});


// --- Material Sliders (unchanged) ---
document.getElementById('friction-slider').addEventListener('input', e => { if(selectedBody) selectedBody.friction = Number(e.target.value); updateObjectInfo(); });
document.getElementById('restitution-slider').addEventListener('input', e => { if(selectedBody) selectedBody.restitution = Number(e.target.value); updateObjectInfo(); });

// --- User Interaction Logic ---
let selectedBody = null;
let draggingBody = null;
let paintMode = false;
let isPainting = false;
let lastPaintPoint = new THREE.Vector3();
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();

// Invisible plane for painting on
const paintPlane = new THREE.Mesh(
  new THREE.PlaneGeometry(100, 100),
  new THREE.MeshBasicMaterial({ visible: false, side: THREE.DoubleSide })
);
paintPlane.rotateX(-Math.PI / 2); // Align with ground
scene.add(paintPlane);

document.getElementById('paint-btn').addEventListener('click', () => {
    paintMode = !paintMode;
    document.getElementById('paint-btn').classList.toggle('active', paintMode);
    controls.enabled = !paintMode; // Disable camera controls in paint mode
});

function getMouseIntersection() {
  raycaster.setFromCamera(mouse, camera);
  const intersects = raycaster.intersectObject(paintPlane);
  return intersects.length > 0 ? intersects[0].point : null;
}

renderer.domElement.addEventListener('pointerdown', e => {
  mouse.set((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  
  if (paintMode) {
    isPainting = true;
    controls.enabled = false;
    const point = getMouseIntersection();
    if (point) {
      spawnBody(spawnShapeSelect.value, new Vec3(point.x, point.y + 1, point.z));
      lastPaintPoint.copy(point);
    }
  } else {
    // Object Selection
    raycaster.setFromCamera(mouse, camera);
    const intersects = raycaster.intersectObjects(scene.children, true);
    if(selectedBody?.mesh) selectedBody.mesh.material.emissive?.setHex(0x000000);
    selectedBody = null;

    if(intersects.length) {
      const mesh = intersects[0].object;
      selectedBody = world.bodies.find(b => b.mesh === mesh);
      if(selectedBody) {
        selectedBody.mesh.material.emissive?.setHex(0x00ff40);
        draggingBody = selectedBody; // Allow dragging selected object
        controls.enabled = false;
      }
    }
    updateObjectInfo();
  }
});

renderer.domElement.addEventListener('pointermove', e => {
  mouse.set((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  
  if (paintMode && isPainting) {
    const point = getMouseIntersection();
    if (point && point.distanceTo(lastPaintPoint) > 1.0) { // Only paint if mouse moved enough
      spawnBody(spawnShapeSelect.value, new Vec3(point.x, point.y + 1, point.z));
      lastPaintPoint.copy(point);
    }
  } else if (draggingBody) {
    const point = getMouseIntersection();
    if (point) {
      draggingBody.position.x = point.x;
      draggingBody.position.z = point.z; // Drag along the plane
      draggingBody.velocity = Vec3.zero();
    }
  }
});

renderer.domElement.addEventListener('pointerup', e => {
  isPainting = false;
  draggingBody = null;
  if (!paintMode) {
    controls.enabled = true;
  }
});

function updateObjectInfo() {
  const infoDiv = document.getElementById('object-info');
  if(selectedBody) {
    infoDiv.innerHTML = `
      <b>Selected Object</b><br>
      Type: ${Object.keys(SHAPE_TYPE).find(k=>SHAPE_TYPE[k]===selectedBody.shape.type)}<br>
      Mass: ${selectedBody.mass.toFixed(2)}<br>
      Pos: (${selectedBody.position.x.toFixed(2)}, ${selectedBody.position.y.toFixed(2)}, ${selectedBody.position.z.toFixed(2)})<br>
      Velocity: ${selectedBody.velocity.length().toFixed(2)}<br>
      Friction: ${selectedBody.friction}<br>
      Restitution: ${selectedBody.restitution}<br>
    `;
  } else {
    infoDiv.innerHTML = '';
  }
}

// --- Simulation Loop & Stats ---
let lastTime = performance.now(), frameCount = 0, lastStatsTime = performance.now();
function animate() {
  requestAnimationFrame(animate);
  if(paused) { renderer.render(scene, camera); return; }
  let now = performance.now();
  let dt = Math.min(0.05, (now-lastTime)/1000) * timeScale; // Capped dt for stability
  lastTime = now;
  world.update(dt);
  controls.update();
  renderer.render(scene, camera);
  frameCount++;
  if(now-lastStatsTime > 1000) {
    document.getElementById('stats').textContent =
      `FPS: ${frameCount} | Bodies: ${world.bodies.length} | Collisions: ${world.collisionCount} | Gravity: ${world.gravity.y} | TimeScale: ${timeScale.toFixed(2)}`;
    frameCount = 0;
    lastStatsTime = now;
  }
  if(selectedBody) updateObjectInfo(); // Live update info for selected body
}

// --- UI Sidebar/Overlay ---
document.getElementById('start-button').onclick = () => {
  document.getElementById('loading-overlay').style.display='none';
  document.getElementById('sidebar').style.display='block';
  document.getElementById('toggle-sidebar').style.display='block';
  if (window.innerWidth > 768) document.body.classList.add('sidebar-visible');
  animate();
};
document.getElementById('toggle-sidebar').onclick = () => document.body.classList.toggle('sidebar-visible');

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
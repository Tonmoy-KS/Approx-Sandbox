# Approx Sandbox - 3D Physics Engine

_.**Approx Sandbox** is a modern, browser-based 3D physics playground built with [three.js](https://threejs.org/) and a custom physics engine. Simulate, visualize, and interact with spheres, boxes, cylinders, and custom models in real-time._

## Features

- **Physics Engine**:  
  Realistic rigid-body physics supporting spheres, boxes, cylinders, custom shapes, gravity, friction, restitution, and angular velocity.

- **3D Rendering**:  
  High-quality visuals with lighting, ground, bounding box, and object highlighting.

- **User Interface**:  
  Sidebar controls for spawning objects, adjusting gravity, time scale, material properties, camera views, simulation state (pause/resume), and more.

- **Interaction**:  
  Select, drag, reset, and shoot objects. Manipulate objects using mouse or touch.

- **Scene Management**:  
  Save/load scene state, export/import JSON, and drag & drop GLB/GLTF models.

- **Extensible**:  
  Easily add new shapes, physics features, or custom controls.

## Usage

1. **Open in Browser**  
   Just open `src/approx.html` in a modern browser.

2. **Controls**
   - **Spawn Shape**: Choose object type and spawn into the scene.
   - **Gravity/Time Scale**: Adjust simulation realism.
   - **Friction/Restitution**: Tune material properties.
   - **Shoot**: Launch object with velocity.
   - **Pause/Resume**: Control simulation state.
   - **Camera Presets**: Quick navigation views.
   - **Reset**: Clear simulation or selected object.
   - **Save/Load**: Save to browser or export/import as JSON.

3. **Interaction**
   - **Select Object**: Click an object to view and edit properties.
   - **Drag**: Click and drag selected object to move it.
   - **Import Model**: Drag & drop GLB/GLTF files or use file input.

## Development

- **Tech Stack**:  
  - [three.js](https://threejs.org/) for 3D rendering  
  - Pure JavaScript for physics and UI

You can extend the physics engine by adding more shape classes, better collision detection, or advanced controls. The code is modular and well-commented for easy modification.

## License

MIT

## Credits

Developed by [Tonmoy-KS](https://github.com/Tonmoy-KS)
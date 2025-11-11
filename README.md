

# 🧲 Magnetic Newton’s Cradle

A real-time **physics simulation** built with **Pygame** that visualizes a hybrid between a **Newton’s cradle** and **magnetic pendulum system**.  
It demonstrates pendulum motion, magnetic repulsion, and collision dynamics — complete with optional 5-minute frame capture for video export.

---

## 🚀 Features
- Real-time pendulum simulation with gravity and damping  
- Magnetic force interactions between bobs  
- Elastic collisions (no overlaps)  
- Click-and-drag controls to swing individual bobs  
- 5-minute automatic frame capture for high-quality video rendering  
- Customizable constants (gravity, magnet strength, damping, FPS, etc.)

---

## ⚙️ How It Works
Each bob is modeled as a pendulum constrained by its rope:
- **Gravity** pulls it downward.
- **Magnetic repulsion** (∝ 1/r²) pushes it horizontally from nearby bobs.
- **Collisions** between neighboring bobs exchange angular velocity, giving the signature “clack” motion.

A semi-implicit Euler integrator keeps the simulation stable even at high FPS.

---

## 🖱️ Controls
| Action | Effect |
|--------|---------|
| **Left Click + Drag** | Pull a bob to a new position |
| **Release Mouse** | Let it swing |
| **Close Window** | End simulation (and frame capture) |

---

## 🎥 Recording
When `RECORD_VIDEO = True` in `main.py`, the program automatically:
- Saves each rendered frame as `frames/frame_00000.png`, `frame_00001.png`, etc.
- Stops after 5 minutes (`VIDEO_DURATION = 300`).

You can convert the frames into a video using **FFmpeg**:
```bash
ffmpeg -framerate 120 -i frames/frame_%05d.png -c:v libx264 -pix_fmt yuv420p magnetic_cradle.mp4
```

---

## 🧩 Dependencies
Install dependencies before running:
```bash
pip install pygame
```

---

## ▶️ Run It
```bash
python main.py
```

---

## 🧮 Adjustable Parameters
You can tweak constants at the top of `main.py`:

| Variable | Description | Default |
|-----------|--------------|----------|
| `NUM_BOBS` | Number of pendulums | 5 |
| `ROPE_LENGTH` | Length of each pendulum | 260 px |
| `MAGNET_K` | Magnetic strength constant | 2e5 |
| `DAMPING` | Angular damping factor | 0.9995 |
| `VIDEO_DURATION` | Capture time (seconds) | 300 |
| `FPS` | Frame rate | 120 |

---

## 🧠 Concepts Demonstrated
- Pendulum motion and energy conservation  
- Magnetic field interaction (simplified dipole model)  
- Elastic collisions in constrained systems  
- Real-time numerical integration and rendering

---

## 📸 Example Output
A typical 5-minute simulation produces a mesmerizing dance of magnetic pendulums.  
Use FFmpeg to turn the generated frames into a cinematic clip.

---

## 📁 Project Structure
```
📦 Magnetic Newton's Cradle
 ┣ 📜 main.py              # Main simulation script
 ┣ 📜 magnets.py           # Magnetic force utilities
 ┣ 📜 README.md            # Documentation
 ┣ 📂 frames/              # Captured PNG frames
 ┗ 📜 requirements.txt     # (Optional) dependencies list
```
---

## 🧾 License
This project is open-source under the MIT License.
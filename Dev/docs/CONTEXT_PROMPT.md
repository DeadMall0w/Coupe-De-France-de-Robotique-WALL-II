# WALL-II — Project Context Prompt

> **Purpose**: Copy-paste this document (or relevant sections) at the start of any new AI chat session so the assistant understands the full project context immediately.

---

## 1. What is this project?

Embedded C++17 software for **WALL-II**, a fully autonomous robot competing in the **Coupe de France de Robotique**. The robot must navigate a 300 × 200 cm game board, detect its environment via LIDAR, pick up and deposit game elements ("cleats"/palets), and avoid the opponent robot — all within a **100-second match**.

The codebase lives under two top-level folders:
- `Dev/` — main production code (compiled and deployed on the robot)
- `Tests/` — standalone test programs (LIDAR visual debug, unit stubs)

---

## 2. Hardware

| Category | Component | Qty | Notes |
|---|---|---|---|
| **Microcontrollers** | Raspberry Pi 5 | 1 | Main brain, runs the C++ program |
| | Teensy 4.1 | 2 | Low-level motor/sensor control |
| **Power** | LiPo 11.1 V 4000 mAh | 1 | Main battery |
| | LiPo 11.1 V 3200 mAh | 2 | Spares (fireproof bag!) |
| | Buck converters 5 V 5 A (D24V50F5) | 6 | |
| | Level shifters 3.3 V → 5 V | 3 | Pi ↔ Teensy/servos |
| **Sensors** | LIDAR Slamtec C1M1 | 1 | 360° scan, serial USB, 115200 baud, ~10 Hz |
| | SparkFun optical odometry sensor | 1 | Robot self-localization |
| | Start lever (switch) | 1 | Triggers match start |
| | Emergency stop button | 1 | Physical kill switch |
| | Pi Camera 5 | 1 | Vision / object recognition (not yet integrated in code) |
| **Actuators** | Motoréducteur + encoder 4844 | 4 | Base drive motors (holonomic) |
| | Cytron motor drivers | 2 | Drive the 4 base motors |
| | Servomoteur FB5118M | 4 | Arm/gripper |
| | SG90 micro-servo | 4 | Auxiliary mechanisms |
| | Upper motors | 2 | Upper mechanism |
| **Mobility** | Holonomic wheels (omni) | 4 | Allows movement in any direction |
| | Free wheels (casters) | 2 | Support |
| **Structure** | PLA filament, MakerBeam, laser-cut wood | — | Chassis |

**Communication chain**: Raspberry Pi 5 (C++ program) → USB serial → Teensy 4.1 (motor/servo control). LIDAR connects via USB serial (`/dev/ttyUSB0`). Start lever and emergency stop are digital inputs.

---

## 3. Software Architecture Overview

```
main.cpp                 — Program entry, thread creation, match timing (100 s), emergency stop
  ├── Board (singleton)  — Centralized thread-safe game state (board.cpp / Board.hpp)
  ├── vision thread      — LIDAR acquisition + processing (vision.cpp)
  │     ├── Slamtec      — LIDAR driver, implements ILidar interface (Slamtec.cpp)
  │     └── LidarProcessing — Scan pipeline: blind spots → polar→absolute → filter map → cluster → enemy position (LidarProcessing.cpp)
  ├── strategy thread    — Decision-making (strategy.cpp) ⚠️ NOT IMPLEMENTED YET
  └── UnityDebug         — Board → JSON → Unity visualization (UnityDebug.cpp) ⚠️ STUB
```

### Threading model
- **Main thread**: match timer loop (1 ms sleep), checks emergency stop.
- **Vision thread** (`std::thread`): blocking `grabData()` loop at ~10 Hz.
- **Input thread**: waits for Enter key (simulates emergency stop).
- **Strategy thread**: planned but not created yet.
- All shared state goes through the `Board` singleton, protected by a single `std::mutex`.

---

## 4. File Map (Dev/)

### Source files (`src/`)
| File | Role | Status |
|---|---|---|
| `main.cpp` | Entry point, thread orchestration, timing | ✅ Working |
| `board.cpp` | Board singleton implementation, JSON config loading | ✅ Working |
| `vision.cpp` | LIDAR thread: init → scan loop → LidarProcessing pipeline | ✅ Working |
| `Slamtec.cpp` | ILidar implementation for Slamtec C1M1 (connect/scan/grab/disconnect) | ✅ Working |
| `LidarProcessing.cpp` | Full scan processing pipeline (5 stages) | ✅ Working |
| `strategy.cpp` | Game strategy / decision engine | ❌ Empty |
| `UnityDebug.cpp` | Export board state to Unity | ❌ Stub |

### Headers (`includes/`)
| File | Content |
|---|---|
| `Board.hpp` | All data structures: `Position`, `Size`, `Robot`, `Zone`, `Map`, `Cleat`, enums (`Team`, `GameState`, `ZoneState`), `Board` class |
| `ILidar.h` | Abstract LIDAR interface + `ScanPoint` struct + `LidarUtils::writeScanToCSV()` |
| `Slamtec.h` | Slamtec driver class (inherits ILidar) |
| `LidarProcessing.h` | `AbsolutePoint`, `Cluster`, `LidarProcessingResult`, `LidarProcessing` class |
| `Constant.h` | All project constants (map dimensions, LIDAR params, clustering params, blind spots) |
| `vision.h` | `void vision(std::atomic<bool>* stop)` declaration |
| `color.h` | ANSI color macros for terminal output |

### Config (`data/`)
| File | Content |
|---|---|
| `config.json` | Full game configuration: team, robot positions/sizes, storage zones (7), deposit zones (7), nid orange/blue |

### External libs (`lib/`)
- `lib/bin/libsl_lidar_sdk.a` — Static LIDAR SDK library
- `lib/include/` — Slamtec RPLidar/SL Lidar SDK headers
- `lib/include/json.hpp` — nlohmann/json (single-header JSON library)

### Build
- `makefile` — `make official` (optimized -O2), `make clean`, `make distclean`
- Compiler: `g++ -std=c++17`
- Output: `build/bin/wallII`

---

## 5. Key Data Structures (Board.hpp)

```cpp
struct Position { double x; double y; };               // cm, origin = bottom-left of board
struct Size     { double width; double length; };       // cm
struct Cleat    { Team color; };                        // game element
struct Zone     { Position position; Size size; ZoneState state; };

struct Robot {
    Position position;
    double orientation_deg = 0.0;   // 0° = X+ axis, trigonometric direction
    Size size;
    std::array<Cleat, 4> cleatsHeld;  // max 4 palets
};

struct Map {
    std::array<Zone, 8>  storagesZones;   // pick-up zones
    std::array<Zone, 10> depositsZones;   // drop-off zones
    Zone nid;                             // team nest
};

enum class Team      { Orange, Blue };
enum class GameState { Waiting, Started, Finished };
enum class ZoneState { Empty, Filled, UsedByMe, UsedByEnemy };
```

The `Board` class is a **singleton** (`Board::instance()`) with:
- **Getters**: `getMyRobot()`, `getEnemyRobot()`, `getMap()`, `getTimeLeft()`, `getState()`, `getTeam()`, `getMyRobotOrientation()`, `getNid()`
- **Setters**: `moveMyRobot()`, `moveEnemyRobot()`, `setMyRobotOrientation()`, `setState()`, `setTeam()`, `setTimeLeft()`, `setStorageZoneState()`, `setDepositZoneState()`, `setNidState()`, `setMyRobotCleat()`, `setEnemyRobotCleat()`
- **All methods** lock a `std::mutex` for thread safety.
- **`initialiseData(path)`** reads `config.json` and populates everything.

---

## 6. LIDAR Processing Pipeline (LidarProcessing.cpp)

Each scan (~10 Hz, up to 8192 raw points) goes through:

```
① filterBlindSpots()    — Remove points in known blind angles (4 configurable sectors)
② polarToAbsolute()     — Convert (angle_deg, dist_mm) relative to robot → (x_cm, y_cm) absolute on board
                           Formula: absolute_angle = robot_orientation + lidar_angle
                                    x = robot_x + dist_cm * cos(absolute_angle)
                                    y = robot_y + dist_cm * sin(absolute_angle)
③ isInsideMap()         — Keep only points within [0, 300] × [0, 200] cm
④ clusterPoints()       — BFS-based clustering (gap < CLUSTER_MAX_GAP_CM), return largest cluster
⑤ checkZoneVisits()     — If enemy centroid is within ZONE_PROXIMITY_CM of any zone center → mark UsedByEnemy
```

Result struct: `LidarProcessingResult { bool enemyDetected, AbsolutePoint enemyPosition, int pointsInMap, int pointsFiltered, int clusterSize }`

---

## 7. Key Constants (Constant.h)

```cpp
GAME_DURATION_MS      = 100'000      // 100 seconds match
MAP_WIDTH_CM          = 300.0        // board X dimension
MAP_HEIGHT_CM         = 200.0        // board Y dimension
BORDER_HEIGHT_CM      = 7.0          // physical border height
LIDAR_HEIGHT_CM       = 17.0         // LIDAR mounted ~17cm high (above borders)
ENEMY_ROBOT_RADIUS_CM = 25.0         // approximate enemy robot radius
ZONE_PROXIMITY_CM     = 20.0         // distance to consider enemy "in" a zone
CLUSTER_MAX_GAP_CM    = 30.0         // max inter-point distance within a cluster
MIN_CLUSTER_POINTS    = 3            // min points for a valid cluster
LIDAR_BLIND_SPOTS     = {{40-50°}, {130-140°}, {220-230°}, {310-320°}}  // 4 sectors, to calibrate
```

LIDAR driver constants (Slamtec.h): `BAUDRATE = 115200`, `MAX_NODES = 8192`, `SCAN_TIMEOUT = 0.5s`

---

## 8. Game Board Layout (from config.json)

- **Board**: 300 × 200 cm, coordinates in cm, origin = bottom-left
- **Teams**: Orange (left side, nid at x=0..60, y=155..200) and Blue (right side, nid at x=240..300, y=155..200)
- **7 storage zones** (pick-up): scattered across the board, initially `Filled`
- **7 deposit zones** (drop-off): along the middle area, initially `Empty`
- **Robot size**: 30 × 30 cm
- Team is configurable in `config.json` (`"team": "Orange"` or `"Blue"`)

---

## 9. Tests (Tests/)

| Path | Description |
|---|---|
| `Tests/test.cpp` | Minimal stub: Slamtec constructor test (no real LIDAR) |
| `Tests/lidar_visual/` | Full visual LIDAR debug tool |
| `Tests/lidar_visual/test_lidar_visual.cpp` | Connects to real LIDAR, runs pipeline, exports detailed JSON every scan |
| `Tests/lidar_visual/lidar_viewer.html` | Browser-based visualization: canvas showing board, points, enemy, zones (auto-refreshes JSON) |
| `Tests/lidar_visual/makefile` | `make` / `make run ARGS="x y angle"` / `make serve` (HTTP server on port 8089) |

Also: `Dev/test.html` and `Dev/lidar_data.csv` for basic LIDAR data visualization.

---

## 10. Code Conventions

- **Language**: C++17, comments and user-facing strings in **French**
- **Naming**: Classes `PascalCase`, functions `camelCase`, constants `UPPER_SNAKE_CASE`, enums `PascalCase`
- **Terminal output**: Color-coded via `color.h` — RED=errors, GREEN=success, YELLOW=info, BOLDBLUE=important, MAGENTA=user prompts
- **TODO markers**: `//todo:` or `//Todo:` in comments
- **JSON library**: nlohmann/json (`lib/include/json.hpp`)
- **Pattern**: Singleton for Board, Interface/Implementation for LIDAR (`ILidar` → `Slamtec`)

---

## 11. Current State & Known TODOs

### ✅ Implemented
- Board singleton with full JSON config loading
- LIDAR driver (Slamtec C1M1) with connect/scan/grab/disconnect
- Complete LIDAR processing pipeline (blind spots → absolute coords → map filter → BFS clustering → enemy detection → zone visit tracking)
- Vision thread integration
- Match timing with emergency stop
- Visual debug tool (LIDAR viewer HTML + JSON export)

### ⚠️ TODO (priority order)
1. **Strategy module** (`strategy.cpp`) — entirely empty, needs: pathfinding, action decisions, objective prioritization
2. **Motor control / Teensy communication** — no serial communication code to Teensy 4.1 yet
3. **Start lever integration** — `main.cpp` has a TODO to wait for physical start sensor before beginning match
4. **Odometry integration** — SparkFun optical sensor not yet integrated for self-localization
5. **Camera integration** — Pi Camera 5 not used in code yet
6. **Unity debug** (`UnityDebug.cpp`) — stub, board→JSON→Unity not implemented
7. **Holonomic movement** — 4 omni wheels, kinematics not implemented in software
8. **Servo/gripper control** — FB5118M and SG90 servos not controlled from the main program yet
9. **Game state management** — transitions Waiting→Started→Finished partially wired
10. **Robustness** — error handling, logging, match replay

### ⚠️ Known Issues
- Blind spot angles in `Constant.h` are placeholder values — must be calibrated on the real robot
- `config.json` has a storage zone with `y: 300` which is outside the 200cm board height — likely a data entry error
- `start_map.json` exists but is empty (unused)

---

## 12. How to Build & Run

```bash
# From Dev/
make official          # → build/bin/wallII (optimized)
make clean             # remove object files

# Run (needs LIDAR on /dev/ttyUSB0)
./build/bin/wallII

# Visual LIDAR test (from Tests/lidar_visual/)
make                   # compile
make run ARGS="50 100 0"   # run with robot at (50,100) facing 0°
make serve             # HTTP server on port 8089 → open lidar_viewer.html
```

---

## 13. Tips for AI Assistants

- **Always read the relevant header** before modifying a `.cpp` file — the struct/class definitions are in `includes/`.
- **Thread safety**: any new data added to `Board` must be accessed through mutex-locked getters/setters.
- **Units**: positions are in **cm**, LIDAR raw data is in **mm** (converted in pipeline), angles in **degrees**.
- **Config**: game parameters come from `data/config.json` — new parameters should be added there and loaded in `Board::initialiseData()`.
- **New modules**: follow the existing pattern — header in `includes/`, implementation in `src/`, function launched in a thread from `main.cpp`.
- **The robot uses holonomic (omni) wheels** — it can translate in any direction without rotating. Keep this in mind for pathfinding/movement code.
- **Two Teensy 4.1** handle low-level motor/servo control — the Pi communicates with them via serial. This communication layer does not exist in code yet and is a critical TODO.

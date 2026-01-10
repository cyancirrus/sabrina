# Sabrina

**Sabrina** is a lightweight Rust-based simulation for grid-based environment exploration and navigation. It uses a simple LIDAR-like sensor to scan surroundings and an A*-based planner to navigate a virtual environment.

## Features

* Parse 2D maps with walls, obstacles, corners, and doorways.
* Represent environments with a flexible `Environment` struct.
* Simulate LIDAR measurements in 4 principal directions.
* Plan paths using a basic A* algorithm.
* Stepwise navigation that updates the agent’s knowledge of the environment.

## Project Structure

```
sabrina/
├── Cargo.toml             # Rust package configuration
├── Cargo.lock
├── data/
│   └── sample/
│       └── test0.map      # Example map
├── src/
│   ├── main.rs            # Entry point / demo navigation
│   ├── lib.rs
│   ├── environment/       # Environment representation
│   │   ├── mod.rs
│   │   └── representation.rs
│   └── parser/            # Map parsing
│       ├── mod.rs
│       └── map.rs
```

### Core Components

* **Environment & Objects**
  Represents the grid world. Objects include `Wall`, `Doorway`, `Corner`, `Obstacle`, and `Unknown`. The environment tracks objects and dynamic bounds.

* **Lidar**
  Simulates a sensor that can "see" in four cardinal directions up to a configurable maximum range. Returns relative coordinates of detected obstacles.

* **Sabrina**
  The main agent struct, containing its position, environment knowledge, and LIDAR. Can `scan()`, `plan()` paths, and `navigate()` toward a target.

* **Map Parser**
  Reads simple text-based maps (`*.map`) into an `Environment` struct.

## Getting Started

### Prerequisites

* Rust (1.70+ recommended)
* Cargo package manager

### Build and Run

```bash
git clone <repo-url>
cd sabrina
cargo run
```

This will run the example map `data/sample/test0.map` and simulate Sabrina navigating from a start position to a target.

### Map Format

* Maps are simple ASCII grids with 3-character cells:

  * `[ ]` empty space
  * `[#]` wall
  * `[*]` obstacle
  * `[x]` corner
  * `[+]` doorway

Each line in the file corresponds to a row in the environment grid.

## Example Output

```
Final Status Complete
Final map
[ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][?][ ][ ][ ][ ][?][ ][ ][ ][ ]
[ ][ ][ ][ ][ ][?][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
[ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
[?][ ][ ][ ][ ][ ][?][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][?][ ]
[ ][?][ ][ ][?][ ][?][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
[ ][ ][?][?][ ][ ][?][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ]
[?][ ][ ][ ][ ][ ][?][?][?][?][ ][?][?][?][?][ ][?][?][ ][?]
[?][ ][?][?][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][ ][?]
[ ][?][ ][ ][?][?][?][?][?][?][?][?][?][?][?][?][?][?][?][ ]
```

## Next Steps / TODO

* Incorporate orientation and beam rotation for more realistic sensing.
* Implement frontier-based exploration.
* Extend pathing with multiple agents or dynamic environments.

## License

MIT / Apache-2.0 

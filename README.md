# LiDAR Obstacle Detection

> Real-time LiDAR obstacle detection project using PCL (Point Cloud Library)

## Disclaimer

This project has been tested with the following versions on macOS, ONLY:
- **Operating System**: macOS 26.5.2 (25F84)
- **Compiler**: Apple Clang 21.0.0 (clang-2100.1.1.101)
- **CMake**: 4.4.0
- **PCL**: 1.15.1

## Dependencies

The following packages can be installed via the brew-too and are required to build and run this project:

### Core Dependencies
| Package | Version | Purpose |
|---------|---------|---------|
| [CMake](https://cmake.org/) | 4.4+ | Build system |
| [PCL](https://pointclouds.org/) | 1.15+ | Point Cloud Library (requires C++17) |
| [Eigen](https://eigen.tuxfamily.org/) | 3.x | Linear algebra library |
| [Boost](https://www.boost.org/) | 1.8x | C++ utilities (filesystem, etc.) |
| [VTK](https://vtk.org/) | 9.x | Visualization toolkit |

### Additional Dependencies (installed automatically via PCL)
- flann
- qhull
- glew
- libpcap
- libpng
- libusb
- lz4
- cjson
- nlohmann-json
- utf8cpp
- freetype
- libomp
- Qt (qtbase, qtdeclarative, qtsvg)

## Setup

### 1. Install Dependencies (macOS)

```bash
# Install via Homebrew
brew install cmake pcl eigen boost vtk flann qhull glew libpcap libpng libusb lz4
```

### 2. Configure the Project

```bash
mkdir build
cd build
cmake ..
```

The CMake configuration will automatically:
- Detect your CPU cores
- Display the recommended parallel build level
- Configure C++17 standard (required by PCL 1.15)

Example output:
```
-- Detected 10 CPU cores, parallel builds will use 5 jobs
-- Use 'cmake --build .' or 'make -j5' for parallel compilation
```

### 3. Build the Project

#### Sequential Build (default)
```bash
make
```

#### Parallel Build (Recommended)
To significantly speed up compilation, use parallel jobs with half your CPU cores:

```bash
# Use the detected parallel level (e.g., -j5 for 10 cores)
make -j5

# Or let CMake use the configured level
cmake --build . -- -j$(sysctl -n hw.ncpu)
```

### 4. Run the Executables

The build creates three executables:
- `environment` - Main LiDAR obstacle detection application
- `quizCluster` - KD-tree clustering quiz implementation (from the lessons)
- `quizRansac` - RANSAC line fitting quiz implementation (from the lessons)

Run from the build directory:
```bash
./environment
./quizCluster
./quizRansac
```

## Project Structure

```
.
├── CMakeLists.txt          # Main build configuration
├── src/
│   ├── environment.cpp     # Main application
│   ├── processPointClouds.cpp/h  # Point cloud processing
│   ├── render/             # Rendering utilities
│   ├── cluster.cpp        # Quiz: KD-tree clustering
│   ├── kdtree.h            # Quiz: KD-tree implementation
│   ├── ransac2d.cpp        # Quiz: RANSAC 2D line fitting
│   └── sensors/            # Sensor implementations
├── data/                  # Sample point cloud data
└── README.md
```

## Notes

- PCL 1.15 **requires C++17 or higher** - the project is configured accordingly

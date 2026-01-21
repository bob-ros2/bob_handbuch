# ROS Package [bob_flux2](https://github.com/bob-ros2/bob_flux2)

The `bob_flux2` package provides a ROS 2 node for generating images using the **FLUX.2** model. It supports both pure text-to-image (TTI) and image-to-image (ITTI) generation, optimized for memory efficiency by sequentially loading model components.

## Features

- **Memory Efficient**: Loads and unloads the text encoder and transformer sequentially to stay within VRAM limits.
- **High Quality**: Quantized 4-bit support for high-quality generation on consumer GPUs.
- **Flexible Filenaming**: Automatic filename generation with counters and random suffixes.
- **Image Publishing**: Publishes generated images directly to the `generated_image` topic using `cv_bridge`.
- **GPU Optimized**: Specifically optimized for consumer GPUs with 24GB VRAM (e.g., NVIDIA RTX 4090), ensuring smooth execution within these hardware limits.

## Setup and Installation

### 1. Clone the Repository

Navigate to your ROS 2 workspace `src` directory and clone the package:

```bash
cd ~/ros2_ws/src
git clone https://github.com/bob-ros2/bob_flux2.git
```

### 2. Install Dependencies

It is **highly recommended** to use a Python virtual environment (`venv`) to avoid library version conflicts and maintain a clean system environment.

Once your environment is active, install the required packages:

```bash
pip install -r requirements.txt
```

> [!NOTE]
> Ensure `cv_bridge` and `sensor_msgs` are present in your ROS 2 environment. These are usually included in the `desktop` installation or can be installed via `apt install ros-<distro>-cv-bridge`.

### 3. Build the Package

From the root of your ROS 2 workspace:

```bash
colcon build --packages-select bob_flux2
source install/setup.bash
```

## Parameters

| Parameter | Type | Default | Env Variable | Description |
| :--- | :--- | :--- | :--- | :--- |
| `repo_id` | string | `diffusers/FLUX.2-dev-bnb-4bit` | `FLUX2_REPO_ID` | The Hugging Face repository ID. |
| `model_dir` | string | `./models` | `FLUX2_MODEL_DIR` | Directory to cache models. |
| `device` | string | `cuda:0` | `FLUX2_DEVICE` | Torch device to use (e.g., `cuda:0` or `cpu`). |
| `prompt` | string | `''` | `FLUX2_PROMPT` | Initial prompt to run at startup. |
| `input_image` | string | `''` | `FLUX2_INPUT_IMAGE` | Path or URL to an input image for ITTI. |
| `once` | bool | `false` | `FLUX2_ONCE` | If `true`, the node exits after one generation. |
| `image_path` | string | `''` | `FLUX2_IMAGE_PATH` | Save path. Ends with `auto` for autogeneration. |
| `seed` | int | `-1` | `FLUX2_SEED` | Seed for generation (`-1` for random). |
| `image_counter_start` | int | `1` | `FLUX2_IMAGE_COUNTER_START` | Starting value for the filename counter. |

## Topics

### Subscribed Topics

- **`prompt`** (`std_msgs/String`): Receives text prompts for image generation.

### Published Topics

- **`generated_image`** (`sensor_msgs/Image`): Publishes the generated image.

## Usage

### Using the Example Configuration

The package includes a sample configuration file at `config/tti.yaml`. This is the easiest way to start the node with customized settings.

```bash
ros2 run bob_flux2 flux2_node --ros-args --params-file src/bob_flux2/config/tti.yaml
```

### Command Line Examples

#### Simple Text-to-Image
```bash
ros2 run bob_flux2 flux2_node --ros-args -p prompt:="A futuristic city in the style of cyberpunk"
```

#### Save with Auto-filenaming
Setting `image_path` to a path ending in `auto` will generate files like `auto_0001_rtRfTDrt.png`.

```bash
ros2 run bob_flux2 flux2_node --ros-args \
    -p image_path:=/path/to/output/auto \
    -p image_counter_start:=1
```

#### Image-to-Image (ITTI)
```bash
ros2 run bob_flux2 flux2_node --ros-args \
    -p input_image:=/path/to/input.png \
    -p prompt:="Make it look like a oil painting"
```

# ROS Package [bob_sd35](https://github.com/bob-ros2/bob_sd35)

A ROS2 package for image generation using Stable Diffusion 3.5 Large.

This package provides two nodes:
- **itti** - Image & Text to Image generation (img2img)
- **tti** - Text to Image generation (text2img)

Both nodes automatically download the SD3.5 model from HuggingFace on first run and support CPU offloading for systems with limited VRAM.

> **Note:** The SD3.5 Large model is approximately **67 GB** in size. The initial download will take a significant amount of time depending on your internet connection.

## Dependencies

### System Dependencies
- ROS2 Humble
- ros-humble-cv-bridge
- ros-humble-image-transport

### Python Dependencies
Install via pip:
```bash
pip install -r requirements.txt
```

## Installation and Building

> **Note:** It is recommended to use a Python virtual environment for installing the Python dependencies to avoid conflicts with system packages.

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone https://github.com/bob-ros2/bob_sd35.git

# Install Python dependencies (preferably in a virtual environment)
pip install -r bob_sd35/requirements.txt

# Build the package
cd ~/ros2_ws
colcon build --packages-select bob_sd35

# Source the workspace
source install/setup.bash
```

## Nodes

This package provides two nodes for different use cases:

| Node | Description |
|------|-------------|
| `itti` | **Image & Text to Image** - Transforms an input image based on a text prompt using img2img pipeline. Requires `input_image`. |
| `tti` | **Text to Image** - Generates images purely from text prompts using text2img pipeline. No input image required. |

## Usage

### ITTI Node (Image-to-Image)

**One-shot generation:**
```bash
ros2 run bob_sd35 itti --ros-args \
  -p model_path:=/path/to/models/stable-diffusion-3.5-large \
  -p input_prompt:="a beautiful sunset" \
  -p input_image:=/path/to/input.jpg \
  -p output_image:=auto \
  -p once:=true
```

**Continuous mode (topic-based):**
```bash
ros2 run bob_sd35 itti --ros-args \
  -p model_path:=/path/to/models/stable-diffusion-3.5-large \
  -p input_image:=/path/to/default_input.jpg \
  -p output_image:=auto
```

Then publish prompts:
```bash
# Plain text prompt (uses input_image parameter)
ros2 topic pub --once --keep-alive 1.0 /input_prompt std_msgs/msg/String "data: 'forest background'"

# JSON with image_url (ITTI only - overrides input_image parameter)
ros2 topic pub --once --keep-alive 1.0 /input_prompt std_msgs/msg/String \
  "data: '{\"role\": \"user\", \"content\": \"cosmic background\", \"image_url\": \"file:///path/to/image.jpg\"}'"
```

### TTI Node (Text-to-Image)

**One-shot generation:**
```bash
ros2 run bob_sd35 tti --ros-args \
  -p model_path:=/path/to/models/stable-diffusion-3.5-large \
  -p input_prompt:="a spacecraft in deep space" \
  -p output_image:=spacecraft.png \
  -p once:=true
```

**Continuous mode (topic-based):**
```bash
ros2 run bob_sd35 tti --ros-args \
  -p model_path:=/path/to/models/stable-diffusion-3.5-large \
  -p output_image:=auto
```

Then publish prompts:
```bash
ros2 topic pub --once --keep-alive 1.0 /input_prompt std_msgs/msg/String "data: 'a beautiful mountain landscape'"
```

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `input_prompt` | `std_msgs/String` | Text prompt to trigger image generation. For **itti**: accepts plain text or JSON with `role`, `content`, and `image_url` keys. For **tti**: plain text only. |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `generated_image` | `sensor_msgs/Image` | Generated image published when subscribers are connected. |

## ROS Parameters

Both nodes share most parameters. Parameters marked with **(itti only)** are only available in the `itti` node.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_repo` | string | `stabilityai/stable-diffusion-3.5-large` | HuggingFace model repository ID. |
| `model_path` | string | `./models/stable-diffusion-3.5-large` | Path to the local model directory. |
| `input_prompt` | string | `""` | Initial prompt for generation. |
| `input_image` | string | `""` | **(itti only)** Initial input image path. |
| `output_image` | string | `output.png` | Output image path. If ending with `auto`, generates unique filenames. |
| `strength` | double | `0.6` | **(itti only)** Denoising strength (0.0-1.0). Low=keep original, high=more change. |
| `guidance_scale` | double | `4.5` | Classifier-free guidance scale. |
| `negative_prompt` | string | `""` | Negative prompt to exclude unwanted elements from generation. |
| `num_inference_steps` | int | `40` | Number of inference steps. |
| `once` | bool | `false` | If true, shuts down after first generation. |
| `cpu_offload` | bool | `true` | Enable model CPU offload to save VRAM. |
| `keep_model_loaded` | bool | `false` | Keep model loaded between generations. Set to true to save loading time. |
| `keep_alive` | double | `1.0` | Seconds to wait before shutdown when `once` is true, allowing consumers to receive messages. |
| `seed` | int | `-1` | Random seed for reproducibility. Use -1 for random seed each generation. |
| `image_counter_start` | int | `1` | Starting value for auto image numbering. |

## Environment Variables

All parameters can also be set via environment variables with prefix `ITTI_` for the itti node and `TTI_` for the tti node (e.g., `ITTI_MODEL_PATH`, `TTI_GUIDANCE_SCALE`).
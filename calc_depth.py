import cv2
import torch
import numpy as np
import os

from get_unique_name import get_unique_name
from depth_anything_v2.dpt import DepthAnythingV2

DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
print(f"{DEVICE}, version: {torch.version.cuda}")

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

encoder = 'vits' # or 'vits', 'vitb', 'vitg'

model = DepthAnythingV2(**model_configs[encoder])
model.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu', weights_only=True))
model = model.to(DEVICE).eval()

print("load model completely")

def calc_real_depth_and_save(frame, real_depth_center, block_pixel=-1, out_dir='depth_map', base_depth_name='depth_map.png', base_frame_name='frame.png'):
    """
    Calculates real depth values and saves a depth map image.
    Args:
        frame (np.array): Input image for depth calculation.
        real_depth_center (float): Real depth value for the center pixel.
        base_directory (str): Base name for the directory.
        base_depth_name (str): Base name for the image file.
    Returns:
        tuple: Leftmost real depth, rightmost real depth, and center real depth.
    """
    depth_normalized = None
    pooled_depth = None
    C = None
    depth_visualized = None

    try:
        # Infer depth (HxW NumPy array) - replace with your depth model's infer function
        depth = model.infer_image(frame)  # Ensure 'model' is defined elsewhere in your code
        depth_block = -1

        # Normalize the depth map to be within the range [0, 255]
        depth_normalized = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth_normalized[depth_normalized < 10] = 10
        depth_normalized[depth_normalized > 255] = 255

        # Calculate the constant C
        height, width = depth.shape
        center_y, center_x = height // 2, width // 2
        relative_depth_center = depth[center_y, center_x]
        C = real_depth_center * relative_depth_center

        # Real depth calculations
        real_depth_leftmost = C / max(depth[center_y, 0], 1e-6)
        real_depth_rightmost = C / max(depth[center_y, -1], 1e-6)
        if block_pixel != -1:
            depth_block = C / max(depth[center_y, block_pixel], 1e-6)
        # Clamp and round real depth values
        depth_values = [
            min(max(round(real_depth_leftmost, 2), 0), 100),
            min(max(round(real_depth_rightmost, 2), 0), 100),
            min(max(round(depth_block, 2), 0), 100) if depth_block != -1 else -1,
            depth_normalized
        ]

        print(f"{depth_values[0]} -- {real_depth_center} -- {depth_values[1]}")
        return tuple(depth_values)

    except Exception as e:
        print(f"An error occurred: {e}")
        return None

    finally:
        try:
            # Mean pooling
            kernel_size = 5
            pooled_height = depth.shape[0] // kernel_size
            pooled_width = depth.shape[1] // kernel_size
            pooled_depth = np.zeros((pooled_height, pooled_width), dtype=np.float32)
            for i in range(pooled_height):
                for j in range(pooled_width):
                    pooled_depth[i, j] = depth[
                        i * kernel_size:(i + 1) * kernel_size,
                        j * kernel_size:(j + 1) * kernel_size
                    ].mean()

            # Normalize pooled depth map
            pooled_depth_normalized = (pooled_depth - pooled_depth.min()) / (pooled_depth.max() - pooled_depth.min()) * 255.0
            pooled_depth_normalized = pooled_depth_normalized.astype(np.uint8)

            # Upscale pooled depth map
            upscale_factor = 60
            up_width = pooled_depth.shape[1] * upscale_factor
            up_height = pooled_depth.shape[0] * upscale_factor
            up_dim = (up_width, up_height)
            depth_upscaled = cv2.resize(pooled_depth_normalized, up_dim, interpolation=cv2.INTER_NEAREST)

            # Convert to BGR for visualization
            depth_visualized = cv2.cvtColor(depth_upscaled, cv2.COLOR_GRAY2BGR)

            # Overlay real depth values
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.4
            center_color = (0, 0, 255)  # Red for center row
            other_color = (0, 255, 0)  # Green for other rows
            thickness = 1

            center_row = pooled_depth.shape[0] // 2  # Calculate center row
            for y in range(pooled_depth.shape[0]):
                for x in range(pooled_depth.shape[1]):
                    value = C / (pooled_depth[y, x] + 1e-6)  # Avoid division by zero
                    text = f"{int(value)}"
                    position = (x * upscale_factor + upscale_factor // 4, y * upscale_factor + upscale_factor // 2)
                    if y == center_row:
                        color = center_color  # Red for center row
                    else:
                        color = other_color  # Green for other rows
                    cv2.putText(depth_visualized, text, position, font, font_scale, color, thickness)

            # Upscale original frame for saving
            upscale_frame_factor = 60
            upscaled_frame = cv2.resize(frame, 
                                         (frame.shape[1] * upscale_frame_factor, frame.shape[0] * upscale_frame_factor), 
                                         interpolation=cv2.INTER_LINEAR)

            # Save the images
            depth_base, depth_extension = os.path.splitext(base_depth_name)
            unique_depth_name = get_unique_name(os.path.join(out_dir, depth_base), is_directory=False, extension=depth_extension)
            frame_base, frame_extension = os.path.splitext(base_frame_name)
            unique_frame_name = get_unique_name(os.path.join(out_dir, frame_base), is_directory=False, extension=frame_extension)

            cv2.imwrite(unique_depth_name, depth_visualized)  # Save upscaled depth map
            cv2.imwrite(unique_frame_name, frame)  # Save upscaled original frame
            print(f"Depth map image saved as '{unique_depth_name}'")
            print(f"Upscaled frame image saved as '{unique_frame_name}'")
        except Exception as final_error:
            print(f"An error occurred while writing the image: {final_error}")


def calc_real_depth(frame, real_depth_center, block_pixel=-1):
    """
    Calculates real depth values and saves a depth map image.
    Args:
        frame (np.array): Input image for depth calculation.
        real_depth_center (float): Real depth value for the center pixel.
        base_directory (str): Base name for the directory.
        base_depth_name (str): Base name for the image file.
    Returns:
        tuple: Leftmost real depth, rightmost real depth, and center real depth.
    """
    depth_normalized = None
    C = None

    try:
        # Infer depth (HxW NumPy array) - replace with your depth model's infer function
        depth = model.infer_image(frame)  # Ensure 'model' is defined elsewhere in your code
        depth_block = -1

        # Normalize the depth map to be within the range [0, 255]
        depth_normalized = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth_normalized[depth_normalized < 10] = 10
        depth_normalized[depth_normalized > 255] = 255

        # Calculate the constant C
        height, width = depth.shape
        center_y, center_x = height // 2, width // 2
        relative_depth_center = depth[center_y, center_x]
        C = real_depth_center * relative_depth_center

        # Real depth calculations
        real_depth_leftmost = C / max(depth[center_y, 0], 1e-6)
        real_depth_rightmost = C / max(depth[center_y, -1], 1e-6)
        if block_pixel != -1:
            depth_block = C / max(depth[center_y, block_pixel], 1e-6)
        # Clamp and round real depth values
        depth_values = [
            min(max(round(real_depth_leftmost, 2), 0), 100),
            min(max(round(real_depth_rightmost, 2), 0), 100),
            min(max(round(depth_block, 2), 0), 100) if depth_block != -1 else -1,
            depth_normalized
        ]

        print(f"{depth_values[0]} -- {real_depth_center} -- {depth_values[1]}")
        return tuple(depth_values)

    except Exception as e:
        print(f"An error occurred: {e}")
        return None
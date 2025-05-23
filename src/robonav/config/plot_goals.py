import cv2
import numpy as np
import yaml

def draw_goals_on_map(yaml_path, image_path=None, image_width=300, image_height=200, save_path=None):
    # Load YAML file
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    goals = data.get("goals", [])

    # Load the base image if provided, otherwise create a blank white image
    if image_path:
        image = cv2.imread(image_path)
        if image is None:
            raise FileNotFoundError(f"Image file not found: {image_path}")
        image_height, image_width, _ = image.shape
    else:
        image = np.ones((image_height, image_width, 3), dtype=np.uint8) * 255

    for idx, goal in enumerate(goals):
        x_cm = goal['x']
        y_cm = goal['y']

        # Convert cm to image pixels (assuming 10 cm = 1 pixel)
        x_px = int(x_cm / 10)
        y_px = int(y_cm / 10)

        # Flip Y to match image coordinates (origin top-left)
        y_px = image_height - y_px

        if 0 <= x_px < image_width and 0 <= y_px < image_height:
            # Draw the goal point
            cv2.circle(image, (x_px, y_px), 2, (0, 0, 255), -1)  # Red dot
            cv2.putText(image, str(idx + 1), (x_px + 2, y_px - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
        else:
            print(f"⚠️ Goal {idx + 1} is out of image bounds: x={x_px}, y={y_px}")

    if save_path:
        cv2.imwrite(save_path, image)
        print(f"Image saved to {save_path}")
    else:
        scale_factor = 5  # Adjust this factor to make the image bigger
        resized_image = cv2.resize(image, (image_width * scale_factor, image_height * scale_factor), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Goals Map", resized_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# Example usage
if _name_ == "_main_":
    yaml_file = "/home/ferdinand/2024-2025-Robotics-cup/src/robonav/config/goals_all.yaml"  # Replace with the path to your YAML file
    image_file = "/home/ferdinand/2024-2025-Robotics-cup/src/robonav/maps/Eurobot_map_real_bw_10_p.png"  # Replace with the path to your image file
    draw_goals_on_map(yaml_file, image_path=image_file)
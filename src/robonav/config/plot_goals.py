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
        # Add a border around obstacles
        border_size = 22 ###################

        border_color = (200, 100, 100)  # Gray color for the border
        obstacle_threshold = 120  # Threshold for detecting dark gray obstacles

        # Create a mask for all pixels below the threshold
        obstacle_mask = np.zeros((image_height, image_width), dtype=np.uint8)
        for y in range(image_height):
            for x in range(image_width):
                pixel_color = image[y, x]
                if pixel_color[0] == pixel_color[1] == pixel_color[2]:  # Check if the color is gray
                    if pixel_color[0] < obstacle_threshold:  # Check if it's darker than the threshold
                        obstacle_mask[y, x] = 255  # Mark obstacle in the mask

        # Dilate the obstacle mask (make the mask bigger) to include the border
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2 * border_size + 1, 2 * border_size + 1))
        dilated_mask = cv2.dilate(obstacle_mask, kernel)

        # Subtract the original obstacle mask from the dilated mask to isolate the border
        border_mask = cv2.subtract(dilated_mask, obstacle_mask)

        # Color each pixel of the image where the mask is the border color
        for y in range(image_height):
            for x in range(image_width):
                if border_mask[y, x] == 255:
                    image[y, x] = border_color

        print(f"Added border of {border_size} pixels to permanent (dark gray) obstacles.")
        # Ensure the original image is preserved after adding the border
        pass



    for idx, goal in enumerate(goals):
        x_cm = goal['x']
        y_cm = goal['y']

        # Convert cm to image pixels (1 cm = 1 pixel)
        x_px = x_cm
        y_px = y_cm
        
        # Flip Y to match image coordinates (origin top-left)
        y_px = image_height - y_px - 1

        if 0 <= x_px < image_width and 0 <= y_px < image_height:
            if idx <= 17:
                team = (0, 255, 255)  # yellow team
            else:
                team = (255, 0, 0)    # blue team
            # Draw the goal point
            cv2.circle(image, (x_px, y_px), 2, (0, 0, 255), -1)  # Red dot
            if idx in [25, 26, 27, 28, 29, 30, 31, 32, 33, 34]:
                cv2.putText(image, str(idx), (x_px - 10, y_px - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, team, 1)   
            else:
                cv2.putText(image, str(idx), (x_px + 2, y_px - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, team, 1)
        else:
            print(f"⚠️ Goal {idx} is out of image bounds: x={x_px}, y={y_px}")

    if save_path:
        cv2.imwrite(save_path, image)
        print(f"Image saved to {save_path}")
    else:
        scale_factor = 5  # Adjust this factor to make the image bigger
        resized_image = cv2.resize(image, (image_width * scale_factor, image_height * scale_factor), interpolation=cv2.INTER_LINEAR)
        cv2.imshow("Goals Map", resized_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    yaml_file = "/home/baschi777/nantrobot/2024-2025-Robotics-cup/src/robonav/config/goals_all.yaml"  # Replace with the path to your YAML file
    image_file = "/home/baschi777/nantrobot/2024-2025-Robotics-cup/src/robonav/maps/Eurobot_map_real_bw_10_p.png"  # Replace with the path to your image file
    draw_goals_on_map(yaml_file, image_path=image_file)
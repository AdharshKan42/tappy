import cv2
import numpy as np
from PIL import ImageDraw, ImageFont, Image
import io

def draw_key_detections(image_path, detections):
    """
    Draws bounding boxes and labels on the input image based on the detections.

    Args:
        image_path (str): Path to the input image.
        detections (list): A list of dictionaries, where each dictionary
                           contains bounding box coordinates, label, and text.

    Returns:
        PIL.Image.Image: The augmented PIL image.
    """
    try:
        # Open the image with PIL for drawing
        pil_image = Image.open(image_path).convert("RGB")
        draw = ImageDraw.Draw(pil_image)

        # Choose a font (you might need to adjust the path)
        try:
            font = ImageFont.truetype("arial.ttf", 16)  # Adjust font size as needed
        except IOError:
            font = ImageFont.load_default()

        for detection in detections:
            bbox = detection['bounding_box']
            label = detection['label']
            text = detection['text']

            x_min, y_min =  bbox[1], bbox[0]
            x_max, y_max =  bbox[1] + detection['width'], bbox[0] + detection['height']

            # Draw bounding box
            draw.rectangle([(x_min, y_min), (x_max, y_max)], outline="lime", width=2)

            # Draw label and text
            label_text = f"{label}: {text}"
            text_x = x_min
            text_y = y_min - 20
            draw.text((text_x, text_y), label_text, fill="yellow", font=font)

        return pil_image

    except FileNotFoundError:
        print(f"Error: Image not found at {image_path}")
        return None
    except Exception as e:
        print(f"An error occurred during drawing: {e}")
        return None

if __name__ == "__main__":
    image_file = "main.png"  # Replace with the actual path to your image
    detections = [
        {"bounding_box": [327, 166, 372, 199], "label": "Key", "text": "1", "width": 36, "height": 33},
        {"bounding_box": [327, 209, 372, 242], "label": "Key", "text": "2", "width": 36, "height": 33},
        {"bounding_box": [327, 253, 372, 286], "label": "Key", "text": "3", "width": 36, "height": 33},
        {"bounding_box": [327, 297, 372, 330], "label": "Key", "text": "4", "width": 36, "height": 33},
        {"bounding_box": [327, 341, 372, 374], "label": "Key", "text": "5", "width": 36, "height": 33},
        {"bounding_box": [327, 385, 372, 418], "label": "Key", "text": "6", "width": 36, "height": 33},
        {"bounding_box": [327, 429, 372, 462], "label": "Key", "text": "7", "width": 36, "height": 33},
        {"bounding_box": [327, 473, 372, 506], "label": "Key", "text": "8", "width": 36, "height": 33},
        {"bounding_box": [327, 517, 372, 550], "label": "Key", "text": "9", "width": 36, "height": 33},
        {"bounding_box": [327, 561, 372, 594], "label": "Key", "text": "0", "width": 36, "height": 33},
        {"bounding_box": [400, 176, 446, 209], "label": "Key", "text": "Q", "width": 33, "height": 33},
        {"bounding_box": [400, 221, 446, 254], "label": "Key", "text": "W", "width": 33, "height": 33},
        {"bounding_box": [400, 266, 446, 299], "label": "Key", "text": "E", "width": 33, "height": 33},
        {"bounding_box": [400, 311, 446, 344], "label": "Key", "text": "R", "width": 33, "height": 33},
        {"bounding_box": [400, 356, 446, 389], "label": "Key", "text": "T", "width": 33, "height": 33},
        {"bounding_box": [400, 401, 446, 434], "label": "Key", "text": "Y", "width": 33, "height": 33},
        {"bounding_box": [400, 446, 446, 479], "label": "Key", "text": "U", "width": 33, "height": 33},
        {"bounding_box": [400, 491, 446, 524], "label": "Key", "text": "I", "width": 33, "height": 33},
        {"bounding_box": [400, 536, 446, 569], "label": "Key", "text": "O", "width": 33, "height": 33},
        {"bounding_box": [400, 581, 446, 614], "label": "Key", "text": "P", "width": 33, "height": 33},
        {"bounding_box": [477, 176, 537, 219], "label": "Key", "text": "A", "width": 43, "height": 43},
        {"bounding_box": [477, 229, 537, 272], "label": "Key", "text": "S", "width": 43, "height": 43},
        {"bounding_box": [477, 283, 537, 326], "label": "Key", "text": "D", "width": 43, "height": 43},
        {"bounding_box": [477, 337, 537, 380], "label": "Key", "text": "F", "width": 43, "height": 43},
        {"bounding_box": [477, 391, 537, 434], "label": "Key", "text": "G", "width": 43, "height": 43},
        {"bounding_box": [477, 445, 537, 488], "label": "Key", "text": "H", "width": 43, "height": 43},
        {"bounding_box": [477, 499, 537, 542], "label": "Key", "text": "J", "width": 43, "height": 43},
        {"bounding_box": [477, 553, 537, 596], "label": "Key", "text": "K", "width": 43, "height": 43},
        {"bounding_box": [477, 607, 537, 650], "label": "Key", "text": "L", "width": 43, "height": 43},
        {"bounding_box": [558, 188, 600, 231], "label": "Key", "text": "Z", "width": 43, "height": 43},
        {"bounding_box": [558, 242, 600, 285], "label": "Key", "text": "X", "width": 43, "height": 43},
        {"bounding_box": [558, 296, 600, 339], "label": "Key", "text": "C", "width": 43, "height": 43},
        {"bounding_box": [558, 350, 600, 393], "label": "Key", "text": "V", "width": 43, "height": 43},
        {"bounding_box": [558, 404, 600, 447], "label": "Key", "text": "B", "width": 43, "height": 43},
        {"bounding_box": [558, 458, 600, 501], "label": "Key", "text": "N", "width": 43, "height": 43},
        {"bounding_box": [558, 512, 600, 555], "label": "Key", "text": "M", "width": 43, "height": 43}
        ]

    augmented_image = draw_key_detections(image_file, detections)

    if augmented_image:
        augmented_image.show()
        augmented_image.save("augmented_keyboard.jpg") # Save the augmented image
import os
import random
import shutil

def copy_random_images(origin, destiny, porcentaje=0.3):
    # Create directory
    os.makedirs(destiny, exist_ok=True)

    # Get availble file type
    valid_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.gif']
    images = [f for f in os.listdir(origin)
                if os.path.isfile(os.path.join(origin, f)) and os.path.splitext(f)[1].lower() in valid_extensions]

    # Calculate random images
    amount_to_copy = max(1, int(len(images) * porcentaje))
    seleccionadas = random.sample(images, amount_to_copy)

    
    for img in seleccionadas:
        path_origin = os.path.join(origin, img)
        path_destiny = os.path.join(destiny, img)
        shutil.copy2(path_origin, path_destiny)
        print(f"Copied: {img}")

    print(f"\nTotal: {amount_to_copy} copied images from {len(images)} available.")


if __name__ == "__main__":
    folder_origin = "/home/andre14/dev_ws/src/welectron_deployments/welectron_deployments/recorded_frames"
    folder_destiny = "/home/andre14/dev_ws/src/welectron_deployments/welectron_deployments/split_frames"
    copy_random_images(folder_origin, folder_destiny)
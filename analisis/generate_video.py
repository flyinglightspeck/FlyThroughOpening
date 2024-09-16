import cv2
import os
from tqdm import tqdm

def create_video_from_images(image_folder, output_video, fps, border=[0, 0, 0, 0]):
    files = [img for img in os.listdir(image_folder) if img.endswith(".png")]
    # images.sort()  # Ensure the images are in the correct order

    images = [f'{i}.png' for i in range(1, len(files) + 1)]

    if not images:
        print("No PNG images found in the directory.")
        return

    # Read the first image to get the dimensions
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    height, width = height - border[0] - border[1], width - border[2] - border[3]

    # Define the codec and create a VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can use 'XVID' or other codecs
    video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

    for i in tqdm(range(len(images))):
        image = images[i]
        frame = cv2.imread(os.path.join(image_folder, image))
        height, width, _ = frame.shape

        # Crop the frame by slicing it
        cropped_frame = frame[border[0]:height - border[1], border[2]:width - border[3]]

        # Write the cropped frame to the video
        video.write(cropped_frame)
        # video.write(frame)

    video.release()
    print(f"Video saved as {output_video}")


image_folder = '/Users/shuqinzhu/Desktop/video_frames/kangaroo_972_3_OPT/'
output_video = '/Users/shuqinzhu/Desktop/video_frames/kangaroo_972_3_OPT/simulation.mp4'  # Output video file name
fps = 30  # Frames per second

border = [450, 250, 0, 0]  # distance to top, bottom, left, right

create_video_from_images(image_folder, output_video, fps, border)

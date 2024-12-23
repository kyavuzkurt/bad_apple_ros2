import os
import subprocess
from PIL import Image

def check_and_create_directories():
    ascii_dir = 'frames-ascii'
    bad_apple_dir = 'frames-bad-apple'

    if not os.path.exists(ascii_dir):
        os.makedirs(ascii_dir)
        print(f"Created directory: {ascii_dir}")

    if not os.path.exists(bad_apple_dir):
        os.makedirs(bad_apple_dir)
        print(f"Created directory: {bad_apple_dir}")

def convert_video_to_frames(video_path, output_dir):
    if not os.path.isfile(video_path):
        raise FileNotFoundError(f"Video file {video_path} does not exist.")

    command = [
        'ffmpeg',
        '-i', video_path,
        os.path.join(output_dir, 'frame_%04d.png')
    ]

    print(f"Converting {video_path} to frames in {output_dir}...")
    subprocess.run(command, check=True)
    print("Video conversion to frames completed.")

def map_pixels_to_ascii(image, width=100):
    ascii_chars = ['@', '#', 'S', '%', '?', '*', '+', ';', ':', ',', '.']
    # Resize image
    aspect_ratio = image.height / image.width
    new_height = int(aspect_ratio * width * 0.55)
    image = image.resize((width, new_height))
    # Convert to grayscale
    image = image.convert('L')

    pixels = image.getdata()
    ascii_str = ''.join([ascii_chars[pixel // 25] for pixel in pixels])
    ascii_lines = [ascii_str[index:index + width] for index in range(0, len(ascii_str), width)]
    return "\n".join(ascii_lines)

def convert_frames_to_ascii(frames_dir, ascii_dir):
    frame_files = sorted([f for f in os.listdir(frames_dir) if f.endswith('.png')])
    total_frames = len(frame_files)
    print(f"Converting {total_frames} frames to ASCII...")

    for idx, frame_file in enumerate(frame_files, 1):
        frame_path = os.path.join(frames_dir, frame_file)
        with Image.open(frame_path) as img:
            ascii_art = map_pixels_to_ascii(img)
        
        ascii_filename = f"{os.path.splitext(frame_file)[0]}.txt"
        ascii_path = os.path.join(ascii_dir, ascii_filename)
        with open(ascii_path, 'w') as f:
            f.write(ascii_art)
        
        if idx % 100 == 0 or idx == total_frames:
            print(f"Converted {idx}/{total_frames} frames.")

    print("All frames have been converted to ASCII.")

def main():
    video_file = 'bad_apple.mp4'
    ascii_directory = 'frames-ascii'
    frames_directory = 'frames-bad-apple'

    check_and_create_directories()

    if not os.listdir(frames_directory):
        convert_video_to_frames(video_file, frames_directory)
    else:
        print(f"Frames already exist in {frames_directory}.")

    if not os.listdir(ascii_directory):
        convert_frames_to_ascii(frames_directory, ascii_directory)
    else:
        print(f"ASCII frames already exist in {ascii_directory}.")

if __name__ == "__main__":
    main()

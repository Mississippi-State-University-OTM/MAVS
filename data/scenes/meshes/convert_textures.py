import os
from PIL import Image
import sys

def convert_to_bmp(directory):
    for filename in os.listdir(directory):
        if filename.endswith(('tga', '.jpg', '.jpeg', '.png', '.gif', '.tif','.tiff')):  # Add more extensions if needed
            filepath = os.path.join(directory, filename)
            with Image.open(filepath) as img:
                bmp_filename = os.path.splitext(filename)[0] + ".bmp"
                bmp_filename = bmp_filename.replace(" ", "_")
                bmp_filepath = os.path.join(directory, bmp_filename)
                img.save(bmp_filepath, "BMP")

if __name__ == "__main__":
    directory = sys.argv[1] #"/path/to/your/image/directory"  # Replace with your directory
    convert_to_bmp(directory)
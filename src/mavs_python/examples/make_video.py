import os
import moviepy.video.io.ImageSequenceClip

image_folder='./'
fps=25

image_files = [image_folder+'/'+img for img in sorted(os.listdir(image_folder))
if img.endswith(".bmp")]
clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(image_files, fps=fps
)
clip.write_videofile('video.mp4')
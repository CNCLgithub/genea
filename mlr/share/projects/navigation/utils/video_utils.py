import subprocess

from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils


def make_video(images_dirpath, out_vid_filepath, fps=24):
    image_template = images_dirpath.split("/")[-1]

    subprocess.run([
        "ffmpeg",
        "-y",
        "-framerate", str(fps),
        "-start_number", "0001",
        "-i", f"{image_template}%04d.png",
        "-c:v", "libx264",
        "-crf", "0",
        "-pix_fmt", "yuv444p",
        out_vid_filepath,
    ], check=True, cwd=images_dirpath)


def main():
    stim_videos_dirpath = PathUtils.get_out_videos_dirpath() + "_220"
    for stimulus_dirpath in FileUtils.get_dir_list_in_directory(stim_videos_dirpath):
        stimulus_name = stimulus_dirpath.split("/")[-1]
        if "23" not in stimulus_name:
            continue
        out_vid_filepath = PathUtils.join(stim_videos_dirpath, f"{stimulus_name}.mp4")
        make_video(stimulus_dirpath, out_vid_filepath)


if __name__ == '__main__':
    main()

import os
import math

try:
    import bmesh
    import bpy

    from mathutils import Vector

    BASE_PATH = os.path.dirname(os.path.dirname(bpy.data.texts.get(bpy.context.space_data.text.name).filepath))
    STIMULI_DIRPATH = os.path.join(BASE_PATH, "library", "stimuli")

except ImportError:
    STIMULI_DIRPATH = None
    print("Unable to import or load Blender Python API.")


STIMULI_SET_NAME = "diff"
VIDEO_FRAME_COUNT = 50


class _FileUtils:
    @staticmethod
    def import_stl_file(filepath):
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"ERROR [video_utils]: file not found: {filepath}")
        bpy.ops.wm.stl_import(filepath=filepath)

    @staticmethod
    def import_obj_file(filepath):
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"ERROR [video_utils]: file not found: {filepath}")
        bpy.ops.import_scene.obj(filepath=filepath)

    @staticmethod
    def get_dir_list_in_directory(dirpath):
        try:
            return list(filter(os.path.isdir, [os.path.join(dirpath, i) for i in os.listdir(dirpath)]))
        except Exception as e:
            print("ERROR: issue in retrieving directory list within given directory " + dirpath)
            print(str(e))


class StimuliBPYUtils:
    CAMERA_ELLIPTICAL = "CAMERA_ELLIPTICAL"

    @staticmethod
    def _set_elliptical_trajectory(blender_obj, ellipse_multiplier, ellipse_height, total_waypoints):
        major_axis = (StimuliBPYUtils.get_stim_x_max() - StimuliBPYUtils.get_stim_x_min()) * ellipse_multiplier
        minor_axis = (StimuliBPYUtils.get_stim_y_max() - StimuliBPYUtils.get_stim_y_min()) * ellipse_multiplier
        minor_axis = max(minor_axis, 0.8 * major_axis)

        theta = math.pi / total_waypoints
        for i in range(total_waypoints + 1):
            x = StimuliBPYUtils.get_stim_center_x() + major_axis * math.cos(theta * i + math.pi)
            y = StimuliBPYUtils.get_stim_center_y() + minor_axis * math.sin(theta * i + math.pi)
            blender_obj.location = (x, y, ellipse_height)
            blender_obj.keyframe_insert(data_path="location", frame=i + 1)

    @staticmethod
    def _add_lighting(light_location=(0., 0., 0.), light_energy=10000.0, light_size=2):
        bpy.ops.object.light_add(type='AREA', align='WORLD', location=light_location)
        area = bpy.context.object
        area.data.energy = light_energy
        area.data.size = light_size

    @staticmethod
    def _add_camera(tracer_height, tracer_radius, camera_height, camera_radius):
        bpy.ops.object.empty_add()
        tracer = bpy.context.active_object

        bpy.ops.object.camera_add()
        camera = bpy.context.object
        bpy.context.scene.camera = camera

        StimuliBPYUtils._set_elliptical_trajectory(tracer, tracer_radius, tracer_height, VIDEO_FRAME_COUNT)
        StimuliBPYUtils._set_elliptical_trajectory(camera, camera_radius, camera_height, VIDEO_FRAME_COUNT)

        camera.constraints.new(type='TRACK_TO')
        camera.constraints["Track To"].target = tracer
        camera.constraints["Track To"].track_axis = 'TRACK_NEGATIVE_Z'
        camera.constraints["Track To"].up_axis = 'UP_Y'

    @staticmethod
    def _get_stim_bounds():
        StimuliBPYUtils.bpy_select_all()

        corners_list = []
        for obj in bpy.context.scene.objects:
            if not obj.type == 'MESH':
                continue

            if obj.name.startswith("ground"):
                obj.select_set(False)
                continue

            corners_list.extend([obj.matrix_world @ Vector(corner) for corner in obj.bound_box])

        x = [v.x for v in corners_list]
        y = [v.y for v in corners_list]
        z = [v.z for v in corners_list]

        return {"x_min": min(x),
                "x_max": max(x),
                "y_min": min(y),
                "y_max": max(y),
                "z_min": min(z),
                "z_max": max(z)}

    @staticmethod
    def bpy_select_all():
        bpy.ops.object.select_all(action='SELECT')

    @staticmethod
    def bpy_deselect_all():
        bpy.ops.object.select_all(action='DESELECT')

    @staticmethod
    def bpy_clear():
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)

    @staticmethod
    def setup_scene(scene_filepath):
        StimuliBPYUtils.bpy_clear()

        bpy.context.scene.world.color = (0, 0, 0)

        if scene_filepath.endswith(".stl"):
            _FileUtils.import_stl_file(scene_filepath)
        elif scene_filepath.endswith(".obj"):
            _FileUtils.import_obj_file(scene_filepath)

        light_height = StimuliBPYUtils.get_stim_z_min() + abs(StimuliBPYUtils.get_stim_height()) * 6
        light_location = (StimuliBPYUtils.get_stim_center_x(), StimuliBPYUtils.get_stim_center_y(), light_height)
        StimuliBPYUtils._add_lighting(light_location=light_location)

        tracer_height = StimuliBPYUtils.get_stim_z_min() + abs(StimuliBPYUtils.get_stim_height()) * 0.5
        camera_height = StimuliBPYUtils.get_stim_z_min() + abs(StimuliBPYUtils.get_stim_height()) * 0.6
        StimuliBPYUtils._add_camera(tracer_height, 1, camera_height, 2.5)

    @staticmethod
    def render_video(out_video_filepath):
        scene = bpy.context.scene
        scene.render.filepath = out_video_filepath
        scene.render.image_settings.file_format = 'FFMPEG'
        scene.render.ffmpeg.format = 'MPEG4'
        scene.frame_end = VIDEO_FRAME_COUNT

        scene.render.fps = 24
        scene.render.resolution_x = 1920
        scene.render.resolution_y = 1080
        scene.render.resolution_percentage = 100

        bpy.context.scene.cycles.device = 'GPU'
        bpy.ops.render.render(animation=True)

    @staticmethod
    def get_stim_x_min():
        return StimuliBPYUtils._get_stim_bounds()["x_min"]

    @staticmethod
    def get_stim_x_max():
        return StimuliBPYUtils._get_stim_bounds()["x_max"]

    @staticmethod
    def get_stim_y_min():
        return StimuliBPYUtils._get_stim_bounds()["y_min"]

    @staticmethod
    def get_stim_y_max():
        return StimuliBPYUtils._get_stim_bounds()["y_max"]

    @staticmethod
    def get_stim_z_min():
        return StimuliBPYUtils._get_stim_bounds()["z_min"]

    @staticmethod
    def get_stim_z_max():
        return StimuliBPYUtils._get_stim_bounds()["z_max"]

    @staticmethod
    def get_stim_center_x():
        return (StimuliBPYUtils.get_stim_x_max() + StimuliBPYUtils.get_stim_x_min()) / 2

    @staticmethod
    def get_stim_center_y():
        return (StimuliBPYUtils.get_stim_y_max() + StimuliBPYUtils.get_stim_y_min()) / 2

    @staticmethod
    def get_stim_height():
        return StimuliBPYUtils.get_stim_z_max() - StimuliBPYUtils.get_stim_z_min()


def main():
    stimuli_set_dirpath = os.path.join(STIMULI_DIRPATH, STIMULI_SET_NAME)
    stimuli_item_path_list = _FileUtils.get_dir_list_in_directory(stimuli_set_dirpath)

    for stimuli_item_path in stimuli_item_path_list:
        stimuli_filepath = os.path.join(stimuli_item_path, "meshes", "stim.obj")
        video_filepath = os.path.join(stimuli_item_path, "stim.mp4")

        StimuliBPYUtils.bpy_clear()
        StimuliBPYUtils.setup_scene(stimuli_filepath)
        StimuliBPYUtils.render_video(video_filepath)

if __name__ == '__main__':
    main()

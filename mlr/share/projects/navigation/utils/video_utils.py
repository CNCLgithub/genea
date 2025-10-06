import math
import os
import random
import shutil

try:
    import bmesh
    import bpy

    from mathutils import Vector

    BASE_PATH = os.path.dirname(os.path.dirname(bpy.data.texts.get(bpy.context.space_data.text.name).filepath))
    OUT_DIRPATH = os.path.join(BASE_PATH, "library", "out")
    STIMULI_DIRPATH = os.path.join(BASE_PATH, "library", "stimuli")

except ImportError:
    OUT_DIRPATH = None
    STIMULI_DIRPATH = None
    print("Unable to import or load Blender Python API.")

#####################
# ----- CONFIG -----
#####################

STIMULI_SET_NAME = "diff"
VIDEO_FRAME_COUNT = 80

#####################


class _PlatformNames:
    GROUND = "ground"
    AGENT = "agent"
    STAGE = "stage"


class _PlatformMaterial:
    WOOD = "MaterialWood"
    STONE = "MaterialStone"


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
    def create_dir(dirpath, do_force_create=False):
        if os.path.isdir(dirpath):
            if do_force_create:
                shutil.rmtree(dirpath)
                os.makedirs(dirpath)
            return

        os.makedirs(dirpath)

    @staticmethod
    def get_dir_list_in_directory(dirpath):
        try:
            return list(sorted(filter(os.path.isdir, [os.path.join(dirpath, i) for i in os.listdir(dirpath)])))
        except Exception as e:
            print("ERROR: issue in retrieving directory list within given directory " + dirpath)
            print(str(e))


class StimuliBPYUtils:
    @staticmethod
    def _get_chaikin_curve(input_points, total_points_to_generate, iterations=3):
        for _ in range(iterations):
            chaikin_points = [input_points[0]]
            for i in range(len(input_points) - 1):
                p0, p1 = input_points[i], input_points[i + 1]
                q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
                r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
                chaikin_points += [q, r]
            chaikin_points.append(input_points[-1])
            input_points = chaikin_points

        arc_lengths = [0.0]
        for i in range(len(input_points) - 1):
            point_x = input_points[i + 1][0] - input_points[i][0]
            point_y = input_points[i + 1][1] - input_points[i][1]
            arc_lengths.append(arc_lengths[-1] + math.hypot(point_x, point_y))

        return_points = []
        for index in range(total_points_to_generate):
            target_position = arc_lengths[-1] * index / (total_points_to_generate - 1)

            if index <= 0:
                return_points.append(input_points[0])
                continue

            if index >= total_points_to_generate:
                return_points.append(input_points[-1])
                continue

            j = 1
            while j < len(arc_lengths) and arc_lengths[j] < target_position:
                j += 1
            j = min(j, len(arc_lengths) - 1)

            t = (target_position - arc_lengths[j - 1]) / (arc_lengths[j] - arc_lengths[j - 1])
            point_x = (1 - t) * input_points[j - 1][0] + t * input_points[j][0]
            point_y = (1 - t) * input_points[j - 1][1] + t * input_points[j][1]
            return_points.append((point_x, point_y))

        return return_points

    @staticmethod
    def _set_camera_trajectory(blender_obj, shape_multiplier, stadium_height, total_waypoints):
        minor_axis = (StimuliBPYUtils.get_stim_y_max() - StimuliBPYUtils.get_stim_y_min()) * shape_multiplier

        diff_factor = (StimuliBPYUtils.get_stim_x_max() - StimuliBPYUtils.get_stim_x_min()) / 3

        l_center_x = StimuliBPYUtils.get_stim_x_min() + diff_factor
        l_center_y = StimuliBPYUtils.get_stim_center_y()

        r_center_x = StimuliBPYUtils.get_stim_x_max() - diff_factor
        r_center_y = StimuliBPYUtils.get_stim_center_y()

        waypoint_ends = total_waypoints // 5
        waypoint_line = total_waypoints // 5

        points_on_curve = []

        l_start_angle = math.radians(-150)
        l_final_angle = math.radians(-90)
        for i in range(waypoint_ends + 1):  # left arc
            theta = l_start_angle + i * (l_final_angle - l_start_angle) / waypoint_ends
            point_x = l_center_x + minor_axis * math.cos(theta)
            point_y = l_center_y + minor_axis * math.sin(theta)
            points_on_curve.append((point_x, point_y))

        for i in range(waypoint_line + 1):  # middle line
            point_x = l_center_x + i * ((r_center_x - l_center_x) / waypoint_line)
            point_y = r_center_y - minor_axis
            points_on_curve.append((point_x, point_y))

        start_angle_right = math.radians(-90)
        end_angle_right = math.radians(-30)
        for i in range(waypoint_ends + 1):  # right arc
            theta = start_angle_right + i * (end_angle_right - start_angle_right) / waypoint_ends
            point_x = r_center_x + minor_axis * math.cos(theta)
            point_y = r_center_y + minor_axis * math.sin(theta)
            points_on_curve.append((point_x, point_y))

        start_angle_right = math.radians(-90)
        end_angle_right = math.radians(-30)
        for i in range(waypoint_ends + 1)[::-1]:  # reverse right arc
            theta = start_angle_right + i * (end_angle_right - start_angle_right) / waypoint_ends
            point_x = r_center_x + minor_axis * math.cos(theta)
            point_y = r_center_y + minor_axis * math.sin(theta)
            points_on_curve.append((point_x, point_y))

        for i in range(waypoint_line + 1)[::-1]:  # middle line
            if i == waypoint_line // 2 - 1:
                break
            point_x = l_center_x + i * ((r_center_x - l_center_x) / waypoint_line)
            point_y = r_center_y - minor_axis
            points_on_curve.append((point_x, point_y))

        new_points = StimuliBPYUtils._get_chaikin_curve(points_on_curve, total_waypoints + 1, 3)
        for frame, (point_x, point_y) in enumerate(new_points, start=1):
            blender_obj.location = (point_x, point_y, stadium_height)
            blender_obj.keyframe_insert(data_path="location", frame=frame)

    @staticmethod
    def _add_camera(tracer_height, tracer_radius, camera_height, camera_radius):
        bpy.ops.object.empty_add()
        tracer = bpy.context.active_object

        bpy.ops.object.camera_add()
        camera = bpy.context.object
        bpy.context.scene.camera = camera

        StimuliBPYUtils._set_camera_trajectory(tracer, tracer_radius, tracer_height, VIDEO_FRAME_COUNT)
        StimuliBPYUtils._set_camera_trajectory(camera, camera_radius, camera_height, VIDEO_FRAME_COUNT)

        camera.constraints.new(type='TRACK_TO')
        camera.constraints["Track To"].target = tracer
        camera.constraints["Track To"].track_axis = 'TRACK_NEGATIVE_Z'
        camera.constraints["Track To"].up_axis = 'UP_Y'

    @staticmethod
    def _add_lighting(light_location=(0., 0., 0.), light_energy=20000.0, light_size=10):
        bpy.ops.object.light_add(type='AREA', align='WORLD', location=light_location)
        area = bpy.context.object
        area.data.energy = light_energy
        area.data.size = light_size

    @staticmethod
    def _add_wood_texture(bpy_obj):
        if bpy_obj is None:
            return

        mat = bpy.data.materials.new(name=_PlatformMaterial.WOOD)
        mat.use_nodes = True

        for node in mat.node_tree.nodes:
            mat.node_tree.nodes.remove(node)

        output = mat.node_tree.nodes.new(type="ShaderNodeOutputMaterial")
        bsdf = mat.node_tree.nodes.new(type="ShaderNodeBsdfPrincipled")
        tex_coord = mat.node_tree.nodes.new(type='ShaderNodeTexCoord')

        mapping = mat.node_tree.nodes.new(type='ShaderNodeMapping')
        mapping.inputs['Scale'].default_value = (random.uniform(0.5, 2),
                                                 random.uniform(0.5, 2),
                                                 random.uniform(0.5, 2))
        mapping.inputs['Rotation'].default_value = (random.uniform(0, 2 * math.pi),
                                                    random.uniform(0, 2 * math.pi),
                                                    random.uniform(0, 2 * math.pi))

        wave = mat.node_tree.nodes.new(type="ShaderNodeTexWave")
        wave.wave_type = random.choice(['RINGS', 'BANDS'])
        wave.rings_direction = 'Z'
        wave.inputs["Scale"].default_value = random.uniform(0.6, 6.0)
        wave.inputs["Distortion"].default_value = random.uniform(1.5, 3.5)

        noise = mat.node_tree.nodes.new(type="ShaderNodeTexNoise")
        noise.inputs["Scale"].default_value = random.uniform(2.0, 12.0)
        noise.inputs["Detail"].default_value = random.uniform(2.0, 6.0)
        noise.inputs["Roughness"].default_value = random.uniform(0.5, 1.0)

        vector_math = mat.node_tree.nodes.new(type="ShaderNodeVectorMath")
        vector_math.operation = 'ADD'

        mix = mat.node_tree.nodes.new(type="ShaderNodeMixRGB")
        mix.blend_type = 'MULTIPLY'
        mix.inputs['Fac'].default_value = random.uniform(0.5, 0.8)

        color_ramp = mat.node_tree.nodes.new(type='ShaderNodeValToRGB')
        color_ramp.color_ramp.elements[0].color = (0.2, 0.1, 0.0, 1)
        color_ramp.color_ramp.elements[1].color = (0.8, 0.5, 0.2, 1)

        mat.node_tree.links.new(tex_coord.outputs['Object'], mapping.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], noise.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], vector_math.inputs[0])
        mat.node_tree.links.new(noise.outputs['Fac'], vector_math.inputs[1])
        if random.choice([True, False]):
            mat.node_tree.links.new(vector_math.outputs['Vector'], wave.inputs['Vector'])
        else:
            mat.node_tree.links.new(mapping.outputs['Vector'], wave.inputs['Vector'])
        mat.node_tree.links.new(noise.outputs['Fac'], mix.inputs[1])
        mat.node_tree.links.new(wave.outputs['Color'], mix.inputs[2])
        mat.node_tree.links.new(mix.outputs['Color'], color_ramp.inputs['Fac'])
        mat.node_tree.links.new(color_ramp.outputs['Color'], bsdf.inputs['Base Color'])
        mat.node_tree.links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

        if bpy_obj.data.materials:
            bpy_obj.data.materials[0] = mat
        else:
            bpy_obj.data.materials.append(mat)

    @staticmethod
    def _add_stone_texture(bpy_obj):
        if bpy_obj is None:
            return

        mat = bpy.data.materials.new(name=_PlatformMaterial.STONE)
        mat.use_nodes = True

        for node in mat.node_tree.nodes:
            mat.node_tree.nodes.remove(node)

        output = mat.node_tree.nodes.new(type="ShaderNodeOutputMaterial")
        bsdf = mat.node_tree.nodes.new(type="ShaderNodeBsdfPrincipled")
        tex_coord = mat.node_tree.nodes.new(type='ShaderNodeTexCoord')

        mapping = mat.node_tree.nodes.new(type='ShaderNodeMapping')
        mapping.inputs['Scale'].default_value = (random.uniform(0.5, 2),
                                                 random.uniform(0.5, 2),
                                                 random.uniform(0.5, 2))
        mapping.inputs['Rotation'].default_value = (random.uniform(0, 2 * math.pi),
                                                    random.uniform(0, 2 * math.pi),
                                                    random.uniform(0, 2 * math.pi))

        musgrave = mat.node_tree.nodes.new(type="ShaderNodeTexMusgrave")
        musgrave.inputs["Scale"].default_value = random.uniform(2.0, 8.0)
        musgrave.inputs["Detail"].default_value = random.uniform(1.0, 5.0)
        musgrave.inputs["Dimension"].default_value = random.uniform(0.5, 1.5)

        voronoi = mat.node_tree.nodes.new(type="ShaderNodeTexVoronoi")
        voronoi.feature = 'DISTANCE_TO_EDGE'
        voronoi.distance = 'EUCLIDEAN'
        voronoi.inputs["Scale"].default_value = random.uniform(3.0, 10.0)

        noise = mat.node_tree.nodes.new(type="ShaderNodeTexNoise")
        noise.inputs["Scale"].default_value = random.uniform(2.0, 12.0)
        noise.inputs["Detail"].default_value = random.uniform(2.0, 6.0)
        noise.inputs["Roughness"].default_value = random.uniform(0.5, 1.0)

        mix1 = mat.node_tree.nodes.new(type="ShaderNodeMixRGB")
        mix1.blend_type = 'MULTIPLY'
        mix1.inputs['Fac'].default_value = random.uniform(0.4, 0.7)

        mix2 = mat.node_tree.nodes.new(type="ShaderNodeMixRGB")
        mix2.blend_type = 'ADD'
        mix2.inputs['Fac'].default_value = random.uniform(0.3, 0.6)

        color_ramp = mat.node_tree.nodes.new(type='ShaderNodeValToRGB')
        dark_shade = random.uniform(0.01, 0.05)
        light_shade = random.uniform(0.28, 0.4)
        color_ramp.color_ramp.elements[0].color = (dark_shade, dark_shade, dark_shade, 1)
        color_ramp.color_ramp.elements[1].color = (light_shade, light_shade, light_shade, 1)

        mat.node_tree.links.new(tex_coord.outputs['Object'], mapping.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], noise.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], musgrave.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], voronoi.inputs['Vector'])

        mat.node_tree.links.new(noise.outputs['Fac'], mix1.inputs[1])
        mat.node_tree.links.new(musgrave.outputs['Fac'], mix1.inputs[2])
        mat.node_tree.links.new(mix1.outputs['Color'], mix2.inputs[1])
        mat.node_tree.links.new(voronoi.outputs['Distance'], mix2.inputs[2])
        mat.node_tree.links.new(mix2.outputs['Color'], color_ramp.inputs['Fac'])
        mat.node_tree.links.new(color_ramp.outputs['Color'], bsdf.inputs['Base Color'])
        mat.node_tree.links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])

        if bpy_obj.data.materials:
            bpy_obj.data.materials[0] = mat
        else:
            bpy_obj.data.materials.append(mat)

    @staticmethod
    def _add_texture(platform_texture):
        StimuliBPYUtils.bpy_select_platforms(True, True, False, True)
        for obj in bpy.context.selected_objects:
            StimuliBPYUtils._add_stone_texture(obj)

        StimuliBPYUtils.bpy_select_platforms(True, True, True, False)
        for obj in bpy.context.selected_objects:
            if platform_texture == _PlatformMaterial.WOOD:
                StimuliBPYUtils._add_wood_texture(obj)
            elif platform_texture == _PlatformMaterial.STONE:
                StimuliBPYUtils._add_stone_texture(obj)

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
    def bpy_select_platforms(exclude_ground=True, exclude_agent=True, exclude_stage=True, exclude_platforms=True):
        StimuliBPYUtils.bpy_deselect_all()

        for obj in bpy.context.scene.objects:
            if not obj.type == 'MESH':
                continue

            is_ground = obj.name.startswith(_PlatformNames.GROUND)
            is_agent = obj.name.startswith(_PlatformNames.AGENT)
            is_stage = obj.name.startswith(_PlatformNames.STAGE)

            if exclude_ground and is_ground:
                continue

            if exclude_agent and is_agent:
                continue

            if exclude_stage and is_stage:
                continue

            if exclude_platforms and not (is_stage or is_agent or is_ground):
                continue

            obj.select_set(True)

    @staticmethod
    def bpy_select_all():
        bpy.ops.object.select_all(action='SELECT')

    @staticmethod
    def bpy_deselect_all():
        bpy.ops.object.select_all(action='DESELECT')

    @staticmethod
    def bpy_clear():
        for obj in bpy.data.objects:
            bpy.data.objects.remove(obj, do_unlink=True)

    @staticmethod
    def setup_scene(scene_filepath, platform_texture):
        StimuliBPYUtils.bpy_clear()

        bpy.context.scene.world.color = (0, 0, 0)

        if scene_filepath.endswith(".stl"):
            _FileUtils.import_stl_file(scene_filepath)
        elif scene_filepath.endswith(".obj"):
            _FileUtils.import_obj_file(scene_filepath)

        light_height = StimuliBPYUtils.get_stim_z_min() + abs(StimuliBPYUtils.get_stim_height()) * 10
        light_location = (StimuliBPYUtils.get_stim_center_x(), StimuliBPYUtils.get_stim_center_y(), light_height)
        StimuliBPYUtils._add_lighting(light_location=light_location)

        tracer_height = StimuliBPYUtils.get_stim_z_min() + abs(StimuliBPYUtils.get_stim_height()) * 0.75
        camera_height = StimuliBPYUtils.get_stim_z_min() + abs(StimuliBPYUtils.get_stim_height()) * 1.5
        StimuliBPYUtils._add_camera(tracer_height, 2, camera_height, 8)

        StimuliBPYUtils._add_texture(platform_texture)

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
        bpy.context.preferences.addons['cycles'].preferences.compute_device_type = 'CUDA'
        bpy.context.preferences.addons['cycles'].preferences.get_devices()
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

    out_video_dirpath = os.path.join(OUT_DIRPATH, "stim_videos")
    _FileUtils.create_dir(out_video_dirpath)

    for stimuli_item_dir_path in stimuli_item_path_list:
        stimuli_num = int(stimuli_item_dir_path.split("/")[-1].split("_")[-1])
        out_wood_filepath = os.path.join(out_video_dirpath, f"stim_wood_{stimuli_num}.mp4")
        out_stone_filepath = os.path.join(out_video_dirpath, f"stim_stone_{stimuli_num}.mp4")

        stimuli_filepath = os.path.join(stimuli_item_dir_path, "meshes", "stim.obj")

        StimuliBPYUtils.bpy_clear()
        StimuliBPYUtils.setup_scene(stimuli_filepath, _PlatformMaterial.WOOD)
        StimuliBPYUtils.render_video(out_wood_filepath)

        StimuliBPYUtils.bpy_clear()
        StimuliBPYUtils.setup_scene(stimuli_filepath, _PlatformMaterial.STONE)
        StimuliBPYUtils.render_video(out_stone_filepath)


if __name__ == '__main__':
    main()

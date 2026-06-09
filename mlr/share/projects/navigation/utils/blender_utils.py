import bmesh
import bpy
import math
import os
import random
import shutil
import sys
import xml.etree.ElementTree as ETree

from mathutils import Vector

ROOT_DIRPATH = "/home/jakiroshah/PycharmProjects/genea"
BASE_DIRPATH = os.path.join(ROOT_DIRPATH, "mlr/share/projects/navigation")
PLATFORMS_DIRPATH = os.path.join(BASE_DIRPATH, "library", "platforms")
STIMULI_DIRPATH = os.path.join(BASE_DIRPATH, "library", "stimuli")
OUT_DIRPATH = os.path.join(BASE_DIRPATH, "library", "out")
UTILS_DIRPATH = os.path.join(BASE_DIRPATH, "utils")

sys.path.insert(0, str(ROOT_DIRPATH))
sys.path.insert(0, str(UTILS_DIRPATH))

from config_utils import PlatformConfig, BlenderConfig
from core_utils import NavPosition, NavPose
from platform_utils import PlatformShape, PlatformScale, PlatformMaterial, PlatformType, Platform


PLATFORM_NAME = "platform_name"


class _FileUtils:
    @staticmethod
    def is_dir(dirpath: str):
        return os.path.isdir(dirpath)

    @staticmethod
    def create_dir(dirpath: str, do_force_create=False):
        if _FileUtils.is_dir(dirpath):
            if do_force_create:
                shutil.rmtree(dirpath)
                os.makedirs(dirpath)
            return

        os.makedirs(dirpath)

    @staticmethod
    def get_dir_list_in_directory(dirpath: str):
        try:
            return list(sorted(filter(os.path.isdir, [os.path.join(dirpath, i) for i in os.listdir(dirpath)])))
        except Exception as e:
            print("ERROR: issue in retrieving directory list within given directory " + dirpath)
            print(str(e))
            assert False


class BPYUtils:
    @staticmethod
    def _import_stl_file(filepath: str):
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"ERROR [video_utils]: file not found: {filepath}")
        bpy.ops.import_mesh.stl(filepath=filepath)

    @staticmethod
    def _import_mjcf_file(scene_dirpath: str, mjcf_filepath: str):
        tree = ETree.parse(mjcf_filepath)
        root = tree.getroot()

        mjcf_mesh_dict = {}
        for mesh in root.findall(".//asset/mesh"):
            mjcf_mesh_dict[mesh.attrib["name"]] = os.path.join(scene_dirpath, "meshes", mesh.attrib["file"])

        for body in root.findall(".//worldbody/body"):
            body_name = body.attrib.get("name", "no_name")
            body_pos = list(map(float, body.attrib.get("pos", "0 0 0").split()))
            body_rot = list(map(float, body.attrib.get("euler", "0 0 0").split()))

            geom = body.find("geom")
            if geom is None:
                continue

            geom_type = geom.attrib.get("type")

            if geom_type == "mesh":
                mesh_filepath = mjcf_mesh_dict[geom.attrib["mesh"]]
                if mesh_filepath.endswith(".stl"):
                    BPYUtils._import_stl_file(mesh_filepath)

                obj = bpy.context.selected_objects[0]
                obj.name = body_name
                obj.location = body_pos
                obj.rotation_euler = body_rot
                obj[PLATFORM_NAME] = body_name

            elif geom_type == "box":
                body_scale = list(map(float, geom.attrib["size"].split()))
                bpy.ops.mesh.primitive_cube_add(location=body_pos)
                obj = bpy.context.object
                obj.name = body_name
                obj.scale = body_scale
                obj.rotation_euler = body_rot
                obj[PLATFORM_NAME] = body_name

            else:
                print(f"ERROR [BPYUtils]: encountered an unsupported geom type -- {geom_type} for {body_name})")
                assert False

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
        minor_axis = (BPYUtils.get_stim_y_max() - BPYUtils.get_stim_y_min()) * shape_multiplier

        diff_factor = (BPYUtils.get_stim_x_max() - BPYUtils.get_stim_x_min()) / 3

        l_center_x = BPYUtils.get_stim_x_min() + diff_factor
        l_center_y = BPYUtils.get_stim_center_y()

        r_center_x = BPYUtils.get_stim_x_max() - diff_factor
        r_center_y = BPYUtils.get_stim_center_y()

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

        new_points = BPYUtils._get_chaikin_curve(points_on_curve, total_waypoints + 1, 3)
        for frame, (point_x, point_y) in enumerate(new_points, start=1):
            blender_obj.location = (point_x, point_y, stadium_height)
            blender_obj.keyframe_insert(data_path="location", frame=frame)

    @staticmethod
    def _add_camera(tracer_height: float, tracer_radius: int, cam_height: float, cam_radius: int):
        bpy.ops.object.empty_add()
        tracer = bpy.context.active_object

        bpy.ops.object.camera_add()
        camera = bpy.context.object
        bpy.context.scene.camera = camera

        BPYUtils._set_camera_trajectory(tracer, tracer_radius, tracer_height, BlenderConfig.VIDEO_FRAME_COUNT)
        BPYUtils._set_camera_trajectory(camera, cam_radius, cam_height, BlenderConfig.VIDEO_FRAME_COUNT)

        camera.constraints.new(type='TRACK_TO')
        camera.constraints["Track To"].target = tracer
        camera.constraints["Track To"].track_axis = 'TRACK_NEGATIVE_Z'
        camera.constraints["Track To"].up_axis = 'UP_Y'

    @staticmethod
    def _add_lighting(light_location=(0., 0., 0.), light_energy=4000.0, light_size=40):
        bpy.ops.object.light_add(type='AREA', align='WORLD', location=light_location)
        area = bpy.context.object
        area.data.energy = light_energy * 2
        area.data.size = light_size

        bpy.ops.object.light_add(type='AREA', align='WORLD', location=(0, -30, 0))
        area = bpy.context.object
        area.data.energy = light_energy / 1.25
        area.data.size = light_size
        area.rotation_euler[0] = math.radians(90)

    @staticmethod
    def _add_texture_woody(bpy_obj, is_darker=False):
        if bpy_obj is None:
            return

        mat = bpy.data.materials.new(name=PlatformMaterial.WOODY)
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
        wave.inputs["Scale"].default_value = random.uniform(0.5, 1.5)
        wave.inputs["Distortion"].default_value = random.uniform(1.0, 4.5)

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
        color_ramp.color_ramp.elements[0].color = (0.75, 0.4, 0.25, 1)
        color_ramp.color_ramp.elements[1].color = (0.95, 0.85, 0.7, 1)
        if is_darker:
            color_ramp.color_ramp.elements[0].color = (0.005, 0.001, 0.00, 1)
            color_ramp.color_ramp.elements[1].color = (0.018, 0.008, 0.002, 1)

        mat.node_tree.links.new(tex_coord.outputs['Object'], mapping.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], noise.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], vector_math.inputs[0])
        mat.node_tree.links.new(noise.outputs['Fac'], vector_math.inputs[1])
        if is_darker:
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
    def _add_texture_stone(bpy_obj):
        if bpy_obj is None:
            return

        mat = bpy.data.materials.new(name=PlatformMaterial.STONE)
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
    def _add_color(bpy_obj, color_name: str, color_rgba: tuple):
        bpy_material = bpy.data.materials.new(name=color_name)
        bpy_material.use_nodes = True

        bsdf = bpy_material.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs['Base Color'].default_value = color_rgba

        if bpy_obj.data.materials:
            bpy_obj.data.materials[0] = bpy_material
        else:
            bpy_obj.data.materials.append(bpy_material)

    @staticmethod
    def add_material(obj, material):
        if material == PlatformMaterial.WOODY:
            BPYUtils._add_texture_woody(obj)
        elif material == PlatformMaterial.STONE:
            BPYUtils._add_texture_stone(obj)
        elif material == PlatformMaterial.BLAND:
            BPYUtils._add_color(obj, "bland", (0.625, 0.625, 0.625, 1.0))
        elif material == PlatformMaterial.BRAWN:
            BPYUtils._add_texture_woody(obj, True)
        else:
            raise ValueError(f"ERROR [BPYUtils]: invalid material -- {material}")

    @staticmethod
    def bpy_clear():
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)

    @staticmethod
    def bpy_select_all():
        bpy.ops.object.select_all(action='SELECT')

    @staticmethod
    def bpy_deselect_all():
        bpy.ops.object.select_all(action='DESELECT')

    @staticmethod
    def bpy_object_mode():
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

    @staticmethod
    def bpy_edit_mode():
        if bpy.context.object and bpy.context.object.mode != 'EDIT':
            bpy.ops.object.mode_set(mode='EDIT')

    @staticmethod
    def save_scene_as_stl(stl_filepath: str):
        BPYUtils.bpy_object_mode()
        bpy.ops.export_mesh.stl(filepath=stl_filepath, use_selection=True)

    @staticmethod
    def render_video(out_video_filepath: str):
        scene = bpy.context.scene
        scene.render.filepath = out_video_filepath
        scene.render.image_settings.file_format = 'FFMPEG'
        scene.render.ffmpeg.format = 'MPEG4'
        scene.frame_end = BlenderConfig.VIDEO_FRAME_COUNT

        scene.render.fps = 24
        scene.render.resolution_x = 1920
        scene.render.resolution_y = 1080
        scene.render.resolution_percentage = 100

        bpy.context.scene.cycles.device = 'GPU'
        bpy.context.preferences.addons['cycles'].preferences.compute_device_type = 'CUDA'
        bpy.context.preferences.addons['cycles'].preferences.get_devices()
        bpy.ops.render.render(animation=True)

    @staticmethod
    def render_frame(out_path: str, frame: int):
        scene = bpy.context.scene
        scene.frame_set(frame)
        scene.render.filepath = out_path
        scene.render.image_settings.file_format = 'PNG'
        bpy.ops.render.render(write_still=True)

    @staticmethod
    def load_scene(scene_dirpath, scene_filepath: str):
        BPYUtils.bpy_clear()

        world = bpy.context.scene.world
        world.use_nodes = True
        bg = world.node_tree.nodes["Background"]
        bg.inputs[0].default_value = (0.04, 0.04, 0.04, 1)
        bg.inputs[1].default_value = 1.0

        if scene_filepath.endswith(".stl"):
            BPYUtils._import_stl_file(scene_filepath)
        elif scene_filepath.endswith(".mjcf"):
            BPYUtils._import_mjcf_file(scene_dirpath, scene_filepath)
        else:
            print(f"ERROR [BPYUtils]: invalid scene type -- {scene_filepath}")
            assert False

        light_height = BPYUtils.get_stim_z_min() + abs(BPYUtils.get_stim_height()) * 10
        light_location = (BPYUtils.get_stim_center_x(), BPYUtils.get_stim_center_y(), light_height)
        BPYUtils._add_lighting(light_location=light_location)

        tracer_height = BPYUtils.get_stim_z_min() + abs(BPYUtils.get_stim_height()) * 1.0
        camera_height = BPYUtils.get_stim_z_min() + abs(BPYUtils.get_stim_height()) * 2.2
        BPYUtils._add_camera(tracer_height, 2, camera_height, 8)

        for bpy_obj in bpy.context.scene.objects:
            if PLATFORM_NAME in bpy_obj.keys():
                obj_type = bpy_obj.get(PLATFORM_NAME).split("_")[2]
                BPYUtils.add_material(bpy_obj, obj_type)

    @staticmethod
    def _get_stim_bounds():
        BPYUtils.bpy_select_all()

        corners_list = []
        for obj in bpy.context.scene.objects:
            if not obj.type == 'MESH':
                continue

            if obj.name.startswith(PlatformShape.GROUND):
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
    def get_stim_x_min():
        return BPYUtils._get_stim_bounds()["x_min"]

    @staticmethod
    def get_stim_x_max():
        return BPYUtils._get_stim_bounds()["x_max"]

    @staticmethod
    def get_stim_y_min():
        return BPYUtils._get_stim_bounds()["y_min"]

    @staticmethod
    def get_stim_y_max():
        return BPYUtils._get_stim_bounds()["y_max"]

    @staticmethod
    def get_stim_z_min():
        return BPYUtils._get_stim_bounds()["z_min"]

    @staticmethod
    def get_stim_z_max():
        return BPYUtils._get_stim_bounds()["z_max"]

    @staticmethod
    def get_stim_center_x():
        return BPYUtils.get_stim_x_min() + (BPYUtils.get_stim_x_max() + BPYUtils.get_stim_x_min()) / 2

    @staticmethod
    def get_stim_center_y():
        return (BPYUtils.get_stim_y_max() + BPYUtils.get_stim_y_min()) / 2

    @staticmethod
    def get_stim_height():
        return BPYUtils.get_stim_z_max() - BPYUtils.get_stim_z_min()


class RoundedCuboidPlatform(Platform):
    def __init__(self, platform_type: PlatformType):
        super().__init__("", platform_type, NavPose(NavPosition(0.0, 0.0, 0.0)))

    def save_as_stl(self):
        BPYUtils.save_scene_as_stl(os.path.join(PLATFORMS_DIRPATH, self.get_platform_mesh_filename()))

    def create_platform(self):
        bpy.ops.mesh.primitive_cube_add(location=self.get_platform_pose().get_position().get_position_as_list(),
                                        rotation=self.get_platform_pose().get_rotation().get_rotation_as_list(),
                                        size=1)

        obj = bpy.context.object

        obj.scale = (self.get_platform_bounding_box())
        bpy.ops.object.transform_apply(scale=True)

        obj.location.z += self.get_platform_height() / 2
        bpy.ops.object.transform_apply(location=True)

        BPYUtils.add_material(obj, self.get_platform_type().get_material())

        BPYUtils.bpy_edit_mode()

        mesh = bmesh.from_edit_mesh(obj.data)
        mesh.edges.index_update()
        vertical_edges = []

        for edge in mesh.edges:
            v1, v2 = edge.verts
            delta = v2.co - v1.co
            is_vertical = abs(delta.x) < 1e-5 and abs(delta.y) < 1e-5 and abs(delta.z) > 0
            if is_vertical:
                vertical_edges.append(edge)

        bmesh.ops.recalc_face_normals(mesh, faces=mesh.faces)
        bmesh.ops.bevel(mesh,
                        geom=vertical_edges,
                        offset=PlatformConfig.PLATFORM_BEVEL_WIDTH,
                        segments=PlatformConfig.PLATFORM_BEVEL_SEGMENTS,
                        profile=.5,
                        affect='EDGES')

        top_scale = PlatformConfig.PLATFORM_SCALE_TOP_FACTOR if self.is_top_scaled() else 1.0
        bot_scale = PlatformConfig.PLATFORM_SCALE_BOT_FACTOR if self.is_bot_scaled() else 1.0

        mesh_top_verts = max(v.co.z for v in mesh.verts)
        mesh_bot_verts = min(v.co.z for v in mesh.verts)
        for v in mesh.verts:
            if abs(v.co.z - mesh_top_verts) < 1e-5:
                v.co.x *= top_scale
                v.co.y *= top_scale
            elif abs(v.co.z - mesh_bot_verts) < 1e-5:
                v.co.x *= bot_scale
                v.co.y *= bot_scale

        bmesh.update_edit_mesh(obj.data)
        BPYUtils.bpy_object_mode()


def make_platforms():
    # ====================== ROOT and GOAL ======================
    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.ROOT, PlatformScale.NOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.GOAL, PlatformScale.NOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    # ====================== BASE ======================
    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.BASE, PlatformScale.NOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.BASE, PlatformScale.TOP_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.BASE, PlatformScale.BOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    # ====================== WIDE ======================
    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.WIDE, PlatformScale.NOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.WIDE, PlatformScale.TOP_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.WIDE, PlatformScale.BOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    # ====================== LONG ======================
    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.LONG, PlatformScale.NOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.LONG, PlatformScale.TOP_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.LONG, PlatformScale.BOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    # ====================== WALK ======================
    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.WALK, PlatformScale.NOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.WALK, PlatformScale.TOP_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()

    BPYUtils.bpy_clear()
    platform_type = PlatformType(PlatformShape.WALK, PlatformScale.BOT_SCALED, PlatformMaterial.BLAND)
    platform = RoundedCuboidPlatform(platform_type)
    platform.create_platform()
    platform.save_as_stl()


def make_stimuli(stimuli_set_name: str, save_as_img=False, save_as_video=False):
    stimuli_path_list = _FileUtils.get_dir_list_in_directory(os.path.join(STIMULI_DIRPATH, stimuli_set_name))
    for stimulus_dir_path in stimuli_path_list:
        stimuli_num = int(stimulus_dir_path.split("/")[-1].split("_")[-1])

        BPYUtils.load_scene(stimulus_dir_path, os.path.join(stimulus_dir_path, "stimulus.mjcf"))

        if save_as_img:
            out_img_dirpath = os.path.join(OUT_DIRPATH, "stim_images")
            out_img_filepath = os.path.join(out_img_dirpath, f"stim_{stimuli_num}.png")
            _FileUtils.create_dir(out_img_dirpath)
            BPYUtils.render_frame(out_img_filepath, 2 * (BlenderConfig.VIDEO_FRAME_COUNT // 5) - 5)

        if save_as_video:
            out_vid_dirpath = os.path.join(OUT_DIRPATH, "stim_videos")
            out_vid_filepath = os.path.join(out_vid_dirpath, f"stim_{stimuli_num}.mp4")
            _FileUtils.create_dir(out_vid_dirpath)
            BPYUtils.render_video(out_vid_filepath)


def main():
    # make_platforms()
    make_stimuli("diff", save_as_img=True, save_as_video=False)


if __name__ == '__main__':
    main()

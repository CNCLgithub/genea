import bpy
import os
import math
from mathutils import Vector

# PARAMETERS TO CHANGE
FRAME_COUNT = 300
DIR = "C:/Users/ldebb/Desktop/ISC Lab/Projects/Embodied_plans"
STL_PATH = DIR + "/Obstacles/Scenes/All_scenes/Scene5.stl"
# change this according to width of scene, larger EMPTY_DIST = longer scene
EMPTY_DIST = 3
# change this according to zoom: smaller CAMERA_DIST = more zoomed out
CAMERA_DIST = 1


def _import_stl(filepath):
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File not found: {filepath}")
    bpy.ops.wm.stl_import(filepath=filepath)


def _get_bounds():
    obj = bpy.context.active_object

    corners = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]
    x = [v.x for v in corners]
    y = [v.y for v in corners]
    z = [v.z for v in corners]

    return {"x": (min(x), max(x)), "y": (min(y), max(y)), "z": (min(z), max(z))}


def _create_ellipse(obj, x_bounds, y_bounds, distance, height, frame_count):
    major_axis = (x_bounds[1] - x_bounds[0]) / distance
    minor_axis = (y_bounds[1] - y_bounds[0]) / distance
    center_x = (x_bounds[1] + x_bounds[0]) / 2
    center_y = (y_bounds[1] + y_bounds[0]) / 2
    z = height

    for i in range(frame_count + 1):
        theta = (2 * math.pi * i) / frame_count
        x = center_x + major_axis * math.cos(theta)
        y = center_y + minor_axis * math.sin(theta)
        obj.location = (x, y, z)
        obj.keyframe_insert(data_path="location", frame=1 + i)


class SetMaterials:
    def __init__(self, color=(1, 1, 1, 1)):
        self.material_name = "Material"
        self.color = color
        self.material = None

    def create_or_get_material(self):
        mat = bpy.data.materials.get(self.material_name)
        if mat is None:
            mat = bpy.data.materials.new(name=self.material_name)

        mat.use_nodes = True
        nodes = mat.node_tree.nodes

        bsdf = nodes.get("Principled BSDF")
        if bsdf is None:
            bsdf = nodes.new(type="ShaderNodeBsdfPrincipled")
            bsdf.location = (0, 0)

        bsdf.inputs[0].default_value = self.color

        self.material = mat
        return mat

    def assign_to_object(self, obj):
        if self.material is None:
            self.create_or_get_material()

        if obj.data.materials:
            obj.data.materials[0] = self.material
        else:
            obj.data.materials.append(self.material)


class SceneLighting:
    def __init__(self):
        pass

    def create_light(self, name, light_type, energy, location, rotation=(0, 0, 0)):
        light_data = bpy.data.lights.new(name=name, type=light_type)
        light_data.energy = energy

        light_object = bpy.data.objects.new(name=name, object_data=light_data)
        bpy.context.collection.objects.link(light_object)
        light_object.location = location
        light_object.rotation_euler = rotation

        return light_object

    def set_basic_light(self):
        # Main light
        self.create_light(
            name="KeyLight",
            light_type="SUN",
            energy=5.0,
            location=(10, -10, 10),
            rotation=(0.7, 0.0, 0.7)
        )

        # Fill light (softens shadows)
        self.create_light(
            name="FillLight",
            light_type="POINT",
            energy=1000.0,
            location=(-5, 5, 5)
        )


class CameraSetup:
    def __init__(self, target, x_bounds, y_bounds, z_bounds, frame_count):
        self.target = target
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.z_bounds = z_bounds
        self.frame_count = frame_count
        self.empty = None
        self.camera = None

    def create_empty(self):
        bpy.ops.object.empty_add()
        self.empty = bpy.context.active_object
        _create_ellipse(self.empty, self.x_bounds, self.y_bounds, EMPTY_DIST, 0, self.frame_count)

    def create_camera(self):
        bpy.ops.object.camera_add()
        self.camera = bpy.context.active_object
        height = self.z_bounds[1] * 4
        _create_ellipse(self.camera, self.x_bounds, self.y_bounds, CAMERA_DIST, height, self.frame_count)

        if self.empty:
            self.camera.constraints.new(type='TRACK_TO')
            self.camera.constraints["Track To"].target = self.empty
            self.camera.constraints["Track To"].track_axis = 'TRACK_NEGATIVE_Z'
            self.camera.constraints["Track To"].up_axis = 'UP_Y'


class SetScene:
    def __init__(self, stl_path, frame_count):
        self.stl_path = stl_path
        self.frame_count = frame_count
        self.scene = None
        self.obj = None
        self.x_bounds = None
        self.y_bounds = None
        self.z_bounds = None
        self.empty = None

    def setup(self):
        self.clear_scene()
        self.load_scene()
        self.apply_material()
        self.setup_lighting()
        self.setup_camera()

    def clear_scene(self):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)

    def load_scene(self):
        _import_stl(self.stl_path)
        self.obj = bpy.context.active_object

        bounds = _get_bounds()
        self.x_bounds = bounds["x"]
        self.y_bounds = bounds["y"]
        self.z_bounds = bounds["z"]

    def apply_material(self):
        set_materials = SetMaterials()
        set_materials.create_or_get_material()
        set_materials.assign_to_object(bpy.context.active_object)

    def setup_lighting(self):
        lighting = SceneLighting()
        lighting.set_basic_light()

    def setup_camera(self):
        camera_setup = CameraSetup(target=self.empty, x_bounds=self.x_bounds, y_bounds=self.y_bounds,
                                   z_bounds=self.z_bounds, frame_count=FRAME_COUNT)
        camera_setup.create_empty()
        camera_setup.create_camera()
        self.empty = camera_setup.empty


class VideoRenderer:
    def __init__(self, output_path, file_name="animation", fps=24, resolution=(1920, 1080)):
        self.scene = bpy.context.scene
        self.scene.frame_end = FRAME_COUNT
        self.output_path = output_path
        self.file_name = file_name
        self.fps = fps
        self.resolution_x, self.resolution_y = resolution

    def configure_render_settings(self):
        self.scene.render.filepath = os.path.join(self.output_path, self.file_name)
        self.scene.render.image_settings.file_format = 'FFMPEG'
        self.scene.render.ffmpeg.format = 'MPEG4'
        self.scene.render.fps = self.fps
        self.scene.render.resolution_x = self.resolution_x
        self.scene.render.resolution_y = self.resolution_y
        self.scene.render.resolution_percentage = 100

    def render(self):
        self.configure_render_settings()
        bpy.ops.render.render(animation=True)


def main():
    # Set Scene
    set_scene = SetScene(STL_PATH, FRAME_COUNT)
    set_scene.setup()

    # Render video
    renderer = VideoRenderer(output_path=DIR)
    # UNCOMMENT THIS NEXT LINE TO MAKE VIDEO
#   renderer.render()


if __name__ == "__main__":
    main()

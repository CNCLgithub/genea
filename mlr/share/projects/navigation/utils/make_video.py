import bpy
import os
from mathutils import Vector

FRAME_COUNT = 150
DIR =  "C:/Users/ldebb/Desktop/Yale Job/Projects/Embodied_plans/"
STL_PATH = DIR + "/Obstacles/Scenes/All_scenes/Scene4.stl"
CAMERA_DIST = 15

class FileLoader:
    def __init__(self, filepath):
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"File not found: {filepath}") 
        self.filepath = filepath
            
    def import_stl(self):
        bpy.ops.wm.stl_import(filepath=self.filepath)
                
      
class SceneBounds:
    def __init__(self, filepath):
        self.loader = FileLoader(filepath)
        self.obj = None
        self.bounds = {}

    def load(self):
        self.loader.import_stl()
        self.obj = bpy.context.active_object
        self.bounds = self._get_bounds()

    def _get_bounds(self):
        corners = [self.obj.matrix_world @ Vector(corner) for corner in self.obj.bound_box]
        x = [v.x for v in corners]
        y = [v.y for v in corners]
        z = [v.z for v in corners]
        return {"x": (min(x), max(x)), "y": (min(y), max(y)), "z": (min(z), max(z))}

    def get_bounds(self):
        return self.bounds["x"], self.bounds["y"], self.bounds["z"]

    def get_object(self):
        return self.obj
    
class SetMaterials:
    def __init__(self, material_name="Material", color=(1, 1, 1, 1)):
        self.material_name = material_name
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


class AnimateArc:
    def __init__(self, obj, x_bounds, y_bounds, z_bounds, frame_count):
        self.obj = obj
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.z_bounds = z_bounds
        self.frame_count = frame_count

    def _parabola(self, t, y_peak):
        return 4 * y_peak * t * (1 - t)

    def animate(self):
        x0 = -self.x_bounds[0]
        x1 = -self.x_bounds[1]
        y_peak = self.y_bounds[1] + CAMERA_DIST
        z = self.z_bounds[1]/4*3

        total_frames = self.frame_count * 2

        for i in range(total_frames + 1):
            if i < self.frame_count:
                t = i / self.frame_count
                x = (1 - t) * x0 + t * x1
            else: # reverse direction
                t = (i - self.frame_count) / self.frame_count
                x = (1 - t) * x1 + t * x0
                t = 1 - t  

            y = self._parabola(t, y_peak)
            self.obj.location = (x, y, z)
            self.obj.keyframe_insert(data_path="location", frame=1 + i)


class CameraSetup:
    def __init__(self, target, y_bounds, z_bounds):
        self.target = target
        self.y_bounds = y_bounds
        self.z_bounds = z_bounds
        self.camera = None

    def create_camera(self):
        bpy.ops.object.camera_add()
        self.camera = bpy.context.active_object
        self.camera.location.z = self.z_bounds[1]
        self.camera.location.y = self.y_bounds[1] + CAMERA_DIST*1.5

        constraint = self.camera.constraints.new(type="TRACK_TO")
        constraint.target = self.target
        constraint.track_axis = 'TRACK_NEGATIVE_Z'
        constraint.up_axis = 'UP_Y'

        bpy.context.scene.camera = self.camera
        
        bpy.context.scene.frame_end = FRAME_COUNT*2


class SetScene:
    def __init__(self, stl_path, frame_count, material_name="White"):
        self.stl_path = stl_path
        self.frame_count = frame_count
        self.material_name = material_name
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
        self.add_empty_and_animate()
        self.setup_camera()

    def clear_scene(self):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)

    def load_scene(self):
        self.scene = SceneBounds(self.stl_path)
        self.scene.load()
        self.obj = self.scene.get_object()
        self.x_bounds, self.y_bounds, self.z_bounds = self.scene.get_bounds()

    def apply_material(self):
        set_materials = SetMaterials(material_name=self.material_name)
        set_materials.create_or_get_material()
        set_materials.assign_to_object(bpy.context.active_object)

    def setup_lighting(self):
        lighting = SceneLighting()
        lighting.set_basic_light()

    def add_empty_and_animate(self):
        bpy.ops.object.empty_add()
        self.empty = bpy.context.active_object
        arc = AnimateArc(self.empty, self.x_bounds, self.y_bounds, self.z_bounds, self.frame_count)
        arc.animate()

    def setup_camera(self):
        camera_setup = CameraSetup(target=self.empty, y_bounds=self.y_bounds, z_bounds=self.z_bounds)
        camera_setup.create_camera()
        

class VideoRenderer:
    def __init__(self, output_path, file_name="animation", fps=24, resolution=(1920, 1080)):
        self.scene = bpy.context.scene
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
    set_scene = SetScene(STL_PATH, FRAME_COUNT, material_name = "White")
    set_scene.setup()
    
    # Render video
    renderer = VideoRenderer(output_path=DIR)
    # renderer.render()


if __name__ == "__main__":
    main()

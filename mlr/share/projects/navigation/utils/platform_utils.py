import numpy as np
import os
import shutil

try:
    import bmesh
    import bpy

    BASE_PATH = os.path.dirname(os.path.dirname(bpy.data.texts.get(bpy.context.space_data.text.name).filepath))
    PLATFORM_DIRPATH = os.path.join(BASE_PATH, "library", "platforms")
except ImportError:
    print("Unable to import or load Blender Python API.")


class _FileUtils:
    @staticmethod
    def is_dir(dirpath):
        return os.path.isdir(dirpath)

    @staticmethod
    def create_dir(dirpath, do_force_create=False):
        if _FileUtils.is_dir(dirpath):
            if do_force_create:
                shutil.rmtree(dirpath)
                os.makedirs(dirpath)
            return

        os.makedirs(dirpath)


class PlatformType:
    CUBOIDAL_CUBE = "cube"
    CUBOIDAL_WIDE = "wide"
    CUBOIDAL_LONG = "long"

    NOT_SCALED = "std"
    TOP_SCALED = "top"
    BOT_SCALED = "bot"

    @staticmethod
    def get_platform_name(shape_type, scale_type):
        return shape_type + "_" + str(scale_type)


class Platform:
    PLATFORM_MASS = 7.0

    PLATFORM_SIZE_NORMAL = 4.0
    PLATFORM_SIZE_SCALED = 2.4
    PLATFORM_HEIGHT = 4.0

    PLATFORM_BEVEL_WIDTH = 0.75
    PLATFORM_BEVEL_SEGMENTS = 20

    def __init__(self, platform_name, platform_pose, platform_color=None):
        self._platform_name = platform_name
        self._platform_pose = platform_pose
        self._platform_color = platform_color

    def get_platform_name(self):
        return self._platform_name

    def get_platform_pose(self):
        return self._platform_pose

    def get_platform_color(self):
        return self._platform_color

    def get_platform_filepath(self):
        return os.path.join(PLATFORM_DIRPATH, self._platform_name + ".stl")

    def create_platform(self, *platform_args):
        pass

    def get_platform_surface_measures(self):
        part1 = self._platform_name.split("_")[1]
        part2 = self._platform_name.split("_")[2]

        if part1 == PlatformType.CUBOIDAL_CUBE:
            if part2 == PlatformType.NOT_SCALED:
                return Platform.PLATFORM_SIZE_NORMAL, Platform.PLATFORM_SIZE_NORMAL
            if part2 == PlatformType.TOP_SCALED:
                return Platform.PLATFORM_SIZE_NORMAL / 2.0, Platform.PLATFORM_SIZE_NORMAL / 2.0
            if part2 == PlatformType.BOT_SCALED:
                return Platform.PLATFORM_SIZE_NORMAL, Platform.PLATFORM_SIZE_NORMAL

        if part1 == PlatformType.CUBOIDAL_WIDE:
            if part2 == PlatformType.NOT_SCALED:
                return Platform.PLATFORM_SIZE_SCALED, Platform.PLATFORM_SIZE_NORMAL
            if part2 == PlatformType.TOP_SCALED:
                return Platform.PLATFORM_SIZE_SCALED / 2.0, Platform.PLATFORM_SIZE_NORMAL / 2.0
            if part2 == PlatformType.BOT_SCALED:
                return Platform.PLATFORM_SIZE_SCALED, Platform.PLATFORM_SIZE_NORMAL

        if part1 == PlatformType.CUBOIDAL_LONG:
            if part2 == PlatformType.NOT_SCALED:
                return Platform.PLATFORM_SIZE_NORMAL, Platform.PLATFORM_SIZE_SCALED
            if part2 == PlatformType.TOP_SCALED:
                return Platform.PLATFORM_SIZE_NORMAL / 2.0, Platform.PLATFORM_SIZE_SCALED / 2.0
            if part2 == PlatformType.BOT_SCALED:
                return Platform.PLATFORM_SIZE_NORMAL, Platform.PLATFORM_SIZE_SCALED

        return self._platform_pose.get_rotation().get_rotation_as_np_array()[2]


class PlatformBPYUtils:
    @staticmethod
    def get_bpy_material(platform: Platform):
        platform_color = platform.get_platform_color()
        bpy_material = bpy.data.materials.new(name=platform_color.get_color_name())
        bpy_material.use_nodes = True

        bsdf = bpy_material.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs['Base Color'].default_value = platform_color.get_color_rgba()

        return bpy_material

    @staticmethod
    def bpy_clear():
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)

    @staticmethod
    def bpy_object_mode():
        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

    @staticmethod
    def bpy_edit_mode():
        if bpy.context.object and bpy.context.object.mode != 'EDIT':
            bpy.ops.object.mode_set(mode='EDIT')

    @staticmethod
    def load_platform_mesh(platform: Platform):
        platform_filepath = platform.get_platform_filepath()
        platform_pose = platform.get_platform_pose()
        platform_color = platform.get_platform_color()

        if bpy.context.object and bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')

        bpy.ops.import_mesh.stl(filepath=platform_filepath)

        obj = bpy.context.object
        if obj.data.materials:
            obj.data.materials[0] = PlatformBPYUtils.get_bpy_material(platform_color)
        else:
            obj.data.materials.append(PlatformBPYUtils.get_bpy_material(platform_color))

        obj.location = platform_pose.get_position().get_position_as_np_array()
        obj.rotation_euler = platform_pose.get_rotation().get_rotation_as_np_array()

    @staticmethod
    def save_platform_as_stl(platform: Platform):
        stl_filepath = platform.get_platform_filepath()

        PlatformBPYUtils.bpy_object_mode()
        bpy.ops.export_mesh.stl(filepath=stl_filepath, use_selection=True)


class PlatformPosition:
    def __init__(self, x, y, z):
        self._pos_x = x
        self._pos_y = y
        self._pos_z = z

    def get_position_as_list(self):
        return [self._pos_x, self._pos_y, self._pos_z]

    def get_position_as_np_array(self):
        return np.array(self.get_position_as_list())

    def get_position_as_str(self):
        return f"{self._pos_x} {self._pos_y} {self._pos_z}"


class PlatformRotation:
    def __init__(self, roll, pitch, yaw):
        self._rot_roll = roll
        self._rot_pitch = pitch
        self._rot_yaw = yaw

    def get_rotation_as_list(self):
        return [self._rot_roll, self._rot_pitch, self._rot_yaw]

    def get_rotation_as_np_array(self):
        return np.array(self.get_rotation_as_list())

    def get_rotation_as_str(self):
        return f"{self._rot_roll} {self._rot_pitch} {self._rot_yaw}"


class PlatformPose:
    def __init__(self, position: PlatformPosition, rotation: PlatformRotation=PlatformRotation(0.0, 0.0, 0.0)):
        self._position = position
        self._rotation = rotation

    def get_position(self):
        return self._position

    def get_rotation(self):
        return self._rotation


class PlatformColor:
    def __init__(self, color_name, color_rgba=(0.625, 0.625, 0.625, 1.0)):
        self._color_name = color_name
        self._color_rgba = color_rgba
        
    def get_color_name(self):
        return self._color_name
    
    def get_color_rgba(self):
        return self._color_rgba


class CuboidPlatform(Platform):
    def __init__(self, platform_name, platform_pose, platform_color):
        super().__init__(platform_name, platform_pose, platform_color)

    def create_platform(self, platform_width, platform_length, platform_height, top_scale=1.0, bottom_scale=1.0):
        bpy.ops.mesh.primitive_cube_add(location=self._platform_pose.get_position().get_position_as_list(),
                                        rotation=self._platform_pose.get_rotation().get_rotation_as_list(),
                                        size=1)

        obj = bpy.context.object

        if obj.data.materials:
            obj.data.materials[0] = PlatformBPYUtils.get_bpy_material(self)
        else:
            obj.data.materials.append(PlatformBPYUtils.get_bpy_material(self))

        obj.scale = (platform_width, platform_length, platform_height)
        bpy.ops.object.transform_apply(scale=True)

        PlatformBPYUtils.bpy_edit_mode()

        mesh = bmesh.from_edit_mesh(obj.data)

        mesh_top_verts = max(v.co.z for v in mesh.verts)
        mesh_bot_verts = min(v.co.z for v in mesh.verts)
        for v in mesh.verts:
            if abs(v.co.z - mesh_top_verts) < 1e-5:
                v.co.x *= top_scale
                v.co.y *= top_scale
            elif abs(v.co.z - mesh_bot_verts) < 1e-5:
                v.co.x *= bottom_scale
                v.co.y *= bottom_scale

        bmesh.update_edit_mesh(obj.data)

        PlatformBPYUtils.bpy_object_mode()


class CylinderPlatform(Platform):
    def __init__(self, platform_name, platform_pose, platform_color):
        super().__init__(platform_name, platform_pose, platform_color)

    def create_platform(self, platform_radius, platform_height, top_scale=1.0, bottom_scale=1.0):
        bpy.ops.mesh.primitive_cylinder_add(radius=platform_radius,
                                            location=self._platform_pose.get_position().get_position_as_list(),
                                            rotation=self._platform_pose.get_rotation().get_rotation_as_list())

        obj = bpy.context.object

        if obj.data.materials:
            obj.data.materials[0] = PlatformBPYUtils.get_bpy_material(self)
        else:
            obj.data.materials.append(PlatformBPYUtils.get_bpy_material(self))

        obj.scale[2] = platform_height
        bpy.ops.object.transform_apply(scale=True)

        PlatformBPYUtils.bpy_edit_mode()

        mesh = bmesh.from_edit_mesh(obj.data)

        mesh_top_verts = max(v.co.z for v in mesh.verts)
        mesh_bot_verts = min(v.co.z for v in mesh.verts)
        for v in mesh.verts:
            if abs(v.co.z - mesh_top_verts) < 1e-5:
                v.co.x *= top_scale
                v.co.y *= top_scale
            elif abs(v.co.z - mesh_bot_verts) < 1e-5:
                v.co.x *= bottom_scale
                v.co.y *= bottom_scale

        bmesh.update_edit_mesh(obj.data)

        PlatformBPYUtils.bpy_object_mode()


class RoundedCuboidPlatform(Platform):
    def __init__(self, platform_name, platform_pose, platform_color):
        super().__init__(platform_name, platform_pose, platform_color)

    def create_platform(self, platform_width, platform_length, platform_height,
                        top_scale=1.0, bottom_scale=1.0,
                        bevel_width=.05, bevel_segments=2):
        bpy.ops.mesh.primitive_cube_add(location=self._platform_pose.get_position().get_position_as_list(),
                                        rotation=self._platform_pose.get_rotation().get_rotation_as_list(),
                                        size=1)

        obj = bpy.context.object

        if obj.data.materials:
            obj.data.materials[0] = PlatformBPYUtils.get_bpy_material(self)
        else:
            obj.data.materials.append(PlatformBPYUtils.get_bpy_material(self))

        obj.scale = (platform_width, platform_length, platform_height)
        bpy.ops.object.transform_apply(scale=True)

        PlatformBPYUtils.bpy_edit_mode()

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
        bmesh.ops.bevel(
            mesh,
            geom=vertical_edges,
            offset=bevel_width,
            segments=bevel_segments,
            profile=.5,
            affect='EDGES'
        )

        mesh_top_verts = max(v.co.z for v in mesh.verts)
        mesh_bot_verts = min(v.co.z for v in mesh.verts)
        for v in mesh.verts:
            if abs(v.co.z - mesh_top_verts) < 1e-5:
                v.co.x *= top_scale
                v.co.y *= top_scale
            elif abs(v.co.z - mesh_bot_verts) < 1e-5:
                v.co.x *= bottom_scale
                v.co.y *= bottom_scale

        bmesh.update_edit_mesh(obj.data)
        
        PlatformBPYUtils.bpy_object_mode()


def make_rounded_cubical_platforms(pose, color):
    platform_type_cube = PlatformType.CUBOIDAL_CUBE

    cube_length = Platform.PLATFORM_SIZE_NORMAL
    cube_height = Platform.PLATFORM_HEIGHT

    bw = Platform.PLATFORM_BEVEL_WIDTH
    bs = Platform.PLATFORM_BEVEL_SEGMENTS

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_cube, PlatformType.NOT_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(cube_length, cube_length, cube_height, 1., 1., bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_cube, PlatformType.TOP_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(cube_length, cube_length, cube_height, .5, 1., bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_cube, PlatformType.BOT_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(cube_length, cube_length, cube_height, 1., .5, bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)



def make_rounded_cuboidal_platforms(pose, color):
    height = Platform.PLATFORM_HEIGHT

    size_normal = Platform.PLATFORM_SIZE_NORMAL
    size_scaled = Platform.PLATFORM_SIZE_SCALED

    bw = Platform.PLATFORM_BEVEL_WIDTH
    bs = Platform.PLATFORM_BEVEL_SEGMENTS

    platform_type_wide = PlatformType.CUBOIDAL_WIDE
    platform_type_long = PlatformType.CUBOIDAL_LONG

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_wide, PlatformType.NOT_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(size_scaled, size_normal, height, 1., 1., bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_wide, PlatformType.TOP_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(size_scaled, size_normal, height, .5, 1., bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_wide, PlatformType.BOT_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(size_scaled, size_normal, height, 1., .5, bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_long, PlatformType.NOT_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(size_normal, size_scaled, height, 1., 1., bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_long, PlatformType.TOP_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(size_normal, size_scaled, height, .5, 1., bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_name = PlatformType.get_platform_name(platform_type_long, PlatformType.BOT_SCALED)
    platform = RoundedCuboidPlatform(platform_name, pose, color)
    platform.create_platform(size_normal, size_scaled, height, 1., .5, bw, bs)
    PlatformBPYUtils.save_platform_as_stl(platform)


def main():
    default_pose = PlatformPose(PlatformPosition(0.0, 0.0, 0.0))
    default_color = PlatformColor("gray", (0.625, 0.625, 0.625, 1.0))
    
    make_rounded_cubical_platforms(default_pose, default_color)
    make_rounded_cuboidal_platforms(default_pose, default_color)


if __name__ == '__main__':
    main()

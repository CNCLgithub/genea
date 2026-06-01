import bmesh
import bpy
import numpy as np
import os
import shutil
import sys

BASE_DIRPATH = "/home/jakiroshah/PycharmProjects/genea/mlr/share/projects/navigation"
UTILS_DIRPATH = os.path.join(BASE_DIRPATH, "utils")
sys.path.insert(0, str(UTILS_DIRPATH))

from core_utils import NavColor, NavPosition, NavPose
from platform_utils import PlatformType, Platform


LIB_PLATFORM_DIRPATH = os.path.join(BASE_DIRPATH, "library", "platforms")


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
        platform_filepath = os.path.join(LIB_PLATFORM_DIRPATH, platform.get_platform_type())
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
        stl_filepath = os.path.join(LIB_PLATFORM_DIRPATH, platform.get_platform_type())
        PlatformBPYUtils.bpy_object_mode()
        bpy.ops.export_mesh.stl(filepath=stl_filepath, use_selection=True)


class RoundedCuboidPlatform(Platform):
    def __init__(self, platform_type: str, platform_pose: NavPose, platform_color: NavColor):
        super().__init__(None, platform_type, platform_pose, platform_color)

    def create_platform(self, platform_length: float, platform_width: float,
                        platform_height=Platform.PLATFORM_HEIGHT,
                        bevel_width=Platform.PLATFORM_BEVEL_WIDTH,
                        bevel_segments=Platform.PLATFORM_BEVEL_SEGMENTS):
        bpy.ops.mesh.primitive_cube_add(location=self.get_platform_pose().get_position().get_position_as_list(),
                                        rotation=self.get_platform_pose().get_rotation().get_rotation_as_list(),
                                        size=1)

        obj = bpy.context.object

        if obj.data.materials:
            obj.data.materials[0] = PlatformBPYUtils.get_bpy_material(self)
        else:
            obj.data.materials.append(PlatformBPYUtils.get_bpy_material(self))

        obj.scale = (platform_length, platform_width, platform_height)
        bpy.ops.object.transform_apply(scale=True)

        obj.location.z += platform_height / 2
        bpy.ops.object.transform_apply(location=True)

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

        top_scale = Platform.PLATFORM_SCALE_FACTOR if self.is_top_scaled() else 1.0
        bot_scale = Platform.PLATFORM_SCALE_FACTOR if self.is_bot_scaled() else 1.0

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

        PlatformBPYUtils.bpy_object_mode()


def make_start_final_platforms(pose, color):
    platform_x, platform_y = Platform.PLATFORM_SIZE_NORM, Platform.PLATFORM_SIZE_LONG

    PlatformBPYUtils.bpy_clear()
    platform = RoundedCuboidPlatform(Platform.PLATFORM_ROOT, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform = RoundedCuboidPlatform(Platform.PLATFORM_GOAL, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)


def make_rounded_cuboidal_platforms(pose, color):
    # ====================== NORM ======================
    platform_type = PlatformType.NORM
    platform_x, platform_y = Platform.PLATFORM_SIZE_NORM, Platform.PLATFORM_SIZE_NORM

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.NOT_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.TOP_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.BOT_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    # ====================== WIDE ======================
    platform_type = PlatformType.WIDE
    platform_x, platform_y = Platform.PLATFORM_SIZE_LITE, Platform.PLATFORM_SIZE_LONG

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.NOT_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.TOP_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.BOT_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    # ====================== LONG ======================
    platform_type = PlatformType.LONG
    platform_x, platform_y = Platform.PLATFORM_SIZE_LONG, Platform.PLATFORM_SIZE_LITE

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.NOT_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.TOP_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)

    PlatformBPYUtils.bpy_clear()
    platform_type = PlatformType.get_platform_type(platform_type, PlatformType.BOT_SCALED)
    platform = RoundedCuboidPlatform(platform_type, pose, color)
    platform.create_platform(platform_x, platform_y)
    PlatformBPYUtils.save_platform_as_stl(platform)


def main():
    default_pose = NavPose(NavPosition(0.0, 0.0, 0.0))
    default_color = NavColor("gray", (0.625, 0.625, 0.625, 1.0))

    make_start_final_platforms(default_pose, default_color)
    make_rounded_cuboidal_platforms(default_pose, default_color)


if __name__ == '__main__':
    main()

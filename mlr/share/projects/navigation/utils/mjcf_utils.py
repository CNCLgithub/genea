from __future__ import annotations

import xml.etree.ElementTree as ETree

from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.mujoco_utils import MujocoUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils


class _MJCFTag:
    MJCF = "mujoco"
    MESH = "mesh"
    BODY = "body"
    GEOM = "geom"
    ASSET = "asset"
    LIGHT = "light"
    CAMERA = "camera"
    OPTION = "option"
    COMPILER = "compiler"
    WORLD_BODY = "worldbody"
    FREE_JOINT = "freejoint"


class _MJCFAttr:
    NAME = "name"
    TYPE = "type"
    MESH = "mesh"
    MASS = "mass"
    RGBA = "rgba"
    FILE = "file"
    BOX = "box"
    POS = "pos"
    SIZE = "size"
    ROT = "euler"
    XYAXES = "xyaxes"
    DIR = "dir"
    ANGLE = "angle"
    MESHDIR = "meshdir"
    GRAVITY = "gravity"


class _MJCFElement:
    def __init__(self, tag, parent: _MJCFElement | None):
        self._element = ETree.Element(tag) if parent is None else ETree.SubElement(parent.get_element(), tag)

    def add_attribute(self, attribute_name, attribute_value):
        self._element.set(attribute_name, attribute_value)

    def get_element(self):
        return self._element


class MJCFGenerator:
    BODY_ROOT = "a"
    BODY_GOAL = "z"
    BODY_GROUND = "g"
    BODY_PLATFORM = "p"

    def __init__(self, stimulus_dirpath):
        self._mjcf_root = _MJCFElement(_MJCFTag.MJCF, None)

        self._mjcf_compiler = _MJCFElement(_MJCFTag.COMPILER, self._mjcf_root)
        # self._mjcf_compiler.add_attribute(_MJCFAttr.ANGLE, "radians")
        self._mjcf_compiler.add_attribute(_MJCFAttr.MESHDIR, "meshes/")

        self._mjcf_option = _MJCFElement(_MJCFTag.OPTION, self._mjcf_root)
        self._mjcf_option.add_attribute(_MJCFAttr.GRAVITY, "0 0 -9.80665")

        self._mjcf_asset = _MJCFElement(_MJCFTag.ASSET, self._mjcf_root)
        self._mjcf_body = _MJCFElement(_MJCFTag.WORLD_BODY, self._mjcf_root)

        self._mjcf_filepath = PathUtils.join(stimulus_dirpath, "stimulus.mjcf")

    @staticmethod
    def _mjcf_to_dict(mjcf_filepath):
        return ETree.parse(FileUtils.read_file(mjcf_filepath))

    @staticmethod
    def _get_root(mjcf_filepath):
        return MJCFGenerator._mjcf_to_dict(mjcf_filepath).getroot()

    def get_mjcf_filepath(self):
        return self._mjcf_filepath

    @staticmethod
    def get_stimulus_name(mjcf_filepath):
        return MJCFGenerator._get_root(mjcf_filepath).get(_MJCFAttr.NAME)

    @staticmethod
    def get_body_elements_dict(mjcf_filepath):
        body_elements_dict = {}
        for body in MJCFGenerator._get_root(mjcf_filepath).iter(_MJCFTag.BODY):
            if body.get(_MJCFAttr.NAME):
                body_elements_dict[body.get(_MJCFAttr.NAME)] = body
        return body_elements_dict

    @staticmethod
    def get_body_pose_list(mjcf_filepath):
        body_elements_dict = MJCFGenerator.get_body_elements_dict(mjcf_filepath)
        for body_name, body_element in body_elements_dict.items():
            body_pos = body_element.get(_MJCFAttr.POS)
            body_rot = body_element.get(_MJCFAttr.ROT)
            yield body_name, [float(pos) for pos in body_pos.split(" ")], [float(rot) for rot in body_rot.split(" ")]

    def add_ground(self, ground_pos_as_str, ground_size_str, ground_color_str=".2 .2 .2 1"):
        geom = _MJCFElement(_MJCFTag.GEOM, self._mjcf_body)
        geom.add_attribute(_MJCFAttr.TYPE, _MJCFAttr.BOX)
        geom.add_attribute(_MJCFAttr.POS, ground_pos_as_str)
        geom.add_attribute(_MJCFAttr.SIZE, ground_size_str)
        geom.add_attribute(_MJCFAttr.RGBA, ground_color_str)

    def add_body(self, body_name, body_type, body_mesh_filename, body_pos_str, body_rot_str, body_color_str, body_mass):
        mesh = _MJCFElement(_MJCFTag.MESH, self._mjcf_asset)
        mesh.add_attribute(_MJCFAttr.NAME, body_type)
        mesh.add_attribute(_MJCFAttr.FILE, body_mesh_filename)

        body = _MJCFElement(_MJCFTag.BODY, self._mjcf_body)
        body.add_attribute(_MJCFAttr.NAME, body_name)
        body.add_attribute(_MJCFAttr.POS, body_pos_str)
        body.add_attribute(_MJCFAttr.ROT, body_rot_str)

        geom = _MJCFElement(_MJCFTag.GEOM, body)
        geom.add_attribute(_MJCFAttr.TYPE, _MJCFAttr.MESH)
        geom.add_attribute(_MJCFAttr.MESH, body_type)
        geom.add_attribute(_MJCFAttr.MASS, str(body_mass))
        geom.add_attribute(_MJCFAttr.RGBA, body_color_str)

        _MJCFElement(_MJCFTag.FREE_JOINT, body)

    def add_lighting(self, light_pos_str, light_dir_str):
        light = _MJCFElement(_MJCFTag.LIGHT, self._mjcf_body)
        light.add_attribute(_MJCFAttr.POS, light_pos_str)
        light.add_attribute(_MJCFAttr.DIR, light_dir_str)

    def add_camera(self, camera_pos_str, camera_dir_str):
        camera = _MJCFElement(_MJCFTag.CAMERA, self._mjcf_body)
        camera.add_attribute(_MJCFAttr.NAME, "camera")
        camera.add_attribute(_MJCFAttr.POS, camera_pos_str)
        camera.add_attribute(_MJCFAttr.XYAXES, camera_dir_str)

    def get_mjcf_xml(self):
        return ETree.ElementTree(self._mjcf_root.get_element())

    def save_mjcf(self):
        xml = self.get_mjcf_xml()
        ETree.indent(xml, space="    ")
        xml.write(self._mjcf_filepath, xml_declaration=True, encoding="utf-8")

    def visualize(self):
        MujocoUtils(self._mjcf_filepath)

    # def save_as_stl(self, out_filename="stim"):
    #     base_dirpath = PathUtils.get_parent_dirpath(self._mjcf_filepath, 2)
    #     out_obj_filepath = PathUtils.join(base_dirpath, "meshes", out_filename + ".obj")
    #
    #     urdf_elements_list = self._mjcf_root.get_urdf_dict()["robot"]["link"]
    #     if not isinstance(urdf_elements_list, list):
    #         urdf_elements_list = [urdf_elements_list]
    #
    #     blue_material = SimpleMaterial(name='stim_blue', diffuse=[0.2, 0.2, 0.8])
    #     blue_material.name = "stim_blue"
    #
    #     brown_material = SimpleMaterial(name='stim_brown', diffuse=[0.2, 0.1, 0.07])
    #     brown_material.name = "stim_brown"
    #
    #     black_material = SimpleMaterial(name='stim_black', diffuse=[0.05, 0.05, 0.05])
    #     black_material.name = "stim_black"
    #
    #     green_material = SimpleMaterial(name='stim_green', diffuse=[0.2, 0.8, 0.2])
    #     green_material.name = "stim_green"
    #
    #     white_material = SimpleMaterial(name='stim_white', diffuse=[0.8, 0.8, 0.8])
    #     white_material.name = "stim_white"
    #
    #     scene = trimesh.Scene()
    #
    #     # add agent
    #     mesh = trimesh.load(PathUtils.join(PathUtils.get_misc_dirpath(), "agent.obj"))
    #     mesh.apply_scale(0.4)
    #     mesh.visual = TextureVisuals(material=white_material)
    #     scene.add_geometry(mesh)
    #     scene.export(out_obj_filepath)
    #
    #     # add platforms
    #     for urdf_element in urdf_elements_list:
    #         urdf_element_name = urdf_element["@name"].split("_")[0]
    #
    #         urdf_visual = urdf_element.get("visual", None)
    #         if urdf_visual is None:
    #             continue
    #
    #         urdf_geom = urdf_visual["geometry"]["mesh"]
    #
    #         mesh_filepath = PathUtils.join(base_dirpath, urdf_geom["@filename"])
    #         mesh = trimesh.load(mesh_filepath)
    #
    #         scale = [float(v) for v in urdf_geom.get("@scale", "1 1 1").split()]
    #         mesh.apply_scale(scale)
    #
    #         origin = urdf_visual.get("origin", {})
    #         xyz = [float(v) for v in origin.get("@xyz", "0 0 0").split()]
    #         roll, pitch, yaw = [float(v) for v in origin.get("@rpy", "0 0 0").split()]
    #
    #         rot_x = trimesh.transformations.rotation_matrix(roll, [1, 0, 0])
    #         rot_y = trimesh.transformations.rotation_matrix(pitch, [0, 1, 0])
    #         rot_z = trimesh.transformations.rotation_matrix(yaw, [0, 0, 1])
    #
    #         pos_mat = trimesh.transformations.translation_matrix(xyz)
    #         rot_mat = trimesh.transformations.concatenate_matrices(rot_z, rot_y, rot_x)
    #         aln_mat = trimesh.transformations.rotation_matrix(np.deg2rad(-90), [1, 0, 0])
    #         pose_transform = trimesh.transformations.concatenate_matrices(rot_mat, aln_mat, pos_mat)
    #         mesh.apply_transform(pose_transform)
    #
    #         if "a" in urdf_element_name:
    #             mesh.visual = TextureVisuals(material=blue_material)
    #         elif "g" in urdf_element_name:
    #             mesh.visual = TextureVisuals(material=brown_material)
    #         elif "z" in urdf_element_name:
    #             mesh.visual = TextureVisuals(material=green_material)
    #         else:
    #             mesh.visual = TextureVisuals(material=white_material)
    #
    #         scene.add_geometry(mesh)
    #
    #     scene.export(out_obj_filepath)

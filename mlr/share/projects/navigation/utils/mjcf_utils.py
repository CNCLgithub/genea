from __future__ import annotations

import xml.etree.ElementTree as ETree

from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.mujoco_utils import MujocoUtils


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
    SOLIMP = "solimp"
    SOLREF = "solref"


class _MJCFDefault:
    PLATFORM_COLOR = ".625 .625 .625 1"
    GROUND_COLOR = ".9 .9 .9 1"

    ROT = "0 0 0"
    SOLIMP = "0.999 0.999 0.001"
    SOLREF = "0.001 1"


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

    def __init__(self, mjcf_filepath):
        self._mjcf_root = _MJCFElement(_MJCFTag.MJCF, None)

        self._mjcf_compiler = _MJCFElement(_MJCFTag.COMPILER, self._mjcf_root)
        self._mjcf_compiler.add_attribute(_MJCFAttr.MESHDIR, "meshes/")

        self._mjcf_option = _MJCFElement(_MJCFTag.OPTION, self._mjcf_root)
        self._mjcf_option.add_attribute(_MJCFAttr.GRAVITY, "0 0 -9.80665")

        self._mjcf_asset = _MJCFElement(_MJCFTag.ASSET, self._mjcf_root)
        self._mjcf_body = _MJCFElement(_MJCFTag.WORLD_BODY, self._mjcf_root)

        self._mjcf_filepath = mjcf_filepath

        self._mesh_registry = []

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

    def add_ground(self, ground_name, ground_pos_as_str, ground_size_str):
        body = _MJCFElement(_MJCFTag.BODY, self._mjcf_body)
        body.add_attribute(_MJCFAttr.NAME, ground_name)
        body.add_attribute(_MJCFAttr.POS, ground_pos_as_str)
        body.add_attribute(_MJCFAttr.ROT, _MJCFDefault.ROT)

        geom = _MJCFElement(_MJCFTag.GEOM, body)
        geom.add_attribute(_MJCFAttr.TYPE, _MJCFAttr.BOX)
        geom.add_attribute(_MJCFAttr.SIZE, ground_size_str)
        geom.add_attribute(_MJCFAttr.RGBA, _MJCFDefault.GROUND_COLOR)
        geom.add_attribute(_MJCFAttr.SOLIMP, _MJCFDefault.SOLIMP)
        geom.add_attribute(_MJCFAttr.SOLREF, _MJCFDefault.SOLREF)

    def add_body(self, body_name, mesh_name, mesh_filename, body_pos_str, body_rot_str, body_mass):
        if mesh_name not in self._mesh_registry:
            mesh = _MJCFElement(_MJCFTag.MESH, self._mjcf_asset)
            mesh.add_attribute(_MJCFAttr.NAME, mesh_name)
            mesh.add_attribute(_MJCFAttr.FILE, mesh_filename)
            self._mesh_registry.append(mesh_name)

        body = _MJCFElement(_MJCFTag.BODY, self._mjcf_body)
        body.add_attribute(_MJCFAttr.NAME, body_name)
        body.add_attribute(_MJCFAttr.POS, body_pos_str)
        body.add_attribute(_MJCFAttr.ROT, body_rot_str)

        geom = _MJCFElement(_MJCFTag.GEOM, body)
        geom.add_attribute(_MJCFAttr.TYPE, _MJCFAttr.MESH)
        geom.add_attribute(_MJCFAttr.MESH, mesh_name)
        geom.add_attribute(_MJCFAttr.MASS, str(body_mass))
        geom.add_attribute(_MJCFAttr.RGBA, _MJCFDefault.PLATFORM_COLOR)
        geom.add_attribute(_MJCFAttr.SOLIMP, _MJCFDefault.SOLIMP)
        geom.add_attribute(_MJCFAttr.SOLREF, _MJCFDefault.SOLREF)

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

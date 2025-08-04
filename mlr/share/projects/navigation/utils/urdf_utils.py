import xmltodict

from mlr.share.projects.navigation.utils.file_utils import FileUtils
from mlr.share.projects.navigation.utils.path_utils import PathUtils


class _URDF:
    def __init__(self, tag):
        self._tag = tag
        self._attributes = {}
        self._child_urdf_dict = {}

    def add_attribute(self, key, value):
        self._attributes["@" + key] = value

    def add_child_urdf(self, input_urdf):
        input_urdf_dict = input_urdf.get_urdf_dict()
        for tag, value in input_urdf_dict.items():
            if tag in self._child_urdf_dict:
                if isinstance(self._child_urdf_dict[tag], list):
                    self._child_urdf_dict[tag].append(value)
                else:
                    self._child_urdf_dict[tag] = [self._child_urdf_dict[tag], value]
            else:
                self._child_urdf_dict[tag] = value

    def get_total_children(self):
        return len(list(self._child_urdf_dict.keys()))

    def get_urdf_dict(self):
        return{self._tag: dict(self._attributes, **self._child_urdf_dict)}


class URDFGenerator:
    def __init__(self, urdf_name, urdf_dirpath):
        self._urdf = _URDF("robot")
        self._urdf.add_attribute("name", urdf_name)

        self._urdf_filepath = URDFGenerator.get_urdf_filepath(urdf_dirpath, urdf_name)

    @staticmethod
    def get_urdf_filepath(urdf_dirpath, urdf_name):
        return PathUtils.join(urdf_dirpath, urdf_name + ".urdf")

    @staticmethod
    def _urdf_to_dict(urdf_filepath):
        return xmltodict.parse(FileUtils.read_file(urdf_filepath))

    @staticmethod
    def get_element_name(urdf_filepath):
        urdf_dict = URDFGenerator._urdf_to_dict(urdf_filepath)
        return urdf_dict["robot"]["link"]["@name"]

    @staticmethod
    def get_element_pos_rot_list(urdf_filepath):
        urdf_dict = URDFGenerator._urdf_to_dict(urdf_filepath)

        obj_pos = urdf_dict["robot"]["link"]["inertial"]["origin"]["@xyz"].split(" ")
        obj_rot = urdf_dict["robot"]["link"]["inertial"]["origin"]["@rpy"].split(" ")

        obj_pos = [float(pos) for pos in obj_pos]
        obj_rot = [float(rot) for rot in obj_rot]

        return obj_pos, obj_rot

    def add_element(self, element_name, element_pos_str, element_rot_str, element_mass, element_mesh_filename):
        link = _URDF("link")
        link.add_attribute("name", element_name)

        mesh = _URDF("mesh")
        mesh.add_attribute("filename", "meshes/" + element_mesh_filename)
        mesh.add_attribute("scale", "1 1 1")

        geometry = _URDF("geometry")
        geometry.add_child_urdf(mesh)

        origin = _URDF("origin")
        origin.add_attribute("xyz", element_pos_str)
        origin.add_attribute("rpy", element_rot_str)

        mass = _URDF("mass")
        mass.add_attribute("value", element_mass)

        inertia = _URDF("inertia")
        inertia.add_attribute("ixx", "0.4167")
        inertia.add_attribute("ixy", "0.0")
        inertia.add_attribute("ixz", "0.0")
        inertia.add_attribute("iyy", "0.4167")
        inertia.add_attribute("iyz", "0.0")
        inertia.add_attribute("izz", "0.1667")

        visual = _URDF("visual")
        visual.add_child_urdf(origin)
        visual.add_child_urdf(geometry)

        collision = _URDF("collision")
        collision.add_child_urdf(origin)
        collision.add_child_urdf(geometry)

        inertial = _URDF("inertial")
        inertial.add_child_urdf(origin)
        inertial.add_child_urdf(mass)
        inertial.add_child_urdf(inertia)

        link.add_child_urdf(visual)
        link.add_child_urdf(collision)
        link.add_child_urdf(inertial)

        self._urdf.add_child_urdf(link)

    def add_joint(self, parent_element, child_element, joint_pos_str, joint_type):
        joint = _URDF("joint")
        joint.add_attribute("name", f"{child_element}_joint")
        joint.add_attribute("type", joint_type)

        parent = _URDF("parent")
        parent.add_attribute("link", parent_element)

        child = _URDF("child")
        child.add_attribute("link", child_element)

        origin = _URDF("origin")
        origin.add_attribute("xyz", joint_pos_str)
        origin.add_attribute("rpy", "0.0 0.0 0.0")

        if joint_type == "fixed":
            child = _URDF("axis")
            joint.add_attribute("xyz", "0.0 0.0 0.0")

        joint.add_child_urdf(parent)
        joint.add_child_urdf(child)
        joint.add_child_urdf(origin)

        self._urdf.add_child_urdf(joint)

    def get_urdf_xml(self):
        return xmltodict.unparse(self._urdf.get_urdf_dict(), pretty=True)

    def save_urdf(self):
        FileUtils.write_to_file(self._urdf_filepath, self.get_urdf_xml())

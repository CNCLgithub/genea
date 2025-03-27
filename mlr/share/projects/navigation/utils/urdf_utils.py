import xmltodict

from mlr.share.projects.navigation.utils.file_utils import FileUtils


class _URDF:
    def __init__(self, tag):
        self._tag = tag
        self._attributes = {}
        self._child_urdf_dict = {}

        self._urdf_dict = {}

    def add_attribute(self, key, value):
        self._attributes["@" + key] = value

    def add_child_urdf(self, input_urdf):
        input_urdf_dict = input_urdf.get_urdf_dict()
        if not self._child_urdf_dict:
            self._child_urdf_dict = input_urdf_dict
            return
        self._child_urdf_dict = dict(self._child_urdf_dict, **input_urdf_dict)

    def get_urdf_dict(self):
        return{self._tag: dict(self._attributes, **self._child_urdf_dict)}


class _URDFRobot(_URDF):
    TAG = "robot"
    ATTR_NAME = "name"

    def __init__(self, robot_name):
        super().__init__(_URDFRobot.TAG)
        self.add_attribute(_URDFRobot.ATTR_NAME, robot_name)


class URDFGenerator:
    def __init__(self, urdf_filepath):
        self._urdf_filepath = urdf_filepath
        self._urdf = None

    def get_urdf_xml(self):
        xmltodict.unparse(self._urdf.get_urdf_dict(), pretty=True)

    def save_urdf(self):
        FileUtils.write_to_file(self._urdf_filepath, self.get_urdf_xml())


class URDFViewer:
    # generate scenes here
    pass

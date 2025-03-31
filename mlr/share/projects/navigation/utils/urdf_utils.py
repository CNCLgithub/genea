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


class _URDFObject:
    def __init__(self, object_name, object_pose):
        self._link = _URDF("link")
        self._visual = _URDF("visual")
        self._origin = _URDF("origin")

        self._link.add_attribute("name", object_name)
        self._link.add_child_urdf(self._visual)

        self._origin.add_attribute("xyz", "0 0 0")
        self._origin.add_attribute("rpy", "0 0 0")


class URDFViewer:
    # generate scenes here
    pass


def main():
    pass


if __name__ == '__main__':
    main()
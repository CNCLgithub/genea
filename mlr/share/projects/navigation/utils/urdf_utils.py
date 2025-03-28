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

class _URDFGeometry(_URDF):
    TAG = "geometry"

    def __init__(self, shape, size):
        super().__init__(_URDFGeometry.TAG)
        shape_tag = _URDF(shape)
        shape_tag.add_attribute("size", size)
        self.add_child_urdf(shape_tag)
        
class _URDFOrigin(_URDF):
    TAG = "origin"
    
    def __init__(self, xyz, rpy):
        super().__init__(_URDFOrigin.TAG)
        self.add_attribute("xyz", xyz)
        self.add_attribute("rpy", rpy)
        
class _URDFMaterial(_URDF):
    TAG = "material"
    
    def __init__(self, color):
        super().__init__(_URDFMaterial.TAG)
        self.add_attribute("color rgba", color)
        
class _URDFMass(_URDF):
    TAG = "mass"
    
    def __init__(self, mass):
        super().__init__(_URDFMass.TAG)
        self.add_attribute("value", mass)
        
class _URDFInertia(_URDF):
    TAG = "inertia"
    
    def __init__(self, ixx, ixy, ixz, iyy, iyz, izz):
        super().__init__(_URDFInertia.TAG)
        self.add_attribute("ixx", ixx)
        self.add_attribute("ixy", ixy)
        self.add_attribute("ixz", ixz)
        self.add_attribute("iyy", iyy)
        self.add_attribute("iyz", iyz)
        self.add_attribute("izz", izz)
    
    
    
class _URDFVisual(_URDF):
    TAG = "visual"
    
    def __init__(self):
        super().__init__(_URDFVisual.TAG)
        
    def set_geometry(self, shape, size):
        geometry = _URDFGeometry(shape, size)
        self.add_child_urdf(geometry)
        
    def set_origin(self, xyz, rpy):
        origin = _URDFOrigin(xyz, rpy)
        self.add_child_urdf(origin)     
        
    def set_material(self, color):
        color = _URDFMaterial(color)
        self.add_child_urdf(color)   

class _URDFCollision(_URDF):
    TAG = "collision"
    
    def __init__(self):
        super().__init__(_URDFCollision.TAG)
        
    def set_geometry(self, shape, size):
        geometry = _URDFGeometry(shape, size)
        self.add_child_urdf(geometry)
        
    def set_origin(self, xyz, rpy):
        origin = _URDFOrigin(xyz, rpy)
        self.add_child_urdf(origin) 
        
class _URDFInertial(_URDF):
    TAG = "inertial"
    
    def __init__(self):
        super().__init__(_URDFInertial.TAG)
    
    def set_mass(self, mass):
        mass_tag = _URDFMass(mass)
        self.add_child_urdf(mass_tag)
        
    def set_origin(self, xyz, rpy):
        origin = _URDFOrigin(xyz, rpy)
        self.add_child_urdf(origin)
        
    def set_inertia(self, ixx, ixy, ixz, iyy, iyz, izz):
        inertia_tag = _URDF(ixx, ixy, ixz, iyy, iyz, izz)
        self.add_child_urdf(inertia_tag)



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

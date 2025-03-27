class URDFCubeGenerator:
    def __init__(self, name = "cube1", dimensions = (1.0,1.0,1.0), mass = 0.5, color = (1.0,0,0,1.0), position = (0,0,0)):
        self.name = name
        self.dimensions = dimensions
        self.mass = mass
        self.color = color
        self.position = position

    def generate_urdf_cube(self):

        x, y, z = self.dimensions
        r, g, b, a = self.color
        px, py, pz = self.position

        urdf = f"""
        <robot name="{self.name}">
            <link name="{self.name}_link">
                <visual>
                    <origin xyz="{px} {py} {pz}" rpy="0 0 0"/>
                    <geometry>
                        <box size="{x} {y} {z}"/>
                    </geometry>
                    <material name="cube_color">
                        <color rgba="{r} {g} {b} {a}"/>
                    </material>
                </visual>

                <inertial>
                    <mass value="{self.mass}"/>
                    <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0.0" ixz="0.0" iyz="0.0"/>
                </inertial>

                <collision>
                    <origin xyz="{px} {py} {pz}" rpy="0 0 0"/>
                    <geometry>
                        <box size="{x} {y} {z}"/>
                    </geometry>
                </collision>
            </link>
        </robot>
        """.strip()

        return urdf


class URDFCylinderGenerator:
    def __init__(self, name="cylinder1", radius=0.5, height=1.0, mass=1.0, color=(1.0, 0, 0, 1.0), position=(0, 0, 0)):
        self.name = name
        self.radius = radius
        self.height = height
        self.mass = mass
        self.color = color
        self.position = position
        
    def generate_urdf_cylinder(self):
        r, g, b, a = self.color
        px, py, pz = self.position
        
        urdf = f"""
        <robot name="{self.name}">
            <link name="{self.name}_link">
                <visual>
                    <origin xyz="{px} {py} {pz}" rpy="0 0 0"/>
                    <geometry>
                        <!-- Cylinder geometry -->
                        <cylinder radius="{self.radius}" length="{self.height}"/>
                    </geometry>
                    <material name="cylinder_color">
                        <color rgba="{r} {g} {b} {a}"/>
                    </material>
                </visual>

                <inertial>
                    <mass value="{self.mass}"/>
                    <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0.0" ixz="0.0" iyz="0.0"/>
                </inertial>

                <collision>
                    <origin xyz="{px} {py} {pz}" rpy="0 0 0"/>
                    <geometry>
                        <cylinder radius="{self.radius}" length="{self.height}"/>
                    </geometry>
                </collision>
            </link>
        </robot>
        """.strip()

        return urdf
    
class URDFViewer:
    # generate scenes here
    pass
import urdfpy
import numpy as np

# Define a Link with an STL mesh
link = urdfpy.Link(
    name="base_link",
    visuals=[urdfpy.Visual(
        geometry=urdfpy.Geometry(mesh=urdfpy.Mesh(filename="meshes/base.stl")),
        material=urdfpy.Material(name="gray", color=np.array([0.5, 0.5, 0.5, 1.0]))
    )]
)

# Create Robot and Export
robot = urdfpy.URDF(name="my_robot", links=[link])
robot.export("f450.urdf")

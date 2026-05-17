# f450_uav


In the URDF change:

bottom_plate -> body_link
assets -> meshes


update

<joint name="body_to_rotor_1" type="fixed">

to

<joint name="body_to_rotor_1" type="continuous">

add eixo 

<axis xyz="0 0 1"/>
import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF('boxes.sdf')

l = 1
w = 1
h = 1

x = 0
y = 0
z = h/2

x_layers = 5
y_layers = 5
z_layers = 15

for x_idx in range(x_layers):
    for y_idx in range(y_layers):
        base_height = 0
        for z_idx in range(z_layers):
            scale = 0.9 ** z_idx
            z_loc = base_height + h*scale/2
            
            pyrosim.Send_Cube(name='Box', pos=[x_idx*l, y_idx*w, z_loc], size=[l*scale, w*scale, h*scale])
            
            base_height += h*scale

pyrosim.End()

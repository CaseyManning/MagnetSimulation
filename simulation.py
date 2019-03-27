import bpy

default_strength = 10

# Class to represent a magnet. Obj variable refers to the Blender representation of the object
class Magnet:
    obj = None
    strength = 2
    
    def __init__(self, obj, strength):
        self.obj = obj
        self.strength = strength

magnets = []

for obj in bpy.context.scene.objects:
    if 'Sphere' in obj.name:
        magnets.append(Magnet(obj, default_strength))


print(magnets)

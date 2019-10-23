import bpy
from operator import Test_Operator
from MagnetsPanel import Magnets_Panel

bl_info = {
    "name" : "Magnet Generator",
    "author": "Casey",
    "description" : "Magnet Generator",
    "blender" : (2, 80, 0),
    "location" : "",
    "warning" : "",
    "category" : "Generic"
}
classes = (Test_Operator, Magnets_Panel)
register, unregister = bpy.utils.register_classes_factory(classes)

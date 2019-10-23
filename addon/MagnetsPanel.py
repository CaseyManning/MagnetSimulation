import bpy
from operator import Test_Operator

class Magnets_Panel(bpy.types.Panel):
    bl_idname = "Magnets_Panel"
    bl_label = "Magnet Panel"
    bl_category = "Test Addon"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"


    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator('view3d.createMagnets', text='Create Magnets')
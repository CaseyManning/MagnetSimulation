import bpy
from . operator import *
from bpy.props import *
from . properties import BGProperties

bl_info = {
    "name" : "MagnetGenerator",
    "author" : "MagnetGenerator.py",
    "description" : "",
    "blender" : (2, 80, 0),
    "version" : (0, 0, 1),
    "location" : "",
    "warning" : "",
    "category" : "Generic"
}

class Magnets_PT_Panel(bpy.types.Panel):
    bl_idname = "Magnets_PT_Panel"
    bl_label = "Magnet Panel"
    bl_category = "Magnet Panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        bytool = context.scene.by_tool

        col = layout.column()
        col.prop(bytool, "scale_factor")
        col.prop(bytool, "show_grid")
        col.prop(bytool, "auto_clear")
        col.prop(bytool, "auto_adjust")

        layout.separator()

        box = layout.box()
        box.label(text="Generation Settings")
        bcol = box.column()

        bcol.prop(bytool, "Shape")

        layout.separator()

        if bytool.Shape in ["Line", "Loop", "Grid", "Cylinder"]:
            bcol.prop(bytool, "num_magnets")

        if bytool.Shape == "Grid" or bytool.Shape == "Cylinder":
            layout.separator()
            bcol.prop(bytool, "Order")

        if bytool.Shape == "Line":
            bcol.prop(bytool, "line_direction")
            bcol.prop(bytool, "moment_direction")

        # col.label(text="Generate objects:")
        # col.prop(context.scene.MagnetGenerator, "primitive")
        layout.separator()
        row = layout.row()
        props = row.operator('view3d.create_magnets', text='Create Magnets')

        props2 = layout.operator('view3d.clear_magnets', text='Clear Magnets')

        layout.separator()

        box = layout.box()
        box.label(text="Magnet Partials")
        bcol = box.column()

        bcol.prop(bytool, "Auto_recalculate_Partials")

        layout.separator()

        props3 = bcol.operator('view3d.calc_partials', text='Calculate Partials')

        props4 = bcol.operator('view3d.clear_partials', text='Clear Partials')

classes = (BGProperties, Magnets_PT_Panel, Create_OT_Operator, Clear_OT_Operator, Clear_Partials_OT_Operator, Calculate_Partials_OT_Operator)

# register, unregister = bpy.utils.register_classes_factory(classes)

def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)
    bpy.types.Scene.by_tool = PointerProperty(type=BGProperties)

def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)
    del bpy.types.Scene.by_tool
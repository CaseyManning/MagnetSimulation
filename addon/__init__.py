import bpy
from bpy.props import *
from . StabilitySimulator2 import MagnetSimulator
from . Magnet import Magnet

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

class Test_OT_Operator(bpy.types.Operator):
    bl_idname = "view3d.cursor_center"
    bl_label = "simple operator"
    bl_description = "Create Magnets"

    # def invoke(self, context, event):
    #     return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):

        bytool = context.scene.by_tool

        if bytool.shape=="Loop":
            magnets = MagnetSimulator.loop(bytool.num_magnets, True)
            for i in range(len(magnets)):
                bpy.ops.mesh.primitive_uv_sphere_add(radius=context.scene.by_tool.num_magnets, enter_editmode=False, location=(magnets[i].position[0], magnets[i].position[1], magnets[i].position[2]))
            bpy.ops.view3d.snap_cursor_to_center()

        
        bpy.ops.mesh.primitive_uv_sphere_add(radius=Magnet.radius, enter_editmode=False, location=(1.49944, 0.813217, 3.65082))
        return {'FINISHED'}

def execute_operator(self, context):
    eval('bpy.ops.' + self.primitive + '()')

class BGProperties(bpy.types.PropertyGroup):
    mode_options = [
        ("Loop", "Loop", '', 'MESH_CIRCLE', 0),
        ("Saddle", "Saddle", '', '', 1),
        ("Line", "Line", '', '', 2),
        ("Antitesselated Hypersphere", "Antitesselated Hypersphere", '', '', 3)
    ]

    shape = bpy.props.EnumProperty(
        items=mode_options,
        description="Shape of Magnet Construction",
        default="Loop",
        update=execute_operator
    )

    gen_decimate_collapse: FloatProperty(
        name = "Decimate Collapse",
        description = "Collapse ratio for the Decimation modifier",
        default = 0.2,
        min = 0.0,
        max = 1.0
        )
    #.......... Float for Generation - Decimation - Angle Limit
    num_magnets: IntProperty(
        name = "Number of Magnets",
        default = 5,
        min=1,
        max=15,
        description = "Number of Magnets to create"
    )

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

        layout.prop(bytool, "shape")

        layout.prop(bytool, "num_magnets")
        
        # col.label(text="Generate objects:")
        # col.prop(context.scene.MagnetGenerator, "primitive")
        layout.separator()
        row = layout.row()
        props = row.operator('view3d.cursor_center', text='Create Magnets')

classes = (BGProperties, Magnets_PT_Panel, Test_OT_Operator)

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
import bpy
from bpy.props import *
from . StabilitySimulator2 import MagnetSimulator
from . Magnet import Magnet
# from . operator import 

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

class Create_OT_Operator(bpy.types.Operator):
    bl_idname = "view3d.create_magnets"
    bl_label = "simple operator"
    bl_description = "Create Magnets"

    # def invoke(self, context, event):
    #     return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        scene = context.scene
        bytool = scene.by_tool
        if bytool.auto_clear:
            bpy.ops.view3d.clear_magnets('INVOKE_DEFAULT')
        if bytool.Shape=="Loop":
            magnets = MagnetSimulator.loop(bytool.num_magnets, True)
        elif bytool.Shape=="Line":
            magnets = MagnetSimulator.line(bytool.num_magnets, 'x', 'y')
        else:
            magnets = []
        for i in range(len(magnets)):
            src_obj = bpy.data.objects["BaseMagnet"]
            new_obj = src_obj.copy()
            new_obj.data = src_obj.data.copy()
            new_obj.animation_data_clear()
            context.collection.objects.link(new_obj)
            new_obj.location.x = magnets[i].position[0]*bytool.scale_factor
            new_obj.location.y = magnets[i].position[1]*bytool.scale_factor
            new_obj.location.z = magnets[i].position[2]*bytool.scale_factor
            new_obj.scale.x *= bytool.scale_factor
            new_obj.scale.y *= bytool.scale_factor
            new_obj.scale.z *= bytool.scale_factor
        else:
            pass
            
        return {'FINISHED'}

class Clear_OT_Operator(bpy.types.Operator):
    bl_idname = "view3d.clear_magnets"
    bl_label = "Clears Magnets"
    bl_description = "Clear Magnets"

    # def invoke(self, context, event):
    #     return context.window_manager.invoke_props_dialog(self)

    def execute(self, context):
        scene = context.scene
        bytool = scene.by_tool
        bpy.ops.object.select_all(action='DESELECT')
        for obj in bpy.data.objects:
            if obj.name.startswith("BaseMagnet") and not obj.name == 'BaseMagnet':
                obj.select_set(True)
                bpy.ops.object.delete() 

        return {'FINISHED'}



class Clear_Partials_OT_Operator(bpy.types.Operator):
    bl_idname = "view3d.clear_partials"
    bl_label = "Clears Partials"
    bl_description = "Clears Partials"

    def execute(self, context):
        scene = context.scene
        bytool = scene.by_tool
        return {'FINISHED'}

class Calculate_Partials_OT_Operator(bpy.types.Operator):
    bl_idname = "view3d.calc_partials"
    bl_label = "Calculates Partials"
    bl_description = "Calculates Partials"

    def execute(self, context):
        scene = context.scene
        bytool = scene.by_tool
        return {'FINISHED'}



def execute_operator(self, context):
    if context.scene.by_tool.auto_adjust:
        bpy.ops.view3d.create_magnets('INVOKE_DEFAULT')

def toggle_plane(self, context):
    if context.scene.by_tool.show_grid:
        bpy.ops.object.select_all(action='DESELECT')
        bpy.data.objects['Plane'].select = True
        bpy.data.objects['BaseMagnet'].select = True
        bpy.ops.object.hide_view_set(unselected=False)
    else:
        bpy.ops.object.hide_view_clear()


class BGProperties(bpy.types.PropertyGroup):
    mode_options = [
        ("Loop", "Loop", '', 'MESH_CIRCLE', 0),
        ("Saddle", "Saddle", '', 'PANEL_CLOSE', 1),
        ("Line", "Line", '', 'REMOVE', 2),
        ("Grid", "Grid", '', 'MESH_GRID', 3),
        ("Cylinder", "Cylinder", '', 'MESH_CYLINDER', 4),
        ("Cantilated Dodecahedron", "Cantilated Dodecahedron", '', 'MESH_ICOSPHERE', 5)
    ]

    scale_factor: IntProperty(
        name = "Scale Factor",
        default = 50,
        min=1,
        max=100,
        description = "Amount to scale the magnet construction in space."
    )

    Shape = bpy.props.EnumProperty(
        items=mode_options,
        description="Shape of Magnet Construction",
        default="Loop",
        update=execute_operator
    )

    show_grid = bpy.props.BoolProperty(
        name = "Show Grid",
        description = "Whether or not to show the grid background",
        default = True,
        update=toggle_plane
    )

    auto_clear = bpy.props.BoolProperty(
        name = "Auto-clear",
        description = "Whether or not to clear the scene before making a new construction",
        default = True
    )

    auto_adjust = bpy.props.BoolProperty(
        name = "Auto-adjust",
        description = "Whether or not to automatically create new magnets on setting change",
        default = False
    )

    Auto_recalculate_Partials = bpy.props.BoolProperty(
        name = "Auto-recalculate Partials",
        description = "Whether or not to automatically update partials on magnets change",
        default = False
    )

    Order = bpy.props.EnumProperty(
        items=[('Parallel', 'Parallel', '', 0), ('Antiparallel', 'Antiparallel', '', 1)],
        description="",
        default="Parallel",
        update=execute_operator
    )

    gen_decimate_collapse: FloatProperty(
        name = "Decimate Collapse",
        description = "Collapse ratio for the Decimation modifier",
        default = 0.2,
        min = 0.0,
        max = 1.0
    )
    num_magnets: IntProperty(
        name = "Number of Magnets",
        default = 5,
        min=1,
        max=15,
        description = "Number of Magnets to create",
        update=execute_operator
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

        bcol.prop(bytool, "num_magnets")

        if bytool.Shape == "Grid" or bytool.Shape == "Cylinder":
            layout.separator()
            bcol.prop(bytool, "Order")
        
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
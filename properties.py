import bpy
from bpy.props import *

def execute_operator(self, context):
    if context.scene.by_tool.auto_adjust:
        bpy.ops.view3d.create_magnets('INVOKE_DEFAULT')
        if context.scene.by_tool.Auto_recalculate_Partials:
            bpy.ops.view3d.calc_partials('INVOKE_DEFAULT')

def toggle_plane(self, context):
    if context.scene.by_tool.show_grid:
        print("SHOWING PLANE")
        bpy.data.objects['Plane'].hide_viewport = False
    else:
        bpy.data.objects['Plane'].hide_viewport = True
        print("HIDING PLANE")


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
        description = "Amount to scale the magnet construction in space.",
        update=execute_operator
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

    line_direction = bpy.props.EnumProperty(
        items=[('x', 'x', '', 0), ('y', 'y', '', 1), ('z', 'z', '', 2)],
        description="",
        default="x",
        update=execute_operator
    )

    moment_direction = bpy.props.EnumProperty(
        items=[('x', 'x', '', 0), ('y', 'y', '', 1), ('z', 'z', '', 2)],
        description="",
        default="x",
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
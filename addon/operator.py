import bpy

class Test_Operator(bpy.types.Operator):
    bl_idname = "view3d.createMagnets"
    bl_label = "simple operator"
    bl_description = "Create Magnets"

    def excecute(self, context):
        bpy.ops.view3d.snap_cursor_to_center()
        return {'FINISHED'}
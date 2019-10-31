import bpy
from . StabilitySimulator2 import MagnetSimulator
from . Magnet import Magnet
import math

class Create_OT_Operator(bpy.types.Operator):
    bl_idname = "view3d.create_magnets"
    bl_label = "simple operator"
    bl_description = "Create Magnets"

    def vec2angle(self, vec):
        x = vec[0]
        y = vec[1]
        z = vec[2]
        ax = math.atan(z/y)
        ay = math.atan(x/z)
        az = math.atan(x/y)
        return ax, ay, az

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
            rotation = self.vec2angle(magnets[i].moment)
            print("ROTATION: " + str(rotation))
            new_obj.rotation_euler.x = 180/3.14 * rotation[0]
            new_obj.rotation_euler.y = 180/3.14 * rotation[1]
            new_obj.rotation_euler.z = 90 + 180/3.14 * rotation[2]
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
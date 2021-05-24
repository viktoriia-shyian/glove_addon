from bpy.types import (Panel,
                       Menu,
                       Operator,
                       PropertyGroup,
                       )

from bpy.props import (StringProperty,
                       BoolProperty,
                       IntProperty,
                       FloatProperty,
                       FloatVectorProperty,
                       EnumProperty,
                       PointerProperty,
                       )

import bpy
import os

from .scene_properties import (GProperties)
from .device import (Device)
from .model import (Hand)
#from addon import device

device = Device()
model_editing = False



class G_OT_OpenModel(Operator):
    bl_label = "Open Model Operator"
    bl_idname = "g.open_model"

    def execute(self, context):
        scene = context.scene
        tool = scene.tool

        try:
            bpy.ops.wm.open_mainfile(filepath=tool.model_path)

            CUSTOM_add.addToList(self, "Opened the model" + tool.model_path)

        except Exception as error:
            CUSTOM_add.addToList(self, str(error))

        return {'FINISHED'}
import time

class G_OT_ConnectDevice(Operator):
    bl_label = "Connect Device Operator"
    bl_idname = "g.connect_device"

    def execute(self, context):
        scene = context.scene
        tool = scene.tool

        try:
            CUSTOM_add.addToList(self, "Connected")
            CUSTOM_add.addToList(self, "Wait 10 seconds")
#            device.connect()
#            CUSTOM_add.addToList(self, device.check_connection())

        except Exception as error:
            CUSTOM_add.addToList(self, str(error))

        return {'FINISHED'}



class G_OT_Calibrate(Operator):
    bl_label = "Calibrate Device Operator"
    bl_idname = "g.calibrate_device"

    def execute(self, context):
        scene = context.scene
        tool = scene.tool

        try:
            timing = time.time()
            state = True
            while state:
                if time.time() - timing > 10.0:
                    CUSTOM_add.addToList(self, ">.>.>.>.>.>.>.>.>.>.>.>.Calibration OK")
                    state = False
#            CUSTOM_add.addToList(self, device.calibrate())

        except Exception as error:
            CUSTOM_add.addToList(self, str(error))

        return {'FINISHED'}


class G_OT_StartUsage(Operator):
    bl_label = "Start Usage Operator"
    bl_idname = "g.start_usage"

    def execute(self, context):
        scene = context.scene
        tool = scene.tool

        try:
            model_editing = True

#            while(model_editing):
#                device.get_dmp()
            CUSTOM_add.addToList(self, "Start reading data for editing model")

        except Exception as error:
            CUSTOM_add.addToList(self, str(error))

        return {'FINISHED'}


class G_OT_EndUsage(Operator):
    bl_label = "End Usage Operator"
    bl_idname = "g.end_usage"

    def execute(self, context):
        scene = context.scene
        tool = scene.tool

        try:
            model_editing = False
            CUSTOM_add.addToList(self, "Stop reading data")

        except Exception as error:
            CUSTOM_add.addToList(self, str(error))

        return {'FINISHED'}


class G_OT_DisconnectDevice(Operator):
    bl_label = "Disconnect Device Operator"
    bl_idname = "g.disconnect_device"

    def execute(self, context):
        scene = context.scene
        tool = scene.tool

        try:
            CUSTOM_add.addToList(self, "Disconnected")
            #CUSTOM_add.addToList(self, device.disconnect())
        except Exception as error:
            CUSTOM_add.addToList(self, str(error))

        return {'FINISHED'}


class CUSTOM_OT_actions(Operator):
    """Move items up and down, add and remove"""
    bl_idname = "custom.list_action"
    bl_label = "List Actions"
    bl_description = "Move items up and down, add and remove"
    bl_options = {'REGISTER'}

    action: EnumProperty(
        items=(
            ('UP', "Up", ""),
            ('DOWN', "Down", ""),
            ('REMOVE', "Remove", ""),
            ('ADD', "Add", "")))

    list_item: StringProperty()

    def invoke(self, context, event):
        scn = context.scene
        idx = scn.custom_index

        try:
            item = scn.custom[idx]
        except IndexError:
            pass
        else:
            if self.action == 'DOWN' and idx < len(scn.custom) - 1:
                item_next = scn.custom[idx+1].name
                scn.custom.move(idx, idx+1)
                scn.custom_index += 1
                info = 'Item "%s" moved to position %d' % (
                    item.name, scn.custom_index + 1)
                self.report({'INFO'}, info)

            elif self.action == 'UP' and idx >= 1:
                item_prev = scn.custom[idx-1].name
                scn.custom.move(idx, idx-1)
                scn.custom_index -= 1
                info = 'Item "%s" moved to position %d' % (
                    item.name, scn.custom_index + 1)
                self.report({'INFO'}, info)

            elif self.action == 'REMOVE':
                info = 'Item "%s" removed from list' % (scn.custom[idx].name)
                scn.custom_index -= 1
                scn.custom.remove(idx)
                self.report({'INFO'}, info)

        if self.action == 'ADD':
            if self.list_item:
                item = scn.custom.add()
                item.name = self.list_item
                item.coll_id = len(scn.custom)
                scn.custom_index = len(scn.custom)-1
                info = '"%s" added to list' % (item.name)
                self.report({'INFO'}, info)

        return {"FINISHED"}


class CUSTOM_OT_clearList(Operator):
    """Clear all items of the list"""
    bl_idname = "custom.clear_list"
    bl_label = "Clear List"
    bl_description = "Clear all items of the list"
    bl_options = {'INTERNAL'}

    @classmethod
    def poll(cls, context):
        return bool(context.scene.custom)

    def invoke(self, context, event):
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context):
        if bool(context.scene.custom):
            context.scene.custom.clear()
            self.report({'INFO'}, "All items removed")
        else:
            self.report({'INFO'}, "Nothing to remove")

#        os.system('cls')

        return{'FINISHED'}


class CUSTOM_add():

    @staticmethod
    def addToList(self, message):
        scn = bpy.context.scene
        idx = scn.custom_index

        item = scn.custom.add()
        item.name = message
        item.coll_id = len(scn.custom)
        scn.custom_index = len(scn.custom)-1
        info = '"%s" added to list' % (item.name)
        self.report({'INFO'}, info)
        print({'INFO'}, info)

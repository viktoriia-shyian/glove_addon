from bpy.types import (Panel,
                       Menu,
                       Operator,
                       PropertyGroup,
                       UIList
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


class OBJECT_PT_CustomPanel(Panel):
    bl_label = "Glove Control Panel"
    bl_idname = "OBJECT_PT_custom_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Glove Tool"
    #bl_context = "objectmode"

#    @classmethod
#    def poll(self, context):
#        return context.object is not None

    @classmethod
    def poll(cls, context):
        return context.mode in {'OBJECT', 'POSE'}

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        tool = scene.tool

        layout.prop(tool, "model_path")
        layout.operator("g.open_model")

        row = layout.row()
        row.template_list("custom.ui_list", "", scene,
                          "custom", scene, "custom_index", rows=7)

        row = layout.row().operator("custom.clear_list", icon="X")

        col = layout.column(align=True)

        row2 = col.row(align=True)
        row2.operator("g.connect_device", text="Connect", icon="ACTION")
        row2.operator("g.disconnect_device", text="Disconnect", icon="QUIT")

        col.operator("g.calibrate_device", text="Calibrate", icon="MODIFIER")

        row3 = col.row(align=True)
        row3.operator("g.start_usage", text="Start", icon="PLAY")
        row3.operator("g.end_usage", text="Stop", icon="PAUSE")

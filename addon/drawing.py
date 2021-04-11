import bpy

from bpy.props import (IntProperty,
                       BoolProperty,
                       StringProperty,
                       EnumProperty,
                       CollectionProperty)

from bpy.types import (Operator,
                       Panel,
                       PropertyGroup,
                       UIList)


class CUSTOM_UL_items(UIList):
    bl_idname = "custom.ui_list"

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        split = layout.split(factor=0.2)
        split.label(text="%d" % (index))
        #split.prop(item, "name", text="", emboss=False, translate=False, icon=custom_icon)
        split.label(text=item.name)  # avoids renaming the item by accident

    def invoke(self, context, event):
        pass


class CUSTOM_OT_popup(Operator):
    bl_idname = "custom.call_popup"
    bl_label = "Log Window"
    bl_options = {'REGISTER'}

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        return {'FINISHED'}

    def check(self, context):
        return True

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=500)

    def draw(self, context):
        layout = self.layout
        scn = bpy.context.scene

        rows = 7
        row = layout.row()
        row.template_list("CUSTOM_UL_items", "", scn,
                          "custom", scn, "custom_index", rows=rows)

        row = layout.row()
        add = row.operator(CUSTOM_OT_actions.bl_idname,
                           icon="ADD", text="Add Log Line")
        add.list_item = "Error %d" % (len(scn.custom))
        add.action = 'ADD'

        row = layout.row().operator("custom.clear_list", icon="X")
        row = layout.row()

from .panel import (OBJECT_PT_CustomPanel)

from .operators import (G_OT_OpenModel,
                        G_OT_ConnectDevice,
                        G_OT_Calibrate,
                        G_OT_StartUsage,
                        G_OT_EndUsage,
                        G_OT_DisconnectDevice,
                        CUSTOM_OT_actions,
                        CUSTOM_OT_clearList)

from .scene_properties import (GProperties)

from .drawing import (CUSTOM_UL_items,
                      CUSTOM_OT_popup)

import bpy

bl_info = {
    "name": "Glove Add-on",
    "description": "",
    "author": "VYanhy",
    "version": (0, 0, 1),
    "blender": (2, 92, 0),
    "location": "3D View > Tools",
    "warning": "",  # used for warning icon and text in addons panel
    "wiki_url": "",
    "tracker_url": "",
    "category": "Development"
}


classes = (
    GProperties,
    G_OT_OpenModel,
    G_OT_ConnectDevice,
    G_OT_Calibrate,
    G_OT_StartUsage,
    G_OT_EndUsage,
    G_OT_DisconnectDevice,
    OBJECT_PT_CustomPanel,
    CUSTOM_OT_actions,
    CUSTOM_OT_clearList,
    CUSTOM_UL_items,
    CUSTOM_OT_popup
)


def register():
    from bpy.utils import register_class
    for cls in classes:
        register_class(cls)

    bpy.types.Scene.tool = bpy.props.PointerProperty(type=GProperties)
    bpy.types.Scene.custom = bpy.props.CollectionProperty(type=GProperties)
    bpy.types.Scene.custom_index = bpy.props.IntProperty()


def unregister():
    from bpy.utils import unregister_class
    for cls in reversed(classes):
        unregister_class(cls)

    del bpy.types.Scene.tool
    del bpy.types.Scene.custom
    del bpy.types.Scene.custom_index


if __name__ == "__main__":
    register()

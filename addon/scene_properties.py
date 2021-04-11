from bpy.types import (Operator,
                       Panel,
                       Menu,
                       PropertyGroup,
                       UIList)

from bpy.props import (StringProperty,
                       IntProperty,
                       BoolProperty,
                       StringProperty,
                       EnumProperty,
                       CollectionProperty,
                       PointerProperty,
                       FloatProperty,
                       FloatVectorProperty
                       )

import bpy


class GProperties(PropertyGroup):

    # name: StringProperty() -> Instantiated by default
    coll_type: StringProperty()
    coll_id: IntProperty()

    model_path: StringProperty(
        name="Model directory",
        description="Choose a directory:",
        default="",
        maxlen=1024,
        subtype='FILE_PATH'
    )

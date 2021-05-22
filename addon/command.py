import enum


class Command(enum.Enum):
    connection_state = ("0", 6)
    sensitivity = ("1", 1)
    calibrate = ("2", 1)
    offsets = ("3", 6)
    raw_data = ("4", 6)
    get_dmp = ("5", 6)
    start_timer = ("6", 1)
    stop_timer = ("7", 1)

    def __init__(self, id, expected_lines):
        self.id = id
        self.expected_lines = expected_lines

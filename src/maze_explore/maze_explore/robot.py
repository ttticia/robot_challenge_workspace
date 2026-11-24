"""Lite3 mode and gait requests on /simple_cmd (robot_info.md section 6.3)."""
AUTO = 0x21010C03        # navigation mode: the robot follows /cmd_vel
MANUAL = 0x21010C02      # handheld controller drives
MOVE = 0x21010D06        # move mode, needed before a gait request
HSTEP = 0x21010407       # RUG gait, H-Step gear (high stepping)
RUG = 0x21010401         # RUG gait, general gear
AI_ON = 0x21010528       # enter AI state
AI_GAIT = 0x2101052A     # AI basic gait
AI_OFF = 0x2101052B      # leave AI state

GAITS = {'hstep': HSTEP, 'rug': RUG}


def start_sequence(gait):
    """Walking gait, then navigation mode."""
    return [MOVE, GAITS[gait], AUTO]


def ai_enter_sequence():
    """AI state cannot be entered from navigation mode: manual first."""
    return [MANUAL, AI_ON, AI_GAIT]


def ai_leave_sequence(gait):
    return [AI_OFF, MANUAL, MOVE, GAITS[gait], AUTO]

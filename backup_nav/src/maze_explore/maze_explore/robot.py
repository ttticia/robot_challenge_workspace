"""Lite3 mode and gait requests on /simple_cmd (robot_info.md section 6.3)."""
AUTO = 0x21010C03        # navigation mode: the robot follows /cmd_vel
MANUAL = 0x21010C02      # handheld controller drives
MOVE = 0x21010D06        # move mode, needed before a gait request
HSTEP = 0x21010407       # RUG gait, H-Step gear (high stepping)
RUG = 0x21010401         # RUG gait, general gear
AI_ON = 0x21010528       # enter AI state
AI_GAIT = 0x2101052A     # AI basic gait
AI_OFF = 0x2101052B      # leave AI state
STAND_SIT = 0x21010202   # stand / sit TOGGLE: a standing robot lies down. Send it once

# The only two gaits (user, 2026-11-26):
#   'hstep'  = RUG H-Step, for uneven floor
#   'normal' = the normal gait of Move mode, for flat floor: 0x21010300 ("flat gait, slow",
#              state 0, the gait the robot stands up in). NOT RUG general gear: that was the
#              plain-floor gait for an hour that night by mistake.
# The robot must never switch to grip (user). In the 09:06 run the controller showed grip
# on the uneven blocks; the only gait command sent there was H-Step (0x21010407), so why
# grip showed is not known. Nothing here can request the grip code, RUG general or the
# other two flat speeds: they are not in GAITS, and FORBIDDEN is refused by the sender.
NORMAL = 0x21010300
GRIP = 0x21010402
FORBIDDEN = (GRIP, 0x21010307, 0x21010303)      # grip; flat medium / fast (never run here)
GAITS = {'hstep': HSTEP, 'normal': NORMAL}


def gait(name):
    """One of the two gait names: 'hstep' stays, everything else ('rug', 'flat_slow',
    a typo) is 'normal'."""
    return name if name in GAITS else 'normal'


def start_sequence(gait):
    """Walking gait, then navigation mode."""
    return [MOVE, GAITS.get(gait, NORMAL), AUTO]


def ai_enter_sequence():
    """AI state cannot be entered from navigation mode: manual first."""
    return [MANUAL, AI_ON, AI_GAIT]


def ai_leave_sequence(gait):
    return [AI_OFF, MANUAL, MOVE, GAITS.get(gait, NORMAL), AUTO]

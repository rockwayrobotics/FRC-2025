
# Name constants to avoid typos and duplication.
# These are used for both NetworkTables and DataLog, for consistency and simplicity.
class Name:
    DIST_MM = "/Pi/dist_mm"
    TS_DIST_MM = "/Pi/ts_dist_mm"
    CORNERS = "/Pi/Corners"  # note inconsistent capitalization
    CORNER = "/Pi/corner"
    CORNER_TS = "/Pi/corner_ts"
    CORNER_DIST_MM = "/Pi/corner_dist_mm"
    TOF_MODE = "/Pi/tof_mode"
    CHUTE_MODE = "/Pi/chute_mode"

    # Match state and timing topics
    MATCH_TIME = "/AdvantageKit/DriverStation/MatchTime"
    FMS_ATTACHED = "/AdvantageKit/DriverStation/FMSAttached"
    ENABLED = "/AdvantageKit/DriverStation/Enabled"
    AUTONOMOUS = "/AdvantageKit/DriverStation/Autonomous"
    TIMESTAMP = "/AdvantageKit/Timestamp"
    
    # Match information topics
    MATCH_TYPE = "/AdvantageKit/DriverStation/MatchType"
    MATCH_NUMBER = "/AdvantageKit/DriverStation/MatchNumber"
    REPLAY_NUMBER = "/AdvantageKit/DriverStation/ReplayNumber"
    EVENT_NAME = "/AdvantageKit/DriverStation/EventName"
    

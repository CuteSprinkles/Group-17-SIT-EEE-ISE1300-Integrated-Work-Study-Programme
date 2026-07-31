"""
===============================================================================
 EEE/ISE1300 -- TurtleBot3 line follower with barcode navigation and a
                board-toppling (obstacle) routine.
===============================================================================

WHAT THIS PROGRAM DOES
----------------------
The robot drives along a black line using a Pixy2 camera. Barcodes printed
next to the line tell it what to do at the next junction:

    Barcode 0 = turn LEFT        Barcode 2 = do the OBSTACLE routine
    Barcode 1 = turn RIGHT       Barcode 3 = STOP and finish

The obstacle routine is the tricky part. When barcode 2 is confirmed the
robot saves where it is, uses the LiDAR to work out whether the board is on
its left or its right, turns 90 degrees to face it, drives up to it, extends
the linear actuator, pushes the board over, then reverses back to the exact
spot it started from, restores its original heading and picks the line up
again.

HOW THE CODE IS ORGANISED
-------------------------
Everything is one ROS 2 node (TurtleBot3CombinedController) built around a
state machine. `self.current_state` is always one of the names in the
RobotState class, and `control_loop_from_frame()` is the big method that
decides, for the current state, what velocity to publish and which state to
move to next.

Reading order if you are new to this file:

    1. RobotState + PHASE_LABELS   -- the list of things the robot can be doing
    2. __init__                    -- every tuning constant, grouped by topic
    3. control_loop_from_frame()   -- the main state machine
    4. process_obstacle_scan()     -- the obstacle routine state machine
    5. enter_fail_stop()           -- what happens when something goes wrong

THREE IDEAS THAT EXPLAIN MOST OF THE CODE
-----------------------------------------
1. VOTING. Nothing is believed from a single reading. A barcode must be read
   several times and win a majority; the obstacle side must win several LiDAR
   scans in a row. One bad frame should never make the robot turn.

2. ONE EXIT POINT. Every velocity command goes through publish_motion().
   That is the only place /cmd_vel is written, so the safety overrides cannot
   be bypassed by accident from somewhere inside the state machine.

3. FAIL SAFE, NOT FAIL SILENT. If a manoeuvre cannot be completed the robot
   does NOT guess. It latches a fail-stop: wheels stopped, fast red flash,
   siren, and a printed recovery card. Only a human can clear it.

SENSORS AND WHAT THEY ARE FOR
-----------------------------
    Pixy2 camera  -> the line, junctions and barcodes
    LiDAR (/scan) -> finding the board, aiming at it, measuring the approach
    Odometry      -> distances and turn angles (metres and degrees)

HOW TO RUN IT (three terminals)
-------------------------------
    TERMINAL 1  ros2 launch turtlebot3_bringup robot.launch.py
    TERMINAL 2  (kept free for keyboard teleop during a fail-stop)
    TERMINAL 3  python3 V26_2.py

USEFUL ENVIRONMENT VARIABLES
----------------------------
    CONSOLE_STYLE=friendly   short plain-English output (default)
    CONSOLE_STYLE=debug      every number, for when something is wrong
    NO_COLOUR=1              plain text, e.g. when piping into a log file
    ENABLE_ACTUATOR=0        dry run: full routine, actuator never powered
    FAIL_STOP_HTML=0         do not write the HTML fail-stop report

GPIO PINS (BCM numbering)
-------------------------
    Left LED  = 23      Stop/red LED = 25
    Right LED = 24      Actuator     = 18
"""

import math
import os
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from statistics import median
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

# =============================================================================
# OPTIONAL GPIO
# =============================================================================

try:
    import RPi.GPIO as GPIO

    GPIO_AVAILABLE = True
except Exception as exc:
    print(f"RPi.GPIO import failed: {exc}")
    GPIO_AVAILABLE = False
    GPIO = None

# =============================================================================
# OPTIONAL TURTLEBOT3 SOUND SERVICE
# =============================================================================

try:
    from turtlebot3_msgs.srv import Sound

    SOUND_AVAILABLE = True
except Exception as exc:
    print(f"turtlebot3_msgs.srv.Sound import failed: {exc}")
    SOUND_AVAILABLE = False
    Sound = None

# =============================================================================
# PIXY2 PYTHON BINDINGS
# =============================================================================

PIXYPATH = os.path.expanduser("~/pixy2/build/python_demos")

if PIXYPATH not in sys.path:
    sys.path.append(PIXYPATH)

try:
    import pixy
    from pixy import BarcodeArray, IntersectionArray, VectorArray
    from pixy import line_get_all_features
    from pixy import line_get_barcodes
    from pixy import line_get_intersections
    from pixy import line_get_vectors

    PIXY_AVAILABLE = True
except Exception as exc:
    print(f"Pixy module import failed: {exc}")
    PIXY_AVAILABLE = False
    pixy = None


# =============================================================================
# STATES
# =============================================================================

class RobotState:
    LINE_FOLLOW = "LINE_FOLLOW"
    LINE_FOLLOW_ARMED = "LINE_FOLLOW_ARMED"

    MOVE_TO_TURN_CENTER = "MOVE_TO_TURN_CENTER"
    EXECUTE_TURN_ODOM = "EXECUTE_TURN_ODOM"
    SNAP_BACK_TO_LINE = "SNAP_BACK_TO_LINE"

    OBSTACLE_SCAN_SETTLE = "OBSTACLE_SCAN_SETTLE"
    OBSTACLE_SEARCH = "OBSTACLE_SEARCH"
    OBSTACLE_RESCAN_SETTLE = "OBSTACLE_RESCAN_SETTLE"
    OBSTACLE_TURN_LEFT = "OBSTACLE_TURN_LEFT"
    OBSTACLE_TURN_RIGHT = "OBSTACLE_TURN_RIGHT"
    OBSTACLE_FINE_ALIGN_SETTLE = "OBSTACLE_FINE_ALIGN_SETTLE"
    OBSTACLE_FINE_ALIGN = "OBSTACLE_FINE_ALIGN"
    OBSTACLE_FRONT_NAN_REAR_CHECK = "OBSTACLE_FRONT_NAN_REAR_CHECK"
    OBSTACLE_FRONT_NAN_REVERSE = "OBSTACLE_FRONT_NAN_REVERSE"
    OBSTACLE_CENTERING_EXTRA = "OBSTACLE_CENTERING_EXTRA"
    OBSTACLE_APPROACH = "OBSTACLE_APPROACH"
    OBSTACLE_BRAKE = "OBSTACLE_BRAKE"
    OBSTACLE_ACTUATOR_EXTEND = "OBSTACLE_ACTUATOR_EXTEND"
    OBSTACLE_TACKLE = "OBSTACLE_TACKLE"
    OBSTACLE_TACKLE_HOLD = "OBSTACLE_TACKLE_HOLD"
    OBSTACLE_RETURN = "OBSTACLE_RETURN"
    OBSTACLE_FRONT_NAN_RETURN_FORWARD = "OBSTACLE_FRONT_NAN_RETURN_FORWARD"
    OBSTACLE_RETURN_YAW = "OBSTACLE_RETURN_YAW"
    OBSTACLE_REACQUIRE_LINE = "OBSTACLE_REACQUIRE_LINE"
    OBSTACLE_FAIL_STOP = "OBSTACLE_FAIL_STOP"

    LINE_LOST_ALARM = "LINE_LOST_ALARM"
    STOP_COMMAND = "STOP_COMMAND"


# Plain-English label for each state, used by the friendly console output so
# a beginner can tell at a glance what the robot thinks it is doing.
PHASE_LABELS: Dict[str, str] = {
    RobotState.LINE_FOLLOW: "following the line",
    RobotState.LINE_FOLLOW_ARMED: "following the line (turn stored)",
    RobotState.MOVE_TO_TURN_CENTER: "driving into the junction",
    RobotState.EXECUTE_TURN_ODOM: "turning",
    RobotState.SNAP_BACK_TO_LINE: "re-centring on the line",
    RobotState.OBSTACLE_SCAN_SETTLE: "obstacle: settling",
    RobotState.OBSTACLE_SEARCH: "obstacle: looking for the board",
    RobotState.OBSTACLE_RESCAN_SETTLE: "obstacle: settling for a re-scan",
    RobotState.OBSTACLE_TURN_LEFT: "obstacle: turning left to the board",
    RobotState.OBSTACLE_TURN_RIGHT: "obstacle: turning right to the board",
    RobotState.OBSTACLE_FINE_ALIGN_SETTLE: "obstacle: settling before aiming",
    RobotState.OBSTACLE_FINE_ALIGN: "obstacle: aiming at the board",
    RobotState.OBSTACLE_FRONT_NAN_REAR_CHECK: "obstacle: checking space behind",
    RobotState.OBSTACLE_FRONT_NAN_REVERSE: "obstacle: backing up to see board",
    RobotState.OBSTACLE_CENTERING_EXTRA: "obstacle: aiming at the board",
    RobotState.OBSTACLE_APPROACH: "obstacle: driving to the board",
    RobotState.OBSTACLE_BRAKE: "obstacle: braking",
    RobotState.OBSTACLE_ACTUATOR_EXTEND: "obstacle: extending the pusher",
    RobotState.OBSTACLE_TACKLE: "obstacle: PUSHING the board",
    RobotState.OBSTACLE_TACKLE_HOLD: "obstacle: holding against the board",
    RobotState.OBSTACLE_RETURN: "obstacle: reversing back to the track",
    RobotState.OBSTACLE_FRONT_NAN_RETURN_FORWARD: "obstacle: undoing the backup",
    RobotState.OBSTACLE_RETURN_YAW: "obstacle: restoring the heading",
    RobotState.OBSTACLE_REACQUIRE_LINE: "obstacle: finding the line again",
    RobotState.OBSTACLE_FAIL_STOP: "FAIL-STOP (needs you)",
    RobotState.LINE_LOST_ALARM: "line lost - searching",
    RobotState.STOP_COMMAND: "STOPPED (barcode 3)",
}


# =============================================================================
# PIXY DATA CLASSES
# =============================================================================

@dataclass
class VectorInfo:
    index: int
    x0: float
    y0: float
    x1: float
    y1: float
    bottom_x: float
    bottom_y: float
    top_x: float
    top_y: float
    center_x: float
    center_y: float
    length: float
    y_span: float
    x_span: float
    angle_deg: float
    flags: int

    def x_at_y(self, target_y: float) -> float:
        """Estimate the vector x-coordinate at a selected image y-coordinate."""
        dy = self.y1 - self.y0

        if abs(dy) < 1e-6:
            return self.center_x

        t = (target_y - self.y0) / dy
        return self.x0 + t * (self.x1 - self.x0)


@dataclass
class PixyFrame:
    vector_count: int
    barcode_count: int
    intersection_count: int
    reliable_vectors: List[VectorInfo]
    best_vector: Optional[VectorInfo]
    line_error: Optional[float]
    raw_intersection: bool
    crossing_point: Optional[Tuple[float, float]]
    intersection_reason: str


@dataclass
class LidarBoardTarget:
    distance: float
    bearing: float
    point_count: int
    width_m: float
    # Closest single point in the cluster. `distance` is the chord midpoint,
    # which over-reports how near the board really is when the board is seen
    # at an angle.
    near_distance: float = float("inf")
    # Forward distance to the board face measured ALONG THE PUSHER AXIS
    # (the robot's own x-axis, y = 0). This is the number the actuator
    # actually has to travel through. It is +inf when the pusher axis misses
    # the cluster completely, in which case near_distance is used instead.
    axis_distance: float = float("inf")
    # Signed lateral (left-positive) coordinates of the two cluster ends.
    # Used to work out how far the aim point may sit from the board centre
    # and still land on the board.
    left_edge_y: float = 0.0
    right_edge_y: float = 0.0

    @property
    def bearing_deg(self) -> float:
        """Same bearing as `bearing`, but in degrees (easier to read in logs)."""
        return math.degrees(self.bearing)


# =============================================================================
# MAIN NODE
# =============================================================================

class TurtleBot3CombinedController(Node):
    def __init__(self) -> None:
        """Set up the node: ROS interfaces, hardware, and every tuning number.

        This is long because it is where ALL the tuning lives. It is grouped
        into labelled sections, so if you want to change how fast the robot
        drives, find the "Normal line-following motion" section rather than
        hunting through the state machine.

        Naming convention used throughout the file:

            CAPITALS  = a setting. Set once here, never changed while running.
            lowercase = a live value that changes as the robot drives.

        Most CAPITALS can also be overridden without editing the file, using
        an environment variable of the same name, e.g.:

            FORWARD_SPEED=0.12 python3 V26_2.py
        """
        super().__init__("turtlebot3_combined_controller")
        # Sensor callbacks, the Pixy control loop and the watchdog use
        # different executor threads. Protect FSM transitions while still
        # leaving the potentially blocking Pixy read outside this lock.
        self.state_lock = threading.RLock()

        # ---------------------------------------------------------------------
        # Console presentation
        #
        # CONSOLE_STYLE=friendly (default) -> short, colour-coded, plain
        #   English. One status line per second. Best for driving the robot.
        # CONSOLE_STYLE=debug -> everything, including per-scan numbers.
        #   Use this when something is going wrong and you need the detail.
        # NO_COLOUR=1 disables ANSI colour (e.g. when piping to a log file).
        # ---------------------------------------------------------------------
        self.CONSOLE_STYLE = os.environ.get(
            "CONSOLE_STYLE", "friendly"
        ).strip().lower()
        self.VERBOSE = self.CONSOLE_STYLE == "debug"
        self.USE_COLOUR = (
            os.environ.get("NO_COLOUR", "0") != "1"
            and bool(getattr(sys.stdout, "isatty", lambda: False)())
        )
        self._throttle_times: Dict[str, float] = {}

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------

        # Keep the blocking Pixy2 polling timer separate from ROS sensor callbacks.
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.odom_group = MutuallyExclusiveCallbackGroup()
        self.scan_group = MutuallyExclusiveCallbackGroup()
        self.watchdog_group = MutuallyExclusiveCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
            callback_group=self.odom_group,
        )

        # Match the LD08 publisher: BEST_EFFORT and VOLATILE.
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            scan_qos,
            callback_group=self.scan_group,
        )

        # ---------------------------------------------------------------------
        # GPIO pin assignments
        # ---------------------------------------------------------------------

        self.LEFT_LED = 23
        self.RIGHT_LED = 24
        self.STOP_LED = 25
        self.SERVO_PIN = 18

        self.gpio_ready = False
        self.pwm = None

        if GPIO_AVAILABLE:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)

                GPIO.setup(self.LEFT_LED, GPIO.OUT)
                GPIO.setup(self.RIGHT_LED, GPIO.OUT)
                GPIO.setup(self.STOP_LED, GPIO.OUT)
                GPIO.setup(self.SERVO_PIN, GPIO.OUT)

                self.pwm = GPIO.PWM(self.SERVO_PIN, 50)
                self.pwm.start(0)

                self.gpio_ready = True
                self.all_leds_off()

                self.get_logger().info(
                    "GPIO ready: LEFT=23, RIGHT=24, STOP=25, ACTUATOR=18."
                )
            except Exception as exc:
                self.get_logger().error(f"GPIO setup failed: {exc}")
                self.gpio_ready = False
        else:
            self.get_logger().warning("RPi.GPIO unavailable; LEDs and actuator disabled.")

        # ---------------------------------------------------------------------
        # LED and buzzer settings
        # ---------------------------------------------------------------------

        self.blink_state = False
        self.last_blink_time = time.time()

        # Ordinary lost-line alarm: slow blink + single occasional beep.
        # Fail-stop: very fast blink + a rising/falling siren, repeated often.
        # The two must never be confusable from across the room.
        self.BLINK_INTERVAL = 0.30
        self.FAIL_STOP_BLINK_INTERVAL = float(
            os.environ.get("FAIL_STOP_BLINK_INTERVAL", "0.07")
        )
        self.FAIL_STOP_REMINDER_SEC = float(
            os.environ.get("FAIL_STOP_REMINDER_SEC", "15.0")
        )
        # Everything we want to be able to tell the operator about the last
        # fail-stop. These are filled in by enter_fail_stop() and are what the
        # printed card and the HTML report are built from.
        self.fail_stop_count = 0
        self.fail_stop_reason = ""
        self.fail_stop_state_at_fault = ""
        self.fail_stop_pose = ""
        self.fail_stop_time_text = ""
        self.last_fail_stop_reminder_time = 0.0

        # The fail-stop card is also written to an HTML file, so it can be
        # opened in a browser (real colours, easy to screenshot for the report)
        # instead of being scrolled away in the terminal. FAIL_STOP_HTML=0
        # turns this off.
        self.WRITE_FAIL_STOP_HTML = os.environ.get("FAIL_STOP_HTML", "1") != "0"
        self.FAIL_STOP_HTML_PATH = os.path.expanduser(
            os.environ.get(
                "FAIL_STOP_HTML_PATH", "~/turtlebot3_fail_stop.html"
            )
        )
        self.fail_stop_html_written_path = ""

        self.TURN_LED_THRESHOLD = 0.12

        self.sound_client = None
        self.sound_ready = False
        self.last_buzzer_time = 0.0
        self.buzzer_queue: Deque[Tuple[float, int]] = deque()

        self.BUZZER_INTERVAL = 0.90
        self.BUZZER_SOUND_VALUE = 1
        self.FAIL_STOP_BUZZER_REPEAT_SEC = float(
            os.environ.get("FAIL_STOP_BUZZER_REPEAT_SEC", "2.0")
        )
        self.RESET_FAIL_STOP_COMMAND = (
            "ros2 service call "
            "/turtlebot3_combined_controller/reset_fail_stop "
            'std_srvs/srv/Trigger "{}"'
        )

        # Different short patterns identify the current state. The numbers are
        # TurtleBot3 Sound service values, not frequencies.
        self.BEEP_BARCODE = {
            0: [1],       # LEFT
            1: [3],       # RIGHT
            2: [4, 4],    # OBSTACLE
            3: [5, 5],    # STOP
        }
        self.BEEP_RESCAN = [2, 2]
        self.BEEP_NAN_CLOSE_BOARD = [2, 5, 2, 5]
        self.BEEP_FRONT_NAN_REVERSE = [5, 2, 5, 2]
        self.BEEP_SIDE_CONFIRMED = [4]
        self.BEEP_ALIGNED = [5]
        self.BEEP_TACKLE = [5, 4, 5]
        self.BEEP_SUCCESS = [4, 5]
        self.BEEP_FAILURE = [3, 3, 3]
        # Reserved ONLY for a latched fail-stop: a rising-then-falling siren
        # that sounds nothing like any other state in this program.
        self.BEEP_FAIL_STOP = [1, 2, 3, 4, 5, 4, 3, 2, 1, 5, 1, 5]

        if SOUND_AVAILABLE:
            try:
                self.sound_client = self.create_client(Sound, "/sound")
                # The service may become ready shortly after startup. Keep the
                # client enabled and check service_is_ready() before each call.
                self.sound_ready = False
                self.get_logger().info("Created TurtleBot3 /sound client.")
            except Exception as exc:
                self.get_logger().warning(f"Could not create /sound client: {exc}")

        # ---------------------------------------------------------------------
        # Pixy2 image parameters
        # ---------------------------------------------------------------------

        # The Pixy2 line-tracking image is 79 x 52 'pixels' wide, so the middle
        # column is 39. FRAME_CENTER_X is therefore where the line SHOULD be;
        # anything else is the steering error the PD controller works on.
        # TRACK_Y is the row we measure that error at: near the bottom of the
        # image, i.e. the piece of line closest to the robot right now.

        self.FRAME_CENTER_X = 39.0
        self.TRACK_Y = 45.0

        # ---------------------------------------------------------------------
        # Normal line-following motion
        # ---------------------------------------------------------------------

        # Speeds are in metres per second. There are several because the robot
        # should not take a tight bend at the same speed as a straight, and
        # should be a little more careful once a turn command is armed.

        # Slightly faster than the friend's original 0.15 m/s, while still
        # slowing automatically when the Pixy line error becomes large.
        self.FORWARD_SPEED = float(
            os.environ.get("FORWARD_SPEED", "0.16")
        )
        self.ARMED_FORWARD_SPEED = float(
            os.environ.get("ARMED_FORWARD_SPEED", "0.15")
        )
        self.CURVE_FORWARD_SPEED = float(
            os.environ.get("CURVE_FORWARD_SPEED", "0.14")
        )
        self.SLOW_FORWARD_SPEED = float(
            os.environ.get("SLOW_FORWARD_SPEED", "0.15")
        )
        self.CENTERING_SPEED = float(
            os.environ.get("CENTERING_SPEED", "0.15")
        )
        self.CURVE_FULL_SLOW_ERROR_PX = float(
            os.environ.get("CURVE_FULL_SLOW_ERROR_PX", "18.0")
        )

        # Preserve the friend's turn speeds.
        self.TURN_SPEED = float(
            os.environ.get("TURN_SPEED", "0.95")
        )
        self.MAX_TURN_SPEED = float(
            os.environ.get("MAX_TURN_SPEED", "0.95")
        )

        self.kp = float(os.environ.get("LINE_KP", "0.050"))
        self.kd = float(os.environ.get("LINE_KD", "0.012"))
        self.prev_error = 0.0

        self.FORWARD_DISTANCE_AFTER_INTERSECTION_M = 0.1
        self.MOVE_CENTER_TIMEOUT_SEC = float(
            os.environ.get("MOVE_CENTER_TIMEOUT_SEC", "3.0")
        )

        self.TURN_TARGET_RAD = math.radians(82.0)
        self.TURN_TOLERANCE_RAD = math.radians(3.0)
        self.MAX_TURN_TIME_SEC = 4.0

        # ---------------------------------------------------------------------
        # Pixy vector filtering and intersection detection
        # ---------------------------------------------------------------------

        # The Pixy2 reports every line-ish thing it can see, including short
        # specks of noise. These thresholds throw away anything too short, too
        # flat or too far up the image to be the line the robot is sitting on.
        # Units are Pixy image pixels, not centimetres.

        self.MIN_VECTOR_LENGTH_PX = 10.0
        self.MIN_FOLLOW_Y_SPAN_PX = 7.0
        self.MIN_BOTTOM_Y_PX = 18.0
        self.MAX_ACCEPTED_ERROR_PX = 38.0
        self.MAX_LINE_X_JUMP_PX = 26.0

        self.MIN_CROSSING_ANGLE_DEG = 35.0
        self.CROSSING_SEGMENT_TOL_PX = 5.0
        self.INTERSECTION_CENTER_X_TOL_PX = 26.0

        self.INTERSECTION_TRIGGER_Y_MIN = 14.0
        self.INTERSECTION_TRIGGER_Y_MAX = 51.5

        # The control loop now runs at 20 Hz instead of 10 Hz. Frame counts
        # are increased so the real-time line-loss/reacquisition behaviour does
        # not become unnecessarily sensitive.
        self.LINE_LOST_FRAMES = 8
        self.LINE_REACQUIRE_FRAMES = 5
        # Collect at most three valid Pixy barcode reads in a short window.
        # A command is accepted only when one code has a strict 2-of-3
        # majority. Once two identical reads are present the result is already
        # mathematically decided, so it can be accepted without waiting for a
        # third read and risking that the junction passes underneath the robot.
        self.BARCODE_VOTE_SAMPLES = int(
            os.environ.get("BARCODE_VOTE_SAMPLES", "3")
        )
        self.BARCODE_MIN_VOTES = int(
            os.environ.get("BARCODE_MIN_VOTES", "2")
        )
        self.BARCODE_VOTE_WINDOW_SEC = float(
            os.environ.get("BARCODE_VOTE_WINDOW_SEC", "0.30")
        )
        self.BARCODE_VOTE_SPEED = float(
            os.environ.get("BARCODE_VOTE_SPEED", "0.09")
        )
        self.BARCODE_CLEAR_FRAMES = 6
        self.INTERSECTION_CONFIRM_FRAMES = 2
        self.INTERSECTION_COOLDOWN_SEC = 0.5
        # ---------------------------------------------------------------------
        # BARCODES PLACED CLOSE TOGETHER
        #
        # The old single 0.8 s accept cooldown applied to EVERY barcode, so a
        # second, DIFFERENT barcode a few centimetres after the first was
        # silently thrown away: at 0.15 m/s, 30 mm of track is only 0.2 s.
        # The cooldown is only there to stop one physical barcode being
        # accepted twice, and barcode_latched_code already does that job for
        # a repeat of the SAME code -- so the cooldown now applies to a
        # repeat of the same code only.
        # ---------------------------------------------------------------------
        self.BARCODE_ACCEPT_COOLDOWN = float(
            os.environ.get("BARCODE_ACCEPT_COOLDOWN", "0.8")
        )
        self.BARCODE_ACCEPT_COOLDOWN_DIFFERENT = float(
            os.environ.get("BARCODE_ACCEPT_COOLDOWN_DIFFERENT", "0.0")
        )
        # A barcode is also un-latched once the robot has physically driven
        # this far past it, not only after BARCODE_CLEAR_FRAMES empty Pixy
        # frames. With two barcodes close together the camera may never see a
        # gap, so the frame-count rule alone can never re-arm.
        #
        # NOTE: this is what limits how close two barcodes carrying the SAME
        # code may be. Two different codes are separated by the code itself
        # and are not affected. Lower this if your identical barcodes are
        # closer together than the default, but not below the distance the
        # robot travels while one barcode is in the Pixy's view, or one
        # barcode will be read twice.
        self.BARCODE_LATCH_CLEAR_DISTANCE_M = float(
            os.environ.get("BARCODE_LATCH_CLEAR_DISTANCE_M", "0.06")
        )
        # How many turn commands may be stored at once. The old code kept
        # exactly one and dropped any barcode read while a command was still
        # pending, so a second barcode before the junction was lost.
        self.MAX_QUEUED_COMMANDS = int(
            os.environ.get("MAX_QUEUED_COMMANDS", "2")
        )
        # Do not impose a time-based blind period after a normal turn. The
        # barcode latch/clear logic below already prevents the just-used
        # physical barcode from retriggering. A blanket delay can completely
        # miss a different barcode placed only a few centimetres after the
        # junction.
        self.BARCODE_IGNORE_AFTER_COMMAND_SEC = float(
            os.environ.get("BARCODE_IGNORE_AFTER_COMMAND_SEC", "0.00")
        )
        # A short blanking window after Barcode 2 as a second layer behind the
        # distance-based lockout. The lockout is the real protection; this
        # just covers the moment the robot is still swinging around on top of
        # the barcode it has already used.
        #
        # It now suppresses BARCODE 2 ONLY. Blanking every code here meant a
        # different barcode a few centimetres past Barcode 2 was skipped: at
        # the 0.13 m/s reacquire speed, 0.40 s covers 52 mm of track.
        self.POST_OBSTACLE_BARCODE_IGNORE_SEC = float(
            os.environ.get("POST_OBSTACLE_BARCODE_IGNORE_SEC", "0.40")
        )

        # Barcode 2 needs a special re-arm rule. A time-only cooldown is not
        # enough because the robot returns to almost the same physical point
        # and may see the same barcode again after its normal latch is cleared.
        #
        # BARCODE 2 becomes available again only after:
        #   1. it has been absent for several consecutive Pixy frames; and
        #   2. the robot has moved a minimum distance away from the saved
        #      Barcode 2 / obstacle start position.
        #
        # Other barcodes (0, 1 and 3) stay READABLE inside this zone, but a
        # cropped or oblique view of Barcode 2 can decode as 0, 1 or 3, so
        # inside the zone they must win a UNANIMOUS vote instead of the
        # normal 2-of-3 majority. A real neighbouring barcode produces three
        # identical reads easily; a misread of Barcode 2 does not.
        self.BARCODE2_REARM_CLEAR_FRAMES = int(
            os.environ.get("BARCODE2_REARM_CLEAR_FRAMES", "12")
        )
        self.BARCODE2_REARM_DISTANCE_M = float(
            os.environ.get("BARCODE2_REARM_DISTANCE_M", "0.10")
        )
        self.BARCODE2_ZONE_MIN_VOTES = int(
            os.environ.get("BARCODE2_ZONE_MIN_VOTES", "3")
        )

        self.DEBUG_VECTOR_LOG = False

        # STRAIGHT has been removed.
        self.BARCODE_COMMANDS: Dict[int, str] = {
            0: "LEFT",
            1: "RIGHT",
            2: "OBSTACLE",
            3: "STOP",
        }

        # ---------------------------------------------------------------------
        # Obstacle-routine tuning
        # ---------------------------------------------------------------------

        # ---- Physical geometry of THIS robot (measure before competition) ----
        #
        #        LiDAR axis                       board face
        #            |                                 |
        #            |<-- LIDAR_TO_FRONT_EDGE_M ->|    |
        #            |                        (bumper) |
        #            |<---------- STOP_DISTANCE_M ---->|
        #            |                            |<-->|  gap to close
        #
        # LIDAR_TO_FRONT_EDGE_M: LiDAR rotation axis (the centre of the LiDAR)
        #   to the tip of the actuator WITH THE ACTUATOR RETRACTED. This is
        #   the front-most point of this robot.
        #   MEASURED ON THIS ROBOT = 0.14 m.
        # ACTUATOR_REACH_M: how much FURTHER the pusher tip reaches past that
        #   retracted position when the actuator is FULLY EXTENDED.
        #   >>> MEASURE THIS WITH A RULER AND SET IT. The default is a guess. <<<
        # TOPPLE_OVERTRAVEL_M: extra travel after first contact so the board is
        #   pushed past its tipping point instead of merely being touched.
        #
        # REAR_OVERHANG_M (further down) is the matching rear number: LiDAR
        # centre to the back of the plate that carries the soldered LED board.
        self.STOP_DISTANCE_M = float(
            os.environ.get("STOP_DISTANCE_M", "0.20")
        )
        self.LIDAR_TO_FRONT_EDGE_M = float(
            os.environ.get("LIDAR_TO_FRONT_EDGE_M", "0.14")
        )
        self.ACTUATOR_REACH_M = float(
            os.environ.get("ACTUATOR_REACH_M", "0.05")
        )
        self.TOPPLE_OVERTRAVEL_M = float(
            os.environ.get("TOPPLE_OVERTRAVEL_M", "0.04")
        )

        self.APPROACH_SPEED = 0.10
        self.REVERSE_SPEED = -0.20
        self.RETURN_TOLERANCE_M = 0.02
        self.REVERSE_SLOW_SPEED = -0.10
        self.REVERSE_SLOW_ZONE_M = 0.04
        self.REVERSE_STOP_MARGIN_M = 0.04
        self.REVERSE_LINE_CONFIRM_FRAMES = 5
        self.STOP_HOLD_DURATION = 0.5

        self.NUDGE_SPEED = 0.06
        self.BOARD_DOWN_THRESHOLD_M = 0.35
        self.ACTUATOR_EXTEND_HOLD_SEC = float(
            os.environ.get("ACTUATOR_EXTEND_HOLD_SEC", "3.0")
        )
        self.TACKLE_PUSH_SPEED = float(
            os.environ.get("TACKLE_PUSH_SPEED", "0.04")
        )
        self.TACKLE_MIN_PUSH_DISTANCE_M = float(
            os.environ.get("TACKLE_MIN_PUSH_DISTANCE_M", "0.04")
        )
        self.TACKLE_MAX_PUSH_DISTANCE_M = float(
            os.environ.get("TACKLE_MAX_PUSH_DISTANCE_M", "0.25")
        )
        self.TACKLE_BOARD_DOWN_CONFIRM_SCANS = int(
            os.environ.get("TACKLE_BOARD_DOWN_CONFIRM_SCANS", "3")
        )
        self.TACKLE_CONTACT_HOLD_SEC = float(
            os.environ.get("TACKLE_CONTACT_HOLD_SEC", "0.60")
        )
        self.TACKLE_TIMEOUT_SEC = float(
            os.environ.get("TACKLE_TIMEOUT_SEC", "20.0")
        )
        # Contact/stall detection. If the wheels are commanded forward but the
        # odometry barely advances, the pusher is loaded against the board.
        self.TACKLE_STALL_PROGRESS_M = float(
            os.environ.get("TACKLE_STALL_PROGRESS_M", "0.004")
        )
        self.TACKLE_STALL_CONFIRM_SCANS = int(
            os.environ.get("TACKLE_STALL_CONFIRM_SCANS", "4")
        )
        # Hold the board-facing heading during the push so asymmetric wheel
        # slip on contact cannot walk the pusher off the edge of the board.
        self.TACKLE_YAW_HOLD_KP = float(
            os.environ.get("TACKLE_YAW_HOLD_KP", "1.2")
        )
        self.TACKLE_YAW_HOLD_MAX = float(
            os.environ.get("TACKLE_YAW_HOLD_MAX", "0.10")
        )
        self.RETURN_TIMEOUT_SEC = 20.0

        # Freshness watchdog. A transient sensor interruption pauses all motion
        # and may resume the same FSM checkpoint. A prolonged or repeatedly
        # flapping interruption enters a latched fail-stop instead of replaying
        # old commands or attempting an unsafe time-based rewind.
        self.SENSOR_STALE_TIMEOUT_SEC = float(
            os.environ.get("SENSOR_STALE_TIMEOUT_SEC", "0.75")
        )
        self.SENSOR_RECOVERY_GRACE_SEC = float(
            os.environ.get("SENSOR_RECOVERY_GRACE_SEC", "3.0")
        )
        self.SENSOR_RECOVERY_HEALTHY_CYCLES = int(
            os.environ.get("SENSOR_RECOVERY_HEALTHY_CYCLES", "3")
        )
        self.SENSOR_MAX_RECOVERIES = int(
            os.environ.get("SENSOR_MAX_RECOVERIES", "2")
        )
        self.FAIL_STOP_COMMAND_HOLD_SEC = float(
            os.environ.get("FAIL_STOP_COMMAND_HOLD_SEC", "1.0")
        )
        self.WATCHDOG_PERIOD_SEC = float(
            os.environ.get("WATCHDOG_PERIOD_SEC", "0.05")
        )

        # The LDS-02 specified minimum range is 0.16 m. Measurements below
        # that value are not treated as valid distances. A repeated NaN
        # pattern is handled separately and only while Barcode 2 is active.
        self.SIDE_MIN_RANGE_M = float(
            os.environ.get("SIDE_MIN_RANGE_M", "0.16")
        )
        self.SIDE_MAX_RANGE_M = float(
            os.environ.get("SIDE_MAX_RANGE_M", "0.60")
        )
        self.FRONT_MIN_RANGE_M = float(
            os.environ.get("FRONT_MIN_RANGE_M", "0.16")
        )
        self.FRONT_MAX_RANGE_M = float(
            os.environ.get("FRONT_MAX_RANGE_M", "0.60")
        )

        # Five scans with a three-vote majority. Finite-distance consensus is
        # used unless a repeated one-sided NaN consensus indicates an even
        # nearer board inside the LDS-02 minimum range.
        self.SIDE_SCAN_COUNT = 5
        self.SIDE_REQUIRED_VOTES = 3
        self.SIDE_MAX_ATTEMPTS = 2
        # If two finite boards differ by less than 2 cm, the LDS-02 readings
        # are effectively tied; rescan instead of choosing from sensor noise.
        self.SIDE_BOTH_DISTANCE_MARGIN_M = 0.02
        self.SIDE_MAX_DISTANCE_SPREAD_M = 0.30

        self.INITIAL_SCAN_SETTLE_DURATION_SEC = 0.40
        self.RESCAN_SETTLE_DURATION_SEC = 0.40

        # Close-board NaN classifier. One NaN is meaningless; require a
        # strong one-sided pattern and repeated votes.
        self.NAN_SIDE_MIN_RATIO = float(
            os.environ.get("NAN_SIDE_MIN_RATIO", "0.55")
        )
        self.NAN_SIDE_ADVANTAGE = float(
            os.environ.get("NAN_SIDE_ADVANTAGE", "0.30")
        )
        self.NAN_VOTE_WINDOW = int(
            os.environ.get("NAN_VOTE_WINDOW", "5")
        )
        self.NAN_REQUIRED_VOTES = int(
            os.environ.get("NAN_REQUIRED_VOTES", "3")
        )

        # If the board is still inside the LDS-02 minimum range after the
        # 90-degree turn, require three NaN-heavy front scans before backing
        # away by 0.10 m. The rear-clearance requirement covers the motion,
        # the measured 0.14 m rear overhang, and a 0.03 m safety margin.
        self.FRONT_NAN_HALF_ANGLE_DEG = float(
            os.environ.get("FRONT_NAN_HALF_ANGLE_DEG", "25.0")
        )
        self.FRONT_NAN_MIN_RATIO = float(
            os.environ.get("FRONT_NAN_MIN_RATIO", "0.50")
        )
        self.FRONT_NAN_CONFIRM_SCANS = int(
            os.environ.get("FRONT_NAN_CONFIRM_SCANS", "3")
        )
        # Treat an off-axis cluster as an edge-return artefact (rather than
        # the board face) when the pusher axis itself has no return and the
        # front sector is mostly too-close returns. Set to 0 to restore the
        # old behaviour, where any cluster at all blocked the close-board
        # reverse.
        self.FRONT_NAN_EDGE_FALLBACK = bool(
            int(os.environ.get("FRONT_NAN_EDGE_FALLBACK", "1"))
        )
        self.FRONT_NAN_REVERSE_DISTANCE_M = float(
            os.environ.get("FRONT_NAN_REVERSE_DISTANCE_M", "0.10")
        )
        self.FRONT_NAN_REVERSE_SPEED = float(
            os.environ.get("FRONT_NAN_REVERSE_SPEED", "-0.05")
        )
        self.FRONT_NAN_RETURN_SPEED = float(
            os.environ.get("FRONT_NAN_RETURN_SPEED", "0.05")
        )
        self.FRONT_NAN_HEADING_KP = float(
            os.environ.get("FRONT_NAN_HEADING_KP", "1.5")
        )
        self.FRONT_NAN_MAX_ANGULAR_SPEED = float(
            os.environ.get("FRONT_NAN_MAX_ANGULAR_SPEED", "0.15")
        )
        # Rear geometry of THIS robot: the plate that supports the soldered
        # LED board sticks out behind the chassis, so (a) every reverse needs
        # this much extra clearance, and (b) the rear LiDAR sector partly
        # sees the plate itself. The rear self-reading baseline below cancels
        # that self-view out. Measure LiDAR centre -> back of the plate.
        self.REAR_OVERHANG_M = float(
            os.environ.get("REAR_OVERHANG_M", "0.14")
        )
        self.REAR_NAN_REVERSE_MARGIN_M = float(
            os.environ.get("REAR_NAN_REVERSE_MARGIN_M", "0.03")
        )
        self.REAR_NAN_INCREASE_BLOCK_RATIO = float(
            os.environ.get("REAR_NAN_INCREASE_BLOCK_RATIO", "0.20")
        )
        self.REAR_BASELINE_MATCH_TOLERANCE_M = float(
            os.environ.get("REAR_BASELINE_MATCH_TOLERANCE_M", "0.03")
        )
        # Stage 1: complete an odometry-based 90-degree turn. The speed is
        # reduced in two steps near the target to avoid the old overshoot.
        self.OBSTACLE_COARSE_TURN_DEG = float(
            os.environ.get("OBSTACLE_COARSE_TURN_DEG", "90.0")
        )
        self.OBSTACLE_TURN_TOLERANCE_RAD = math.radians(
            float(os.environ.get("OBSTACLE_TURN_TOLERANCE_DEG", "3.0"))
        )
        self.OBSTACLE_TURN_SPEED = float(
            os.environ.get("OBSTACLE_TURN_SPEED", "0.60")
        )
        self.OBSTACLE_TURN_SLOW_SPEED = float(
            os.environ.get("OBSTACLE_TURN_SLOW_SPEED", "0.28")
        )
        self.OBSTACLE_TURN_CREEP_SPEED = float(
            os.environ.get("OBSTACLE_TURN_CREEP_SPEED", "0.10")
        )
        self.OBSTACLE_TURN_SLOW_ZONE_RAD = math.radians(22.0)
        self.OBSTACLE_TURN_CREEP_ZONE_RAD = math.radians(8.0)
        self.OBSTACLE_TURN_TIMEOUT_SEC = float(
            os.environ.get("OBSTACLE_TURN_TIMEOUT_SEC", "6.0")
        )
        self.FRONT_NAN_REVERSE_TIMEOUT_SEC = float(
            os.environ.get("FRONT_NAN_REVERSE_TIMEOUT_SEC", "5.0")
        )

        # Stage 2: find the visible front-board cluster and rotate until its
        # midpoint is aimed at the actuator centre.
        self.FRONT_CLUSTER_HALF_ANGLE_RAD = math.radians(
            float(os.environ.get("FRONT_CLUSTER_HALF_ANGLE_DEG", "45.0"))
        )
        self.FRONT_CLUSTER_MIN_POINTS = int(
            os.environ.get("FRONT_CLUSTER_MIN_POINTS", "3")
        )
        self.FRONT_CLUSTER_MAX_ANGLE_GAP_RAD = math.radians(
            float(os.environ.get("FRONT_CLUSTER_MAX_ANGLE_GAP_DEG", "3.0"))
        )
        self.FRONT_CLUSTER_MAX_RANGE_JUMP_M = float(
            os.environ.get("FRONT_CLUSTER_MAX_RANGE_JUMP_M", "0.10")
        )
        self.FRONT_CLUSTER_MIN_WIDTH_M = float(
            os.environ.get("FRONT_CLUSTER_MIN_WIDTH_M", "0.015")
        )
        self.FRONT_CLUSTER_MAX_WIDTH_M = float(
            os.environ.get("FRONT_CLUSTER_MAX_WIDTH_M", "0.50")
        )
        self.ACTUATOR_AIM_OFFSET_RAD = math.radians(
            float(os.environ.get("ACTUATOR_AIM_OFFSET_DEG", "0.0"))
        )
        self.FINE_ALIGN_SETTLE_DURATION_SEC = float(
            os.environ.get("FINE_ALIGN_SETTLE_DURATION_SEC", "0.25")
        )
        self.FINE_ALIGN_TOLERANCE_RAD = math.radians(
            float(os.environ.get("FINE_ALIGN_TOLERANCE_DEG", "5.0"))
        )
        # The window test is far less twitchy than the old 2.5 deg test, so
        # three consecutive agreeing scans are enough confirmation.
        self.FRONT_ALIGN_CONFIRM_SCANS = int(
            os.environ.get("FRONT_ALIGN_CONFIRM_SCANS", "3")
        )
        self.FINE_ALIGN_KP = float(
            os.environ.get("FINE_ALIGN_KP", "1.8")
        )
        self.FINE_ALIGN_MIN_SPEED = float(
            os.environ.get("FINE_ALIGN_MIN_SPEED", "0.045")
        )
        self.FINE_ALIGN_MAX_SPEED = float(
            os.environ.get("FINE_ALIGN_MAX_SPEED", "0.18")
        )
        self.FINE_ALIGN_TIMEOUT_SEC = float(
            os.environ.get("FINE_ALIGN_TIMEOUT_SEC", "8.0")
        )

        # ---------------------------------------------------------------------
        # AIM TOLERANCE WINDOW  ("near the centre is good enough")
        # ---------------------------------------------------------------------
        #
        # The old code demanded that the board's midpoint bearing sat within
        # FINE_ALIGN_TOLERANCE_DEG (2.5 deg) of the pusher axis, and it kept
        # demanding that all the way in to the board. Two problems:
        #
        #   1. An angle is not what matters. What matters is WHERE ON THE
        #      BOARD the pusher lands. The same 2.5 deg is 2.2 cm of board at
        #      0.50 m but only 0.9 cm at 0.20 m, so the test silently became
        #      about three times stricter as the robot closed in -- exactly
        #      when LiDAR bearing noise is worst.
        #   2. A board is typically 10-20 cm wide. Landing 3 cm off centre
        #      topples it just as well as landing dead centre (arguably
        #      better: off-centre gives more toppling torque).
        #
        # So alignment is now judged as a LATERAL MISS DISTANCE on the board
        # face:
        #
        #        board face  |<--------- width_m --------->|
        #                    |         .    ^    .         |
        #        aim window  |     |<--- hit window --->|  |
        #                    |     .    (centre)    .      |
        #        margin      |<-->|                  |<--->|   BOARD_HIT_MARGIN_M
        #
        #   hit window = half the measured board width, minus a safety margin
        #                at each edge, then clamped to [MIN, MAX].
        #
        # miss = board_distance * sin(aim_error). Anything inside the window
        # is accepted immediately. Set ALIGN_MODE=centre to restore the old
        # strict centre-seeking behaviour for comparison.
        self.ALIGN_MODE = os.environ.get(
            "ALIGN_MODE", "window"
        ).strip().lower()
        # Keep-off distance from each board edge, so a window hit can never
        # slide off the side of the board.
        self.BOARD_HIT_MARGIN_M = float(
            os.environ.get("BOARD_HIT_MARGIN_M", "0.030")
        )
        # Never accept a window narrower than this (protects against a board
        # that LiDAR only saw a sliver of) ...
        self.BOARD_HIT_WINDOW_MIN_M = float(
            os.environ.get("BOARD_HIT_WINDOW_MIN_M", "0.015")
        )
        # ... and never trust a window wider than this, even for a wide board.
        self.BOARD_HIT_WINDOW_MAX_M = float(
            os.environ.get("BOARD_HIT_WINDOW_MAX_M", "0.060")
        )
        # To ENTER the window the robot aims at this fraction of it; to STAY
        # accepted it may drift out to the full window. Plain hysteresis, so
        # a borderline reading cannot make the robot twitch left-right.
        self.ALIGN_ENTER_FRACTION = float(
            os.environ.get("ALIGN_ENTER_FRACTION", "0.60")
        )
        # Hard sanity cap. However generous the window is in centimetres, a
        # bearing this far off means the cluster is not what we think it is.
        self.FINE_ALIGN_MAX_ANGLE_RAD = math.radians(
            float(os.environ.get("FINE_ALIGN_MAX_ANGLE_DEG", "15.0"))
        )
        self.APPROACH_MAX_ANGLE_RAD = math.radians(
            float(os.environ.get("APPROACH_MAX_ANGLE_DEG", "25.0"))
        )
        # While driving in, stop translating and rotate on the spot only when
        # the miss is this many windows wide. Small misses are corrected on
        # the move instead of stopping and starting.
        self.APPROACH_ROTATE_ONLY_FACTOR = float(
            os.environ.get("APPROACH_ROTATE_ONLY_FACTOR", "2.0")
        )
        # Last-resort valve: if the robot is already at the braking distance
        # but keeps failing the window test for this many scans, accept any
        # aim that still lands ON the board rather than turning for ever and
        # timing out with the board untouched.
        self.APPROACH_AIM_MAX_SCANS = int(
            os.environ.get("APPROACH_AIM_MAX_SCANS", "12")
        )
        # Nothing may be closer than this before the robot must stop, whatever
        # the aim looks like. Protects the chassis when the board is seen at
        # a steep angle.
        self.FRONT_SAFETY_STOP_M = float(
            os.environ.get("FRONT_SAFETY_STOP_M", "0.17")
        )

        # Approach retains the friend's 0.12 m/s far speed, but slows near the
        # board and uses cluster-bearing correction instead of only sector
        # balance. The ramming/tackle state remains unchanged.
        self.APPROACH_NEAR_DISTANCE_M = float(
            os.environ.get("APPROACH_NEAR_DISTANCE_M", "0.40")
        )
        self.APPROACH_NEAR_SPEED = float(
            os.environ.get("APPROACH_NEAR_SPEED", "0.08")
        )
        # LEGACY (ALIGN_MODE=centre only). The live approach judges aim by
        # lateral miss distance on the board face, not by a fixed angle.
        self.APPROACH_AIM_TOLERANCE_RAD = math.radians(
            float(os.environ.get("APPROACH_AIM_TOLERANCE_DEG", "4.0"))
        )
        self.APPROACH_ROTATE_ONLY_RAD = math.radians(
            float(os.environ.get("APPROACH_ROTATE_ONLY_DEG", "9.0"))
        )
        self.APPROACH_BALANCE_KP = float(
            os.environ.get("APPROACH_BALANCE_KP", "1.8")
        )
        self.APPROACH_MAX_ANGULAR_SPEED = float(
            os.environ.get("APPROACH_MAX_ANGULAR_SPEED", "0.20")
        )
        self.APPROACH_TIMEOUT_SEC = float(
            os.environ.get("APPROACH_TIMEOUT_SEC", "10.0")
        )
        self.APPROACH_MAX_DISTANCE_M = float(
            os.environ.get("APPROACH_MAX_DISTANCE_M", "0.75")
        )

        # Faster and more decisive post-obstacle line recovery. Barcode
        # decoding is also allowed during this state.
        self.OBSTACLE_REACQUIRE_CONFIRM_FRAMES = int(
            os.environ.get("OBSTACLE_REACQUIRE_CONFIRM_FRAMES", "2")
        )
        self.OBSTACLE_REACQUIRE_SPEED = float(
            os.environ.get("OBSTACLE_REACQUIRE_SPEED", "0.13")
        )
        self.OBSTACLE_REACQUIRE_LARGE_ERROR_SPEED = float(
            os.environ.get("OBSTACLE_REACQUIRE_LARGE_ERROR_SPEED", "0.10")
        )
        self.OBSTACLE_REACQUIRE_MAX_ANGULAR_SPEED = float(
            os.environ.get("OBSTACLE_REACQUIRE_MAX_ANGULAR_SPEED", "0.70")
        )
        self.OBSTACLE_REACQUIRE_SEARCH_SPEED = float(
            os.environ.get("OBSTACLE_REACQUIRE_SEARCH_SPEED", "0.35")
        )

        # Kept for compatibility with the old fallback branches.
        self.FRONT_CENTER_THRESHOLD = self.FRONT_MAX_RANGE_M
        self.CENTERING_BALANCE_TOLERANCE = 0.060
        self.CENTERING_EXTRA_DURATION_SEC = 0.0

        # Compatibility aliases for the original fallback branches lower in
        # this file. The live obstacle routine is driven by /scan callbacks.
        self.MIN_RANGE = self.FRONT_MIN_RANGE_M
        self.MAX_RANGE = self.SIDE_MAX_RANGE_M

        # Standard RC-style actuator pulse values. Set ENABLE_ACTUATOR=0
        # for a safe motion-only test without driving the actuator.
        self.ENABLE_ACTUATOR = bool(int(os.environ.get("ENABLE_ACTUATOR", "1")))
        self.ACTUATOR_EXTEND_PULSE_MS = float(
            os.environ.get("ACTUATOR_EXTEND_PULSE_MS", "2.0")
        )
        self.ACTUATOR_RETRACT_PULSE_MS = float(
            os.environ.get("ACTUATOR_RETRACT_PULSE_MS", "1.0")
        )

        # ---------------------------------------------------------------------
        # State variables
        # ---------------------------------------------------------------------

        # Live values that change while the robot runs (as opposed to the
        # CONSTANTS above, which are written in CAPITALS and never change).

        self.current_state = RobotState.LINE_FOLLOW
        self.pending_command: Optional[str] = None
        # Commands read while `pending_command` is still waiting for its
        # junction. Without this, a barcode seen a few centimetres after
        # another one was simply discarded.
        self.queued_commands: Deque[str] = deque()
        self.last_command_source_code: Optional[int] = None

        self.line_lost_count = 0
        self.reacquire_count = 0
        self.reverse_line_seen_count = 0
        self.intersection_stable_count = 0
        self.last_intersection_time = 0.0
        self.last_good_line_x: Optional[float] = None

        self.barcode_vote_codes: List[int] = []
        self.barcode_vote_start_time = 0.0
        self.last_barcode_accept_time = 0.0
        self.barcode_clear_count = 0
        self.barcode_latched_code: Optional[int] = None
        self.barcode_ignore_until = 0.0
        # Where the robot was when the current latch was set, so the latch can
        # also clear on distance travelled rather than only on empty frames.
        self.barcode_latch_x = 0.0
        self.barcode_latch_y = 0.0
        self.barcode_latch_pose_valid = False
        # Blanking window that applies to Barcode 2 alone.
        self.barcode2_ignore_until = 0.0
        # True while the robot is still inside the just-used Barcode 2 zone,
        # where a non-2 code must win a unanimous vote to be trusted.
        self.barcode_strict_vote_active = False

        # Prevent the already-executed physical Barcode 2 from triggering
        # again while the robot returns to, and departs from, its start point.
        self.barcode2_rearm_required = False
        self.barcode2_clear_frames = 0

        # ---------------------------------------------------------------------
        # Odometry variables
        # ---------------------------------------------------------------------

        # The robot's own estimate of where it is, taken from the wheel
        # encoders. Distances in metres, angles in radians. It drifts over
        # time, which is why it is only ever used for SHORT measurements
        # (one turn, one approach) and never as a map of the whole track.

        self.odom_ready = False
        self.last_odom_time = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.move_start_x = 0.0
        self.move_start_y = 0.0
        self.move_start_time = 0.0

        self.turn_start_yaw = 0.0
        self.turn_start_time = 0.0
        self.turn_direction: Optional[str] = None

        self.obstacle_start_x = 0.0
        self.obstacle_start_y = 0.0
        self.obstacle_start_yaw = 0.0
        self.obstacle_facing_yaw = 0.0

        # Measured-distance return tracking.
        self.approach_start_x = 0.0
        self.approach_start_y = 0.0
        self.reverse_start_x = 0.0
        self.reverse_start_y = 0.0
        self.required_reverse_distance = 0.0

        self.centering_extra_start_time = 0.0

        # Five-scan side confirmation and one stationary rescan fallback.
        self.side_scan_attempt = 0
        self.initial_scan_settle_start_time = 0.0
        self.side_scan_votes: List[Optional[Tuple[str, float]]] = []
        self.nan_side_votes: Deque[Optional[str]] = deque(
            maxlen=self.NAN_VOTE_WINDOW
        )
        self.left_clearance_samples: List[float] = []
        self.right_clearance_samples: List[float] = []
        self.left_nan_ratio_samples: List[float] = []
        self.right_nan_ratio_samples: List[float] = []
        self.rear_distance_samples: List[float] = []
        self.rear_nan_ratio_samples: List[float] = []
        self.opposite_side_clearance_m = float("inf")
        self.opposite_side_nan_ratio = 0.0
        self.rear_self_distance_baseline_m = float("inf")
        self.rear_self_nan_baseline = 0.0
        self.left_nan_ratio = 0.0
        self.right_nan_ratio = 0.0
        self.front_nan_ratio = 0.0
        self.rear_nan_ratio = 0.0
        self.front_nan_stable_count = 0
        self.front_nan_reverse_attempted = False
        self.front_nan_reverse_used = False
        self.front_nan_reverse_start_x = 0.0
        self.front_nan_reverse_start_y = 0.0
        self.front_nan_reverse_heading_yaw = 0.0
        self.front_nan_reverse_start_time = 0.0
        self.close_board_indicator_active = False
        self.rescan_settle_start_time = 0.0
        self.front_align_stable_count = 0
        self.front_lost_count = 0
        self.obstacle_turn_target_yaw = 0.0
        self.obstacle_turn_start_time = 0.0
        self.fine_align_settle_start_time = 0.0
        self.fine_align_start_time = 0.0
        self.approach_start_time = 0.0
        self.approach_aim_retry_count = 0
        self.front_board_target: Optional[LidarBoardTarget] = None

        # ---------------------------------------------------------------------
        # LiDAR values
        # ---------------------------------------------------------------------

        # Latest distances taken from /scan, in metres. Two special values
        # matter: inf means 'nothing found out to max range' (open space),
        # and NaN means 'something is there but too close to measure'.
        # Mixing those two up is the single easiest mistake to make here.

        self.scan_ready = False
        self.scan_message_count = 0
        self.last_scan_time = 0.0
        self.first_scan_logged = False

        # Independent watchdog state. The paused FSM state itself is preserved;
        # this flag forces every publisher path to send zero until sensor
        # health has been stable for several watchdog cycles.
        self.sensor_pause_active = False
        self.sensor_pause_start_time = 0.0
        self.sensor_pause_state: Optional[str] = None
        self.sensor_pause_missing: Tuple[str, ...] = tuple()
        self.sensor_healthy_count = 0
        self.sensor_recovery_count = 0
        self.fail_stop_enter_time = 0.0
        self.fail_stop_blink_state = False
        self.last_fail_stop_blink_time = 0.0
        self.last_fail_stop_buzzer_time = 0.0

        # LD08 is mounted 180 degrees opposite the robot's forward direction.
        # Therefore robot-front corresponds to LiDAR angle +pi.
        # Robot-left and robot-right are also reversed in the LiDAR frame.
        # ---------------------------------------------------------------------
        # How this LiDAR reports a target that is TOO CLOSE to measure
        #
        # This matters a lot: the FAQ places the toppling board at >= 115 mm
        # from the track, which is INSIDE the LDS-02 minimum range of 160 mm.
        # So the close-board detector is the NORMAL path, not a fallback --
        # and it only works if we know what the driver publishes for a
        # too-close return. Different builds publish NaN, 0.0, or a garbage
        # sub-minimum value.
        #
        # Run once with LIDAR_DIAGNOSTIC=1 and the board placed ~115 mm from
        # one side of the robot; the startup report tells you which it is.
        #
        #   BLIND_VALUE_MODE=nan   count only NaN            (original code)
        #   BLIND_VALUE_MODE=zero  count only 0.0 / near-0
        #   BLIND_VALUE_MODE=both  count NaN, 0.0 and sub-minimum  (safest
        #                          once you have confirmed with the report)
        self.BLIND_VALUE_MODE = os.environ.get(
            "BLIND_VALUE_MODE", "nan"
        ).strip().lower()
        self.BLIND_ZERO_EPSILON_M = float(
            os.environ.get("BLIND_ZERO_EPSILON_M", "0.01")
        )
        self.LIDAR_DIAGNOSTIC = bool(
            int(os.environ.get("LIDAR_DIAGNOSTIC", "0"))
        )
        self.lidar_diagnostic_done = False

        self.LIDAR_FRONT_ANGLE_RAD = math.pi
        self.LIDAR_REAR_ANGLE_RAD = 0.0
        self.LIDAR_LEFT_ANGLE_RAD = -math.pi / 2.0
        self.LIDAR_RIGHT_ANGLE_RAD = math.pi / 2.0

        self.left_side_dist = float("inf")
        self.right_side_dist = float("inf")
        self.front_left_dist = float("inf")
        self.front_right_dist = float("inf")
        self.front_center_dist = float("inf")
        self.front_dist = float("inf")
        self.rear_dist = float("inf")

        # ---------------------------------------------------------------------
        # Obstacle timing
        # ---------------------------------------------------------------------

        # Stopwatches for the board routine. Every phase is given a deadline;
        # if it is not finished in time the robot fail-stops rather than
        # keep pushing at something it cannot see properly.

        self.obstacle_brake_start_time = 0.0
        self.actuator_extend_start_time = 0.0
        self.tackle_start_time = 0.0
        self.tackle_start_x = 0.0
        self.tackle_start_y = 0.0
        self.tackle_board_down_stable_count = 0
        self.tackle_contact_hold_start_time = 0.0
        self.tackle_stop_reason = ""
        self.return_start_time = 0.0
        self.tackle_required_push_m = 0.0
        self.tackle_board_distance_m = float("inf")
        self.tackle_last_push_distance = 0.0
        self.tackle_stall_count = 0
        self.tackle_contact_detected = False

        # ---------------------------------------------------------------------
        # Status logging
        # ---------------------------------------------------------------------

        # How often the one-line status update is printed. Printing every
        # control cycle (20 Hz) would scroll far too fast to read.

        self.last_status_log_time = 0.0
        self.node_start_time = time.time()
        self.STATUS_LOG_INTERVAL = float(
            os.environ.get("STATUS_LOG_INTERVAL", "1.00")
        )

        # ---------------------------------------------------------------------
        # Pixy buffers and initialization
        # ---------------------------------------------------------------------

        # Pre-allocated arrays the Pixy library fills in on every read. They
        # are made once here rather than every frame, because allocating them
        # 20 times a second would waste time in the control loop.

        self.vectors = VectorArray(100) if PIXY_AVAILABLE else None
        self.barcodes = BarcodeArray(10) if PIXY_AVAILABLE else None
        self.intersections = IntersectionArray(100) if PIXY_AVAILABLE else None

        self.pixy_ready = False

        # Pixy2 illumination can help in dim areas. If glossy tape causes glare,
        # try PIXY_LAMP_LOWER=0 or switch both lamps off for comparison.
        self.PIXY_LAMP_UPPER = int(
            os.environ.get("PIXY_LAMP_UPPER", "1")
        )
        self.PIXY_LAMP_LOWER = int(
            os.environ.get("PIXY_LAMP_LOWER", "1")
        )

        if PIXY_AVAILABLE:
            try:
                result = pixy.init()

                if result < 0:
                    self.get_logger().error(
                        "Cannot connect to Pixy2. Check its USB cable and power."
                    )
                    self.current_state = RobotState.LINE_LOST_ALARM
                    self.set_stop_led(True)

                else:
                    pixy.change_prog("line")
                    time.sleep(0.5)

                    self.pixy_ready = True

                    # Only call setLamp if this Pixy Python binding supports it.
                    if hasattr(pixy, "set_lamp"):
                        pixy.set_lamp(
                            self.PIXY_LAMP_UPPER,
                            self.PIXY_LAMP_LOWER,
                        )
                        self.get_logger().info(
                            "Pixy2 lamp setting: "
                            f"upper={self.PIXY_LAMP_UPPER}, "
                            f"lower={self.PIXY_LAMP_LOWER}."
                        )
                    else:
                        self.get_logger().warning(
                            "Pixy Python binding does not support set_lamp()."
                        )

                    # This controls your external GPIO LEDs, not the Pixy LED.
                    self.all_leds_off()

                    self.get_logger().info(
                        "Pixy2 connected. Combined navigation controller started."
                    )

            except Exception as exc:
                self.get_logger().error(f"Pixy2 initialization failed: {exc}")
                self.current_state = RobotState.LINE_LOST_ALARM
                self.set_stop_led(True)

        else:
            self.get_logger().error("Pixy Python module unavailable.")
            self.current_state = RobotState.LINE_LOST_ALARM
            self.set_stop_led(True)

        self.CONTROL_PERIOD_SEC = float(
            os.environ.get("CONTROL_PERIOD_SEC", "0.05")
        )
        self.timer = self.create_timer(
            self.CONTROL_PERIOD_SEC,
            self.control_loop,
            callback_group=self.control_group,
        )
        self.watchdog_timer = self.create_timer(
            self.WATCHDOG_PERIOD_SEC,
            self.watchdog_callback,
            callback_group=self.watchdog_group,
        )
        self.reset_fail_stop_service = self.create_service(
            Trigger,
            "~/reset_fail_stop",
            self.reset_fail_stop_callback,
            callback_group=self.watchdog_group,
        )
        # In friendly mode, silence rclpy's INFO stream so the clean console
        # output below is the only thing on screen. WARN and ERROR still show.
        if not self.VERBOSE:
            try:
                rclpy.logging.set_logger_level(
                    self.get_logger().name,
                    rclpy.logging.LoggingSeverity.WARN,
                )
            except Exception:
                pass

        self.print_banner()
        self.get_logger().debug(
            f"period={self.CONTROL_PERIOD_SEC:.3f}s, "
            f"forward={self.FORWARD_SPEED:.3f}m/s, "
            f"armed={self.ARMED_FORWARD_SPEED:.3f}m/s, "
            f"barcode_vote={self.BARCODE_MIN_VOTES}/"
            f"{self.BARCODE_VOTE_SAMPLES}, "
            f"watchdog={self.SENSOR_STALE_TIMEOUT_SEC:.2f}s, "
            f"actuator={'ON' if self.ENABLE_ACTUATOR else 'DRY RUN'}."
        )

    # =========================================================================
    # CONSOLE OUTPUT
    #
    # say()    -> a milestone the operator should see. Always shown.
    # detail() -> per-scan numbers. Shown only when CONSOLE_STYLE=debug.
    # warn()   -> something went wrong. Always shown, in yellow.
    # =========================================================================

    def colour(self, text: str, code: str) -> str:
        """Wrap `text` in an ANSI colour code so the terminal prints it in colour.

        `code` is an ANSI style string, e.g. "1;91" = bold red.
        If USE_COLOUR is False (NO_COLOUR=1, or output is piped to a file)
        the text is returned unchanged so the log stays readable.
        """
        if not self.USE_COLOUR:
            return text
        return f"\033[{code}m{text}\033[0m"

    def _throttled(self, key: str, seconds: float) -> bool:
        """Return True if this key has not printed within `seconds`."""
        if seconds <= 0.0:
            return True
        now = time.time()
        last = self._throttle_times.get(key, 0.0)
        if now - last < seconds:
            return False
        self._throttle_times[key] = now
        return True

    def say(
        self,
        message: str,
        throttle: float = 0.0,
        throttle_duration_sec: Optional[float] = None,
    ) -> None:
        """Print an operator-facing milestone."""
        if throttle_duration_sec is not None:
            throttle = throttle_duration_sec
        if throttle > 0.0 and not self._throttled(message[:24], throttle):
            return
        print(
            f"  {self.colour('>', '1;92')} {message}",
            flush=True,
        )
        self.get_logger().debug(message)

    def detail(
        self,
        message: str,
        throttle: float = 0.0,
        throttle_duration_sec: Optional[float] = None,
    ) -> None:
        """Print per-scan diagnostics only in debug console style."""
        if not self.VERBOSE:
            return
        if throttle_duration_sec is not None:
            throttle = throttle_duration_sec
        if throttle > 0.0 and not self._throttled(message[:24], throttle):
            return
        print(
            f"    {self.colour('.', '2;37')} {message}",
            flush=True,
        )

    def warn(
        self,
        message: str,
        throttle: float = 0.0,
        throttle_duration_sec: Optional[float] = None,
    ) -> None:
        """Print a yellow '!' warning line. Always shown, even in friendly mode."""
        if throttle_duration_sec is not None:
            throttle = throttle_duration_sec
        if throttle > 0.0 and not self._throttled(message[:24], throttle):
            return
        print(
            f"  {self.colour('!', '1;93')} {message}",
            flush=True,
        )
        self.get_logger().debug(message)

    def print_banner(self) -> None:
        """Startup card so a beginner knows what is running and what to press."""
        width = 68
        line = "-" * width
        title = "TurtleBot3 line-following + barcode + board-toppling"
        print(
            "\n"
            + self.colour(line, "1;96")
            + "\n  "
            + self.colour(title, "1;97")
            + "\n"
            + self.colour(line, "1;96")
            + f"\n  Console style : {self.CONSOLE_STYLE}"
            f"  (set CONSOLE_STYLE=debug for full numbers)"
            f"\n  Actuator      : "
            f"{'ENABLED' if self.ENABLE_ACTUATOR else 'DRY RUN (ENABLE_ACTUATOR=0)'}"
            f"\n  Front geometry: LiDAR centre -> retracted pusher "
            f"{self.LIDAR_TO_FRONT_EDGE_M:.3f} m, "
            f"pusher reach {self.ACTUATOR_REACH_M:.3f} m"
            f"\n  Rear geometry : LiDAR centre -> back of LED plate "
            f"{self.REAR_OVERHANG_M:.3f} m"
            f"\n  Brake at      : {self.STOP_DISTANCE_M:.3f} m from the board"
            f"\n  Aim rule      : "
            + (
                "window - land within "
                f"+/-{self.BOARD_HIT_WINDOW_MIN_M * 100.0:.1f}"
                f"-{self.BOARD_HIT_WINDOW_MAX_M * 100.0:.1f} cm of the "
                "board centre"
                if self.ALIGN_MODE != "centre"
                else "centre - old strict midpoint seeking (ALIGN_MODE=centre)"
            )
            + "\n  Barcodes      : 0=LEFT  1=RIGHT  2=OBSTACLE  3=STOP"
            "\n  Stop anytime  : Ctrl-C in this terminal"
            "\n  If it fail-stops:"
            "\n                  run 'ros2 run turtlebot3_teleop "
            "teleop_keyboard'"
            "\n                  in TERMINAL 2 and drive with W A S D X"
            "\n                  (S or SPACE = stop; arrow keys do nothing)"
            "\n"
            + self.colour(line, "1;96")
            + "\n",
            flush=True,
        )
        if self.ACTUATOR_REACH_M <= 0.0:
            self.warn(
                "ACTUATOR_REACH_M is 0 -- the robot will push the board "
                "with its bumper. Measure the extended pusher and set it."
            )
        elif self.ACTUATOR_REACH_M <= self.TOPPLE_OVERTRAVEL_M:
            self.warn(
                f"ACTUATOR_REACH_M ({self.ACTUATOR_REACH_M:.3f} m) is not "
                f"bigger than TOPPLE_OVERTRAVEL_M "
                f"({self.TOPPLE_OVERTRAVEL_M:.3f} m), so the chassis will "
                "also reach the board. Lower TOPPLE_OVERTRAVEL_M if you "
                "want the pusher to do all the work."
            )

    # =========================================================================
    # ODOMETRY
    # =========================================================================

    def odom_callback(self, msg: Odometry) -> None:
        """Runs every time /odom arrives: store where the robot is and stamp the time.

        Odometry gives us x, y and yaw (heading). The quaternion from ROS is
        converted to a single yaw angle because the robot only drives on a
        flat floor. Messages containing NaN/inf are thrown away so a bad
        reading can never poison a distance or turn measurement.
        """
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if not all(math.isfinite(value) for value in (x, y, yaw)):
            self.get_logger().warning(
                "Rejected malformed odometry containing NaN or infinity.",
                throttle_duration_sec=1.0,
            )
            return

        with self.state_lock:
            self.current_x = x
            self.current_y = y
            self.current_yaw = yaw
            self.last_odom_time = time.time()
            self.odom_ready = True

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Convert a ROS orientation quaternion into a flat-floor heading (radians).

        Only the rotation about the vertical Z axis matters for a TurtleBot3,
        so the full 3D quaternion collapses to one angle.
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Fold any angle back into the range -pi .. +pi.

        Needed because 350 degrees and -10 degrees are the same heading. Without
        wrapping, a turn crossing the +/-180 degree line looks like a huge error.
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def start_distance_move(self) -> None:
        """Remember the current position as the start of a 'drive this far' move."""
        self.move_start_x = self.current_x
        self.move_start_y = self.current_y
        self.move_start_time = time.time()
        self.sensor_recovery_count = 0

    def distance_from_move_start(self) -> float:
        """Straight-line distance (metres) travelled since start_distance_move()."""
        return math.hypot(
            self.current_x - self.move_start_x,
            self.current_y - self.move_start_y,
        )

    def start_odom_turn(self, direction: str) -> None:
        """Remember the current heading as the start of a LEFT or RIGHT turn."""
        self.turn_direction = direction
        self.turn_start_yaw = self.current_yaw
        self.turn_start_time = time.time()

    def turned_angle_abs(self) -> float:
        """How far the robot has turned (radians) since start_odom_turn().

        Returns a positive number for the expected direction only, so a small
        wobble the wrong way does not falsely count as progress.
        """
        delta = self.wrap_angle(self.current_yaw - self.turn_start_yaw)

        if self.turn_direction == "LEFT":
            return max(0.0, delta)

        if self.turn_direction == "RIGHT":
            return max(0.0, -delta)

        return abs(delta)

    def distance_from_obstacle_start(self) -> float:
        """Straight-line distance (metres) from the pose saved when barcode 2 fired."""
        return math.hypot(
            self.current_x - self.obstacle_start_x,
            self.current_y - self.obstacle_start_y,
        )

    def signed_obstacle_displacement(self) -> float:
        """Forward displacement from the saved start along the board-facing heading."""
        dx = self.current_x - self.obstacle_start_x
        dy = self.current_y - self.obstacle_start_y
        return (
            dx * math.cos(self.obstacle_facing_yaw)
            + dy * math.sin(self.obstacle_facing_yaw)
        )

    # =========================================================================
    # LIDAR
    # =========================================================================

    def run_lidar_diagnostic(self, msg: LaserScan) -> None:
        """Print exactly what this LiDAR publishes, once, then never again.

        Run with LIDAR_DIAGNOSTIC=1 and the toppling board placed about
        115 mm from ONE side of the robot. The report answers two questions
        that the rest of the obstacle routine depends on:

          1. What does the driver publish for a target too close to measure?
             -> set BLIND_VALUE_MODE to match.
          2. Is the front sector really the front?
             -> FRONT should read the open corridor, REAR should read the
                robot's own LED mast at about 0.14 m.
        """
        self.lidar_diagnostic_done = True

        total = len(msg.ranges)
        nan_n = sum(1 for r in msg.ranges if math.isnan(float(r)))
        inf_n = sum(1 for r in msg.ranges if math.isinf(float(r)))
        zero_n = sum(
            1
            for r in msg.ranges
            if not math.isnan(float(r))
            and not math.isinf(float(r))
            and float(r) <= self.BLIND_ZERO_EPSILON_M
        )
        finite = [
            float(r)
            for r in msg.ranges
            if not math.isnan(float(r))
            and not math.isinf(float(r))
            and float(r) > self.BLIND_ZERO_EPSILON_M
        ]

        sectors = (
            ("FRONT", self.LIDAR_FRONT_ANGLE_RAD),
            ("REAR ", self.LIDAR_REAR_ANGLE_RAD),
            ("LEFT ", self.LIDAR_LEFT_ANGLE_RAD),
            ("RIGHT", self.LIDAR_RIGHT_ANGLE_RAD),
        )

        width = 68
        print(
            "\n"
            + self.colour("-" * width, "1;96")
            + "\n  "
            + self.colour("LIDAR SELF-REPORT (one time)", "1;97")
            + "\n"
            + self.colour("-" * width, "1;96")
            + f"\n  samples={total}  range_min={msg.range_min:.3f} "
            f"range_max={msg.range_max:.3f}"
            f"\n  NaN={nan_n}  zero/near-zero={zero_n}  inf={inf_n}  "
            f"finite={len(finite)}",
            flush=True,
        )
        if finite:
            print(
                f"  finite range span: {min(finite):.3f} .. {max(finite):.3f} m",
                flush=True,
            )

        print("\n  Per-sector (+/- 20 deg):", flush=True)
        for name, angle in sectors:
            distance = self.get_sector_min_distance(msg, angle, -20.0, 20.0)
            blind = self.get_sector_nan_ratio(msg, angle, -20.0, 20.0)
            shown = "none" if math.isinf(distance) else f"{distance:.3f} m"
            print(
                f"    {name}  nearest={shown:>9}   blind_ratio={blind:.2f}",
                flush=True,
            )

        print(
            f"\n  BLIND_VALUE_MODE is currently "
            f"{self.colour(self.BLIND_VALUE_MODE, '1;96')}."
            "\n  If the board is next to the robot but that side shows"
            "\n  nearest=none AND blind_ratio=0.00, this mode is wrong:"
            "\n    many NaN  -> BLIND_VALUE_MODE=nan"
            "\n    many zero -> BLIND_VALUE_MODE=zero"
            "\n    unsure    -> BLIND_VALUE_MODE=both"
            "\n\n  REAR should show roughly 0.14 m (the robot's own LED"
            "\n  mast). If FRONT shows that instead, the front/rear angle"
            "\n  constants are swapped for your LiDAR mounting.\n"
            + self.colour("-" * width, "1;96")
            + "\n",
            flush=True,
        )

    def scan_callback(self, msg: LaserScan) -> None:
        """Store LiDAR sectors without allowing one malformed scan to kill updates."""
        self.scan_message_count += 1

        # Mark reception immediately for diagnostics. The freshness timestamp
        # is updated only after all required sectors are processed successfully.
        self.scan_ready = True

        if not self.first_scan_logged:
            self.first_scan_logged = True
            self.get_logger().info(
                f"First /scan received: ranges={len(msg.ranges)}, "
                f"angle_min={msg.angle_min:.3f}, angle_max={msg.angle_max:.3f}, "
                f"increment={msg.angle_increment:.6f}"
            )

        if not msg.ranges:
            self.scan_ready = False
            self.get_logger().warning("Received an empty /scan message.")
            return

        try:
            if self.LIDAR_DIAGNOSTIC and not self.lidar_diagnostic_done:
                self.run_lidar_diagnostic(msg)

            # Use wider, symmetric sectors and a filtered nearest-point
            # median instead of one raw minimum reading.
            self.left_side_dist = self.get_sector_min_distance(
                msg, self.LIDAR_LEFT_ANGLE_RAD, -40.0, 40.0
            )
            self.right_side_dist = self.get_sector_min_distance(
                msg, self.LIDAR_RIGHT_ANGLE_RAD, -40.0, 40.0
            )
            self.left_nan_ratio = self.get_sector_nan_ratio(
                msg, self.LIDAR_LEFT_ANGLE_RAD, -40.0, 40.0
            )
            self.right_nan_ratio = self.get_sector_nan_ratio(
                msg, self.LIDAR_RIGHT_ANGLE_RAD, -40.0, 40.0
            )
            self.front_left_dist = self.get_sector_min_distance(
                msg, self.LIDAR_FRONT_ANGLE_RAD, 5.0, 25.0
            )
            self.front_right_dist = self.get_sector_min_distance(
                msg, self.LIDAR_FRONT_ANGLE_RAD, -25.0, -5.0
            )
            self.front_center_dist = self.get_sector_min_distance(
                msg, self.LIDAR_FRONT_ANGLE_RAD, -5.0, 5.0
            )
            self.front_nan_ratio = self.get_sector_nan_ratio(
                msg,
                self.LIDAR_FRONT_ANGLE_RAD,
                -self.FRONT_NAN_HALF_ANGLE_DEG,
                self.FRONT_NAN_HALF_ANGLE_DEG,
            )
            self.rear_dist = self.get_sector_min_distance(
                msg, self.LIDAR_REAR_ANGLE_RAD, -20.0, 20.0
            )
            self.rear_nan_ratio = self.get_sector_nan_ratio(
                msg, self.LIDAR_REAR_ANGLE_RAD, -20.0, 20.0
            )
            self.front_dist = min(
                self.front_left_dist,
                self.front_center_dist,
                self.front_right_dist,
            )
            self.front_board_target = self.get_front_board_target(msg)
            self.last_scan_time = time.time()
        except Exception as exc:
            self.scan_ready = False
            self.get_logger().error(
                f"/scan callback processing failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=1.0,
            )
            return

        # During an obstacle routine, LiDAR messages directly drive the
        # obstacle state machine, matching the proven standalone routine.
        with self.state_lock:
            if self.current_state in self.obstacle_states():
                # The independent watchdog owns pause/recovery. Never advance
                # the FSM from fresh scans while odometry is stale or the
                # watchdog is deliberately collecting healthy recovery cycles.
                if (
                    self.sensor_pause_active
                    or not self.odom_is_fresh()
                ):
                    self.publish_motion(self.stop_twist(), alarm=True)
                    return
                self.process_obstacle_scan()

    def obstacle_states(self):
        """The set of states that belong to the board-toppling routine.

        Used by the watchdog and the logger to ask 'are we in the obstacle
        routine right now?' without listing every state again.
        """
        return {
            RobotState.OBSTACLE_SCAN_SETTLE,
            RobotState.OBSTACLE_SEARCH,
            RobotState.OBSTACLE_RESCAN_SETTLE,
            RobotState.OBSTACLE_TURN_LEFT,
            RobotState.OBSTACLE_TURN_RIGHT,
            RobotState.OBSTACLE_FINE_ALIGN_SETTLE,
            RobotState.OBSTACLE_FINE_ALIGN,
            RobotState.OBSTACLE_FRONT_NAN_REAR_CHECK,
            RobotState.OBSTACLE_FRONT_NAN_REVERSE,
            RobotState.OBSTACLE_CENTERING_EXTRA,
            RobotState.OBSTACLE_APPROACH,
            RobotState.OBSTACLE_BRAKE,
            RobotState.OBSTACLE_ACTUATOR_EXTEND,
            RobotState.OBSTACLE_TACKLE,
            RobotState.OBSTACLE_TACKLE_HOLD,
            RobotState.OBSTACLE_RETURN,
            RobotState.OBSTACLE_FRONT_NAN_RETURN_FORWARD,
            RobotState.OBSTACLE_RETURN_YAW,
        }

    def odom_is_fresh(self, now: Optional[float] = None) -> bool:
        """True if an /odom message arrived recently enough to be trusted."""
        check_time = time.time() if now is None else now
        return (
            self.odom_ready
            and self.last_odom_time > 0.0
            and check_time - self.last_odom_time
            <= self.SENSOR_STALE_TIMEOUT_SEC
        )

    def scan_is_fresh(self, now: Optional[float] = None) -> bool:
        """True if a /scan message arrived recently enough to be trusted."""
        check_time = time.time() if now is None else now
        return (
            self.scan_ready
            and self.last_scan_time > 0.0
            and check_time - self.last_scan_time
            <= self.SENSOR_STALE_TIMEOUT_SEC
        )

    def watchdog_requirements(self) -> Tuple[bool, bool]:
        """Return (needs_scan, needs_odom) for the active motion checkpoint."""
        obstacle_active = self.current_state in self.obstacle_states()
        normal_odom_motion = self.current_state in (
            RobotState.MOVE_TO_TURN_CENTER,
            RobotState.EXECUTE_TURN_ODOM,
        )
        return obstacle_active, obstacle_active or normal_odom_motion

    def adjust_timers_after_sensor_pause(self, paused_duration: float) -> None:
        """Exclude a stationary sensor pause from every active FSM timeout."""
        timer_names = (
            "move_start_time",
            "turn_start_time",
            "initial_scan_settle_start_time",
            "rescan_settle_start_time",
            "obstacle_turn_start_time",
            "fine_align_settle_start_time",
            "fine_align_start_time",
            "front_nan_reverse_start_time",
            "approach_start_time",
            "obstacle_brake_start_time",
            "actuator_extend_start_time",
            "tackle_start_time",
            "tackle_contact_hold_start_time",
            "return_start_time",
        )
        for name in timer_names:
            value = float(getattr(self, name, 0.0))
            if value > 0.0:
                setattr(self, name, value + paused_duration)

    def watchdog_callback(self) -> None:
        """Stop stale-sensor motion and resume only after stable fresh data."""
        now = time.time()

        with self.state_lock:
            if self.current_state == RobotState.OBSTACLE_FAIL_STOP:
                self.fail_stop_outputs()
                # Send zero for a bounded period, then release /cmd_vel so a
                # teleop node can reposition the robot without fighting this
                # controller's repeated zero commands.
                if (
                    now - self.fail_stop_enter_time
                    <= self.FAIL_STOP_COMMAND_HOLD_SEC
                ):
                    try:
                        if rclpy.ok():
                            self.cmd_vel_pub.publish(self.stop_twist())
                    except Exception:
                        pass
                return

            needs_scan, needs_odom = self.watchdog_requirements()
            if not (needs_scan or needs_odom):
                return

            missing: List[str] = []
            if needs_scan and not self.scan_is_fresh(now):
                missing.append("/scan")
                self.scan_ready = False
            if needs_odom and not self.odom_is_fresh(now):
                missing.append("/odom")
                # A historical odometry message is not "ready" for motion.
                # The next validated callback sets this true again.
                self.odom_ready = False

            if missing:
                self.sensor_healthy_count = 0
                self.sensor_pause_missing = tuple(missing)

                if not self.sensor_pause_active:
                    if (
                        self.sensor_recovery_count
                        >= self.SENSOR_MAX_RECOVERIES
                    ):
                        self.enter_fail_stop(
                            "Sensor data became stale again after "
                            f"{self.SENSOR_MAX_RECOVERIES} automatic "
                            "recoveries."
                        )
                        return

                    self.sensor_pause_active = True
                    self.sensor_pause_start_time = now
                    self.sensor_pause_state = self.current_state
                    self.sensor_recovery_count += 1
                    self.queue_buzzer_pattern(self.BEEP_RESCAN)
                    self.get_logger().error(
                        "Sensor watchdog paused motion in "
                        f"{self.current_state}: stale {', '.join(missing)}. "
                        "Waiting for stable fresh data."
                    )

                if (
                    now - self.sensor_pause_start_time
                    > self.SENSOR_RECOVERY_GRACE_SEC
                ):
                    self.enter_fail_stop(
                        "Sensor watchdog recovery timed out after "
                        f"{self.SENSOR_RECOVERY_GRACE_SEC:.1f} s; stale "
                        f"{', '.join(self.sensor_pause_missing)}."
                    )
                    return

                try:
                    if rclpy.ok():
                        self.cmd_vel_pub.publish(self.stop_twist())
                except Exception:
                    pass
                self.alarm_outputs()
                return

            if not self.sensor_pause_active:
                return

            if self.current_state != self.sensor_pause_state:
                self.enter_fail_stop(
                    "FSM state changed unexpectedly during a sensor pause "
                    f"({self.sensor_pause_state} -> {self.current_state})."
                )
                return

            self.sensor_healthy_count += 1
            try:
                if rclpy.ok():
                    self.cmd_vel_pub.publish(self.stop_twist())
            except Exception:
                pass

            if (
                self.sensor_healthy_count
                < self.SENSOR_RECOVERY_HEALTHY_CYCLES
            ):
                return

            paused_duration = now - self.sensor_pause_start_time
            resumed_state = self.sensor_pause_state
            self.adjust_timers_after_sensor_pause(paused_duration)
            self.sensor_pause_active = False
            self.sensor_pause_start_time = 0.0
            self.sensor_pause_state = None
            self.sensor_pause_missing = tuple()
            self.sensor_healthy_count = 0
            self.get_logger().warning(
                f"Sensor data stable again. Resuming {resumed_state} after "
                f"a {paused_duration:.2f} s stationary pause "
                f"(recovery {self.sensor_recovery_count}/"
                f"{self.SENSOR_MAX_RECOVERIES})."
            )

    def reset_fail_stop_callback(self, request, response):
        """ROS service handler that clears a latched fail-stop.

        Called when the operator runs RESET_FAIL_STOP_COMMAND in a terminal.
        Everything that could still be stale (queued barcodes, sensor-pause
        bookkeeping, PD memory) is wiped, then the robot is dropped into
        LINE_LOST_ALARM so it waits for Pixy to see the line again before it
        drives itself. It never resumes mid-manoeuvre.
        """
        del request
        with self.state_lock:
            if self.current_state != RobotState.OBSTACLE_FAIL_STOP:
                response.success = False
                response.message = "Controller is not in fail-stop."
                return response

            self.sensor_pause_active = False
            self.sensor_pause_start_time = 0.0
            self.sensor_pause_state = None
            self.sensor_pause_missing = tuple()
            self.sensor_healthy_count = 0
            self.sensor_recovery_count = 0
            self.fail_stop_enter_time = 0.0
            self.fail_stop_blink_state = False
            self.last_fail_stop_blink_time = 0.0
            self.last_fail_stop_buzzer_time = 0.0
            self.fail_stop_reason = ""
            self.last_fail_stop_reminder_time = 0.0
            self.buzzer_queue.clear()
            self.clear_command_queue()
            self.last_command_source_code = None
            self.turn_direction = None
            self.prev_error = 0.0
            self.line_lost_count = 0
            self.reacquire_count = 0
            self.reverse_line_seen_count = 0
            self.reset_barcode_vote()
            self.stop_actuator_signal()
            self.current_state = RobotState.LINE_LOST_ALARM
            self.publish_motion(self.stop_twist(), alarm=True)

            response.success = True
            response.message = (
                "Fail-stop reset. Place the robot on the line; autonomy will "
                "resume after Pixy confirms the line for consecutive frames."
            )
            print(
                "\n  "
                + self.colour(" FAIL-STOP CLEARED ", "1;30;102")
                + "\n    Buzzer and fast red flash are off."
                "\n    Put the robot on the line if you have not already."
                "\n    It will start driving again by itself once the Pixy"
                "\n    has seen the line for a few frames in a row.\n",
                flush=True,
            )
            self.get_logger().warning(response.message)
            return response

    def process_obstacle_scan(self) -> None:
        """Run the refined obstacle routine from each fresh LiDAR scan."""
        twist = self.stop_twist()
        now = time.time()
        difference = self.front_distance_difference()

        if self.current_state not in (
            RobotState.OBSTACLE_ACTUATOR_EXTEND,
            RobotState.OBSTACLE_TACKLE,
            RobotState.OBSTACLE_TACKLE_HOLD,
        ):
            self.set_pulse_ms(self.ACTUATOR_RETRACT_PULSE_MS)

        if self.current_state == RobotState.OBSTACLE_SCAN_SETTLE:
            if (
                now - self.initial_scan_settle_start_time
                >= self.INITIAL_SCAN_SETTLE_DURATION_SEC
            ):
                self.side_scan_votes.clear()
                self.nan_side_votes.clear()
                self.clear_clearance_histories()
                self.current_state = RobotState.OBSTACLE_SEARCH
                self.say(
                    f"Robot settled after Barcode 2. Starting "
                    f"{self.SIDE_SCAN_COUNT} stationary side scans with "
                    "normal-distance and repeated-NaN voting."
                )

        elif self.current_state == RobotState.OBSTACLE_SEARCH:
            vote = self.make_side_scan_vote()
            self.side_scan_votes.append(vote)
            nan_vote = self.make_nan_side_vote()
            self.nan_side_votes.append(nan_vote)
            if math.isfinite(self.left_side_dist):
                self.left_clearance_samples.append(self.left_side_dist)
            if math.isfinite(self.right_side_dist):
                self.right_clearance_samples.append(self.right_side_dist)
            self.left_nan_ratio_samples.append(self.left_nan_ratio)
            self.right_nan_ratio_samples.append(self.right_nan_ratio)
            if math.isfinite(self.rear_dist):
                self.rear_distance_samples.append(self.rear_dist)
            self.rear_nan_ratio_samples.append(self.rear_nan_ratio)

            scan_number = len(self.side_scan_votes)
            if vote is None:
                self.detail(
                    f"Obstacle side scan {scan_number}/{self.SIDE_SCAN_COUNT} "
                    f"(attempt {self.side_scan_attempt}/{self.SIDE_MAX_ATTEMPTS}): "
                    f"uncertain; left={self.left_side_dist:.2f}, "
                    f"right={self.right_side_dist:.2f}, "
                    f"nan(L/R)={self.left_nan_ratio:.2f}/"
                    f"{self.right_nan_ratio:.2f}, "
                    f"nan_vote={nan_vote}."
                )
            else:
                side, distance = vote
                self.detail(
                    f"Obstacle side scan {scan_number}/{self.SIDE_SCAN_COUNT} "
                    f"(attempt {self.side_scan_attempt}/{self.SIDE_MAX_ATTEMPTS}): "
                    f"nearest={side} at {distance:.2f} m; "
                    f"left={self.left_side_dist:.2f}, "
                    f"right={self.right_side_dist:.2f}, "
                    f"nan(L/R)={self.left_nan_ratio:.2f}/"
                    f"{self.right_nan_ratio:.2f}."
                )

            if len(self.side_scan_votes) >= self.SIDE_SCAN_COUNT:
                result = self.evaluate_side_scan_votes()
                nan_result = self.evaluate_nan_side_votes()

                # A repeated one-sided NaN consensus has priority because
                # that board is probably inside 0.16 m and is therefore
                # nearer than any board with a valid LDS-02 distance. A
                # single stray NaN cannot reach this branch.
                if nan_result is not None:
                    self.start_nan_inferred_obstacle_turn(nan_result)

                elif result is not None:
                    selected_side, selected_distance = result
                    self.start_normal_obstacle_turn(
                        selected_side,
                        selected_distance,
                    )

                elif self.side_scan_attempt < self.SIDE_MAX_ATTEMPTS:
                    self.rescan_settle_start_time = now
                    self.current_state = RobotState.OBSTACLE_RESCAN_SETTLE
                    self.queue_buzzer_pattern(self.BEEP_RESCAN)
                    self.warn(
                        "The first scan group had neither a reliable distance "
                        "nor a reliable one-sided NaN pattern. "
                        "Remaining stationary, settling briefly, then "
                        f"collecting {self.SIDE_SCAN_COUNT} new scans."
                    )
                else:
                    self.fail_obstacle_routine(
                        f"No reliable LEFT/RIGHT result after "
                        f"{self.SIDE_MAX_ATTEMPTS} groups of "
                        f"{self.SIDE_SCAN_COUNT} scans."
                    )

        elif self.current_state == RobotState.OBSTACLE_RESCAN_SETTLE:
            if now - self.rescan_settle_start_time >= self.RESCAN_SETTLE_DURATION_SEC:
                self.side_scan_attempt += 1
                self.side_scan_votes.clear()
                self.nan_side_votes.clear()
                self.clear_clearance_histories()
                self.current_state = RobotState.OBSTACLE_SEARCH
                self.say(
                    f"Starting obstacle side scan attempt "
                    f"{self.side_scan_attempt}/{self.SIDE_MAX_ATTEMPTS}."
                )

        elif self.current_state in (
            RobotState.OBSTACLE_TURN_LEFT,
            RobotState.OBSTACLE_TURN_RIGHT,
        ):
            if (
                now - self.obstacle_turn_start_time
                > self.OBSTACLE_TURN_TIMEOUT_SEC
            ):
                self.fail_obstacle_routine(
                    "Obstacle 90-degree turn made insufficient progress "
                    f"within {self.OBSTACLE_TURN_TIMEOUT_SEC:.1f} s."
                )
            elif not self.odom_is_fresh(now):
                self.fail_obstacle_routine(
                    "Odometry became unavailable during the 90-degree "
                    "obstacle turn."
                )
            else:
                yaw_error = self.wrap_angle(
                    self.obstacle_turn_target_yaw - self.current_yaw
                )
                remaining = abs(yaw_error)

                if remaining <= self.OBSTACLE_TURN_TOLERANCE_RAD:
                    twist = self.stop_twist()
                    self.fine_align_settle_start_time = now
                    self.front_align_stable_count = 0
                    self.front_lost_count = 0
                    self.front_nan_stable_count = 0
                    self.current_state = RobotState.OBSTACLE_FINE_ALIGN_SETTLE
                    self.say(
                        "Odometry-based 90-degree turn complete. "
                        "Stopping briefly before front-LiDAR centre alignment."
                    )
                else:
                    if remaining <= self.OBSTACLE_TURN_CREEP_ZONE_RAD:
                        speed = self.OBSTACLE_TURN_CREEP_SPEED
                    elif remaining <= self.OBSTACLE_TURN_SLOW_ZONE_RAD:
                        speed = self.OBSTACLE_TURN_SLOW_SPEED
                    else:
                        speed = self.OBSTACLE_TURN_SPEED

                    twist.angular.z = math.copysign(speed, yaw_error)

        elif self.current_state == RobotState.OBSTACLE_FINE_ALIGN_SETTLE:
            twist = self.stop_twist()
            if (
                now - self.fine_align_settle_start_time
                >= self.FINE_ALIGN_SETTLE_DURATION_SEC
            ):
                self.fine_align_start_time = now
                self.front_align_stable_count = 0
                self.front_lost_count = 0
                self.front_nan_stable_count = 0
                self.current_state = RobotState.OBSTACLE_FINE_ALIGN
                self.say(
                    "Starting fine front-LiDAR alignment using the midpoint "
                    "of the visible board cluster."
                )

        elif self.current_state == RobotState.OBSTACLE_FINE_ALIGN:
            target = self.front_board_target

            # -----------------------------------------------------------------
            # EDGE-RETURN GUARD  (this was a real trap)
            #
            # A board sitting just inside the LDS-02 minimum range does NOT
            # go fully blind. Everything straight ahead is unmeasurable, but
            # the oblique EDGES still return, because the slant range is
            # longer than the perpendicular distance:
            #
            #     slant = face_distance / cos(angle)
            #     0.145 m face  ->  0.160 m at 25 degrees  ->  VALID reading
            #
            # The clusterer happily turned those two slivers into "a valid
            # board cluster" roughly 25-32 degrees off axis. Because the
            # close-board reverse was gated on `target is None`, it could
            # never fire; the robot instead tried to aim at one sliver, saw
            # the other one on the next scan, flip-flopped between them and
            # timed out with the board untouched.
            #
            # If the pusher axis has no return at all AND the front sector
            # is mostly too-close returns, the cluster is an edge artefact,
            # not the board face. Drop it so the existing (unchanged) NaN
            # reverse logic below can do its job.
            # -----------------------------------------------------------------
            if (
                self.FRONT_NAN_EDGE_FALLBACK
                and target is not None
                and not math.isfinite(target.axis_distance)
                and self.front_nan_ratio >= self.FRONT_NAN_MIN_RATIO
            ):
                self.detail(
                    "Ignoring an edge-return sliver: "
                    f"bearing={target.bearing_deg:+.1f} deg, "
                    f"width={target.width_m:.3f} m, "
                    f"near={target.near_distance:.3f} m, "
                    f"front_NaN={self.front_nan_ratio:.2f}. The board is "
                    "straight ahead but too close to measure.",
                    throttle_duration_sec=0.8,
                )
                target = None

            if target is None:
                self.front_lost_count += 1
                self.front_align_stable_count = 0
            else:
                self.front_lost_count = 0

            # A valid board cluster always keeps the original routine. The
            # reverse fallback is considered only when there is no valid
            # target and the front NaN pattern repeats across fresh scans.
            if (
                target is None
                and not self.front_nan_reverse_attempted
                and self.front_nan_ratio >= self.FRONT_NAN_MIN_RATIO
            ):
                self.front_nan_stable_count += 1
            else:
                self.front_nan_stable_count = 0

            fallback_reason: Optional[str] = None
            if not self.front_nan_reverse_attempted:
                if (
                    self.front_nan_stable_count
                    >= self.FRONT_NAN_CONFIRM_SCANS
                ):
                    fallback_reason = (
                        f"front NaN ratio stayed above "
                        f"{self.FRONT_NAN_MIN_RATIO:.2f} for "
                        f"{self.front_nan_stable_count} scans"
                    )

            if fallback_reason is not None:
                twist = self.stop_twist()
                self.front_nan_reverse_attempted = True
                self.close_board_indicator_active = True
                self.queue_buzzer_pattern(self.BEEP_FRONT_NAN_REVERSE)
                self.current_state = (
                    RobotState.OBSTACLE_FRONT_NAN_REAR_CHECK
                )
                self.say(
                    f"Front recovery requested because {fallback_reason}. "
                    "Checking the remembered and live rear path before the "
                    "0.10 m fallback reverse."
                )
            elif now - self.fine_align_start_time > self.FINE_ALIGN_TIMEOUT_SEC:
                self.fail_obstacle_routine(
                    "Front LiDAR could not centre the board before the "
                    "fine-alignment timeout."
                )
            elif target is None:
                twist = self.stop_twist()

                self.warn(
                    "No valid front cluster: "
                    f"front_NaN_ratio={self.front_nan_ratio:.2f}, "
                    f"NaN_scans={self.front_nan_stable_count}/"
                    f"{self.FRONT_NAN_CONFIRM_SCANS}, "
                    f"lost_scans={self.front_lost_count}, "
                    f"fallback_attempted={self.front_nan_reverse_attempted}.",
                    throttle_duration_sec=0.8,
                )

                if self.front_lost_count >= 8:
                    self.fail_obstacle_routine(
                        "No valid front board cluster was found after the "
                        "90-degree turn."
                    )
            else:
                aim_error = self.wrap_angle(
                    target.bearing - self.ACTUATOR_AIM_OFFSET_RAD
                )

                # ---------------------------------------------------------
                # "Near the centre is good enough."
                #
                # The robot no longer chases the exact midpoint bearing. It
                # only has to put the pusher axis somewhere inside the
                # usable central band of the board face.
                # ---------------------------------------------------------
                hit_window = self.board_hit_window_m(target)
                lateral_miss = self.aim_lateral_offset_m(target, aim_error)

                # Hysteresis: aim into the inner part of the band, but once
                # inside, tolerate drift out to the full band.
                accept_window = (
                    hit_window
                    if self.front_align_stable_count > 0
                    else hit_window * self.ALIGN_ENTER_FRACTION
                )
                inside_window = (
                    abs(lateral_miss) <= accept_window
                    and abs(aim_error) <= self.FINE_ALIGN_MAX_ANGLE_RAD
                )

                if inside_window:
                    self.front_align_stable_count += 1
                    twist = self.stop_twist()
                else:
                    self.front_align_stable_count = 0
                    angular_speed = max(
                        min(
                            self.FINE_ALIGN_KP * aim_error,
                            self.FINE_ALIGN_MAX_SPEED,
                        ),
                        -self.FINE_ALIGN_MAX_SPEED,
                    )
                    if abs(angular_speed) < self.FINE_ALIGN_MIN_SPEED:
                        angular_speed = math.copysign(
                            self.FINE_ALIGN_MIN_SPEED,
                            aim_error,
                        )
                    twist.angular.z = angular_speed

                self.detail(
                    f"Fine align: board={target.distance:.2f} m, "
                    f"face={self.board_face_distance_m(target):.2f} m, "
                    f"bearing={target.bearing_deg:.1f} deg, "
                    f"width={target.width_m:.2f} m, "
                    f"miss={lateral_miss * 100.0:+.1f} cm of "
                    f"+/-{accept_window * 100.0:.1f} cm, "
                    f"stable={self.front_align_stable_count}/"
                    f"{self.FRONT_ALIGN_CONFIRM_SCANS}.",
                    throttle_duration_sec=0.8,
                )

                if (
                    self.front_align_stable_count
                    >= self.FRONT_ALIGN_CONFIRM_SCANS
                ):
                    twist = self.stop_twist()
                    self.obstacle_facing_yaw = self.current_yaw
                    self.approach_start_x = self.current_x
                    self.approach_start_y = self.current_y
                    self.approach_start_time = now
                    self.approach_aim_retry_count = 0
                    self.front_lost_count = 0
                    self.current_state = RobotState.OBSTACLE_APPROACH
                    self.queue_buzzer_pattern(self.BEEP_ALIGNED)
                    self.say(
                        "Pusher is aimed inside the board's centre band "
                        f"(miss {lateral_miss * 100.0:+.1f} cm, allowed "
                        f"+/-{hit_window * 100.0:.1f} cm) for "
                        f"{self.FRONT_ALIGN_CONFIRM_SCANS} scans. "
                        "Starting the LiDAR-corrected approach."
                    )

        elif self.current_state == RobotState.OBSTACLE_FRONT_NAN_REAR_CHECK:
            rear_clear, rear_status = (
                self.evaluate_front_nan_reverse_corridor(
                    self.FRONT_NAN_REVERSE_DISTANCE_M
                )
            )
            if not rear_clear:
                self.fail_obstacle_routine(
                    f"Front-recovery reverse blocked: {rear_status}"
                )
            else:
                self.front_nan_reverse_start_x = self.current_x
                self.front_nan_reverse_start_y = self.current_y
                self.front_nan_reverse_heading_yaw = self.current_yaw
                self.front_nan_reverse_start_time = now
                self.front_nan_reverse_used = True
                self.current_state = RobotState.OBSTACLE_FRONT_NAN_REVERSE
                self.say(
                    f"Rear path accepted ({rear_status}). Reversing "
                    f"{self.FRONT_NAN_REVERSE_DISTANCE_M:.2f} m to bring the "
                    "front board into the LDS-02 valid range."
                )

        elif self.current_state == RobotState.OBSTACLE_FRONT_NAN_REVERSE:
            moved = math.hypot(
                self.current_x - self.front_nan_reverse_start_x,
                self.current_y - self.front_nan_reverse_start_y,
            )
            remaining_distance = max(
                0.0,
                self.FRONT_NAN_REVERSE_DISTANCE_M - moved,
            )
            rear_clear, rear_status = (
                self.evaluate_front_nan_reverse_corridor(
                    remaining_distance
                )
            )

            if (
                now - self.front_nan_reverse_start_time
                > self.FRONT_NAN_REVERSE_TIMEOUT_SEC
            ):
                self.fail_obstacle_routine(
                    "Front-NaN reverse made insufficient progress within "
                    f"{self.FRONT_NAN_REVERSE_TIMEOUT_SEC:.1f} s."
                )
            elif not rear_clear:
                self.fail_obstacle_routine(
                    "Rear path became unsafe during the front-NaN reverse: "
                    f"{rear_status}"
                )
            elif moved >= self.FRONT_NAN_REVERSE_DISTANCE_M:
                twist = self.stop_twist()
                self.fine_align_settle_start_time = now
                self.front_align_stable_count = 0
                self.front_lost_count = 0
                self.front_nan_stable_count = 0
                self.current_state = RobotState.OBSTACLE_FINE_ALIGN_SETTLE
                self.say(
                    "Front-NaN reverse complete. Settling before retrying the "
                    "original front-LiDAR midpoint alignment."
                )
            else:
                heading_error = self.wrap_angle(
                    self.front_nan_reverse_heading_yaw - self.current_yaw
                )
                twist.linear.x = self.FRONT_NAN_REVERSE_SPEED
                twist.angular.z = max(
                    min(
                        self.FRONT_NAN_HEADING_KP * heading_error,
                        self.FRONT_NAN_MAX_ANGULAR_SPEED,
                    ),
                    -self.FRONT_NAN_MAX_ANGULAR_SPEED,
                )
                self.detail(
                    f"Front-NaN reverse: moved={moved:.3f} m, "
                    f"target={self.FRONT_NAN_REVERSE_DISTANCE_M:.3f} m, "
                    f"{rear_status}.",
                    throttle_duration_sec=0.8,
                )

        elif self.current_state == RobotState.OBSTACLE_CENTERING_EXTRA:
            # Retained only for compatibility with older state names.
            self.fine_align_start_time = now
            self.current_state = RobotState.OBSTACLE_FINE_ALIGN

        elif self.current_state == RobotState.OBSTACLE_APPROACH:
            target = self.front_board_target
            approach_distance = math.hypot(
                self.current_x - self.approach_start_x,
                self.current_y - self.approach_start_y,
            )
            approach_elapsed = now - self.approach_start_time

            if (
                approach_distance > self.APPROACH_MAX_DISTANCE_M
                or approach_elapsed > self.APPROACH_TIMEOUT_SEC
            ):
                self.fail_obstacle_routine(
                    "Board approach exceeded its bounded envelope: "
                    f"distance={approach_distance:.2f}/"
                    f"{self.APPROACH_MAX_DISTANCE_M:.2f} m, "
                    f"time={approach_elapsed:.1f}/"
                    f"{self.APPROACH_TIMEOUT_SEC:.1f} s."
                )
                self.publish_motion(self.stop_twist(), alarm=True)
                return

            if target is None:
                self.front_lost_count += 1
                twist = self.stop_twist()

                if self.front_lost_count >= 4:
                    self.fail_obstacle_routine(
                        "Board cluster was lost for four consecutive LiDAR "
                        "scans during approach."
                    )
            else:
                self.front_lost_count = 0
                aim_error = self.wrap_angle(
                    target.bearing - self.ACTUATOR_AIM_OFFSET_RAD
                )

                correction = max(
                    min(
                        self.APPROACH_BALANCE_KP * aim_error,
                        self.APPROACH_MAX_ANGULAR_SPEED,
                    ),
                    -self.APPROACH_MAX_ANGULAR_SPEED,
                )

                # Same "near the centre is good enough" test as fine
                # alignment, kept live all the way in so that closing on the
                # board does not silently tighten the requirement.
                hit_window = self.board_hit_window_m(target)
                lateral_miss = self.aim_lateral_offset_m(target, aim_error)
                aim_ok = (
                    abs(lateral_miss) <= hit_window
                    and abs(aim_error) <= self.APPROACH_MAX_ANGLE_RAD
                )

                # Distance that the push actually has to cover: measured
                # along the pusher axis where possible. near_distance is kept
                # purely as a chassis-protection backstop, because on an
                # obliquely-seen board the nearest point can be an edge that
                # is well off to one side.
                face_distance = self.board_face_distance_m(target)
                reached = (
                    face_distance <= self.STOP_DISTANCE_M
                    or target.near_distance <= self.FRONT_SAFETY_STOP_M
                )

                self.detail(
                    f"Approach: face={face_distance:.2f} m, "
                    f"near={target.near_distance:.2f} m, "
                    f"miss={lateral_miss * 100.0:+.1f} cm of "
                    f"+/-{hit_window * 100.0:.1f} cm, "
                    f"bearing={target.bearing_deg:+.1f} deg, "
                    f"width={target.width_m:.2f} m.",
                    throttle_duration_sec=0.8,
                )

                if reached:
                    # Last-resort valve: an off-centre hit topples the board;
                    # a perfectly aimed timeout does not. After enough failed
                    # scans, accept any aim that still lands on the board.
                    forced_ok = False
                    if not aim_ok:
                        self.approach_aim_retry_count += 1
                        forced_ok = (
                            self.approach_aim_retry_count
                            >= self.APPROACH_AIM_MAX_SCANS
                            and abs(aim_error) <= self.APPROACH_MAX_ANGLE_RAD
                            and self.aim_lands_on_board(target, aim_error)
                        )

                    if aim_ok or forced_ok:
                        twist = self.stop_twist()
                        self.obstacle_brake_start_time = now
                        # Latch the last trustworthy board distance NOW. The
                        # board drops inside the LDS-02 minimum range during
                        # the push, so this is the last good measurement.
                        self.tackle_board_distance_m = face_distance
                        self.current_state = RobotState.OBSTACLE_BRAKE
                        if forced_ok:
                            self.warn(
                                "Could not centre the board within "
                                f"{self.APPROACH_AIM_MAX_SCANS} scans, but "
                                f"the pusher still lands on it "
                                f"({lateral_miss * 100.0:+.1f} cm off "
                                "centre). Accepting the off-centre hit and "
                                "braking."
                            )
                        else:
                            self.say(
                                f"Board face reached at {face_distance:.2f} m, "
                                f"pusher {lateral_miss * 100.0:+.1f} cm off "
                                f"centre (allowed "
                                f"+/-{hit_window * 100.0:.1f} cm). Braking."
                            )
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = correction
                else:
                    self.approach_aim_retry_count = 0
                    twist.angular.z = correction
                    rotate_only = (
                        abs(lateral_miss)
                        > hit_window * self.APPROACH_ROTATE_ONLY_FACTOR
                        or abs(aim_error) > self.APPROACH_MAX_ANGLE_RAD
                    )
                    if rotate_only:
                        twist.linear.x = 0.0
                    elif face_distance <= self.APPROACH_NEAR_DISTANCE_M:
                        twist.linear.x = self.APPROACH_NEAR_SPEED
                    else:
                        twist.linear.x = self.APPROACH_SPEED

        elif self.current_state == RobotState.OBSTACLE_BRAKE:
            twist = self.stop_twist()
            if now - self.obstacle_brake_start_time >= self.STOP_HOLD_DURATION:
                self.actuator_extend_start_time = now
                self.current_state = RobotState.OBSTACLE_ACTUATOR_EXTEND
                self.queue_buzzer_pattern(self.BEEP_TACKLE)
                self.say(
                    "Robot braked and centred. Extending the actuator fully "
                    "while remaining stationary."
                )

        elif self.current_state == RobotState.OBSTACLE_ACTUATOR_EXTEND:
            twist = self.stop_twist()
            self.set_pulse_ms(self.ACTUATOR_EXTEND_PULSE_MS)
            elapsed = now - self.actuator_extend_start_time

            self.detail(
                f"Stationary actuator extension: {elapsed:.1f}/"
                f"{self.ACTUATOR_EXTEND_HOLD_SEC:.1f} s.",
                throttle_duration_sec=0.8,
            )

            if elapsed >= self.ACTUATOR_EXTEND_HOLD_SEC:
                self.tackle_start_time = now
                self.tackle_start_x = self.current_x
                self.tackle_start_y = self.current_y
                self.tackle_board_down_stable_count = 0
                self.tackle_last_push_distance = 0.0
                self.tackle_stall_count = 0
                self.tackle_contact_detected = False
                self.tackle_stop_reason = ""

                # How far the wheels must actually travel for the extended
                # pusher tip to reach the board AND push it past tipping:
                #
                #   push = board face distance along the pusher axis
                #          - 0.14 m  (LiDAR centre -> retracted pusher tip)
                #          - actuator reach beyond the retracted tip
                #          + overtravel
                #
                # board_distance was latched at the braking moment and is
                # measured along the pusher axis, so an off-centre or
                # obliquely-seen board still gives the right push length.
                #
                # This is odometry-only on purpose: the board is inside the
                # LDS-02 minimum range for the whole push, so LiDAR cannot
                # confirm anything until the board has actually fallen.
                board_distance = self.tackle_board_distance_m
                if not math.isfinite(board_distance) or board_distance <= 0.0:
                    board_distance = self.STOP_DISTANCE_M

                raw_push = (
                    board_distance
                    - self.LIDAR_TO_FRONT_EDGE_M
                    - self.ACTUATOR_REACH_M
                    + self.TOPPLE_OVERTRAVEL_M
                )
                self.tackle_required_push_m = min(
                    max(raw_push, self.TACKLE_MIN_PUSH_DISTANCE_M),
                    self.TACKLE_MAX_PUSH_DISTANCE_M,
                )

                self.current_state = RobotState.OBSTACLE_TACKLE
                self.say(
                    "Actuator extended. Pushing "
                    f"{self.tackle_required_push_m:.3f} m "
                    f"(board was {board_distance:.3f} m away)."
                )

        elif self.current_state == RobotState.OBSTACLE_TACKLE:
            elapsed = now - self.tackle_start_time
            pushed_distance = math.hypot(
                self.current_x - self.tackle_start_x,
                self.current_y - self.tackle_start_y,
            )
            self.set_pulse_ms(self.ACTUATOR_EXTEND_PULSE_MS)

            # ---------------------------------------------------------------
            # BUG FIX (was the main cause of missed topples)
            #
            # The old test treated "front_board_target is None" as evidence
            # that the board had fallen. But get_front_board_target() throws
            # away every return below FRONT_MIN_RANGE_M (0.16 m), so the
            # target ALWAYS disappears once the board is closer than that --
            # which happens by design, because braking is at 0.20 m and the
            # robot then drives forward. front_dist likewise becomes inf,
            # and inf > BOARD_DOWN_THRESHOLD_M is also true.
            #
            # Result: the push declared success after roughly 6 cm and quit
            # while the board was still standing.
            #
            # The board is only "down" if the front is POSITIVELY OPEN, and
            # only once the geometry says the pusher must already have
            # reached it.
            # ---------------------------------------------------------------
            # "Open" has to mean two things at once:
            #   - the front sector is NOT full of too-close returns (the board
            #     is no longer pressed against the pusher), AND
            #   - what is left is either genuinely empty (inf, no return
            #     within range) or something comfortably far away.
            #
            # Testing math.isfinite() alone was wrong: when the board topples
            # onto open floor the front sector returns inf, isfinite(inf) is
            # False, and the "board fell" stop reason could never fire. The
            # push then ran on to TACKLE_MAX_PUSH_DISTANCE_M every time.
            front_blind = self.front_nan_ratio >= self.FRONT_NAN_MIN_RATIO
            front_open = not front_blind and (
                math.isinf(self.front_dist)
                or self.front_dist > self.BOARD_DOWN_THRESHOLD_M
            )
            board_down_candidate = (
                pushed_distance >= self.tackle_required_push_m
                and front_open
            )

            if board_down_candidate:
                self.tackle_board_down_stable_count += 1
            else:
                self.tackle_board_down_stable_count = 0

            # Commanded forward but odometry barely moving == loaded against
            # the board. Useful as a positive contact signal for the log and
            # as a backstop if the board refuses to fall.
            progress = pushed_distance - self.tackle_last_push_distance
            self.tackle_last_push_distance = pushed_distance
            if progress < self.TACKLE_STALL_PROGRESS_M:
                self.tackle_stall_count += 1
            else:
                self.tackle_stall_count = 0
            if self.tackle_stall_count >= self.TACKLE_STALL_CONFIRM_SCANS:
                self.tackle_contact_detected = True

            stop_reason: Optional[str] = None
            if (
                self.tackle_board_down_stable_count
                >= self.TACKLE_BOARD_DOWN_CONFIRM_SCANS
            ):
                stop_reason = "board fell (front is now clear)"
            elif (
                pushed_distance >= self.tackle_required_push_m
                and self.tackle_stall_count
                >= self.TACKLE_STALL_CONFIRM_SCANS
            ):
                stop_reason = "full push done, wheels stalled against board"
            elif pushed_distance >= self.TACKLE_MAX_PUSH_DISTANCE_M:
                stop_reason = "maximum allowed push distance reached"
            elif elapsed >= self.TACKLE_TIMEOUT_SEC:
                stop_reason = "push timed out"

            if stop_reason is not None:
                twist = self.stop_twist()
                self.tackle_stop_reason = stop_reason
                self.tackle_contact_hold_start_time = now
                self.current_state = RobotState.OBSTACLE_TACKLE_HOLD
                self.say(
                    f"Push finished: {stop_reason} "
                    f"(pushed {pushed_distance:.3f} m of "
                    f"{self.tackle_required_push_m:.3f} m). Holding."
                )
            else:
                twist.linear.x = self.TACKLE_PUSH_SPEED
                # Hold the board-facing heading so wheel slip on contact
                # cannot walk the pusher sideways off the board.
                yaw_error = self.wrap_angle(
                    self.obstacle_facing_yaw - self.current_yaw
                )
                twist.angular.z = max(
                    min(
                        self.TACKLE_YAW_HOLD_KP * yaw_error,
                        self.TACKLE_YAW_HOLD_MAX,
                    ),
                    -self.TACKLE_YAW_HOLD_MAX,
                )
                self.detail(
                    f"push {pushed_distance:.3f}/"
                    f"{self.tackle_required_push_m:.3f} m, "
                    f"contact={self.tackle_contact_detected}, "
                    f"front_open={front_open}",
                    throttle=0.8,
                )

        elif self.current_state == RobotState.OBSTACLE_TACKLE_HOLD:
            twist = self.stop_twist()
            self.set_pulse_ms(self.ACTUATOR_EXTEND_PULSE_MS)
            hold_elapsed = now - self.tackle_contact_hold_start_time

            if hold_elapsed >= self.TACKLE_CONTACT_HOLD_SEC:
                self.required_reverse_distance = math.hypot(
                    self.current_x - self.approach_start_x,
                    self.current_y - self.approach_start_y,
                )
                self.reverse_start_x = self.current_x
                self.reverse_start_y = self.current_y
                self.return_start_time = now
                self.current_state = RobotState.OBSTACLE_RETURN
                self.detail(
                    f"Contact hold complete ({self.tackle_stop_reason}). "
                    f"Retracting during the return; reverse target="
                    f"{self.required_reverse_distance:.3f} m."
                )

        elif self.current_state == RobotState.OBSTACLE_RETURN:
            self.set_pulse_ms(self.ACTUATOR_RETRACT_PULSE_MS)

            reversed_distance = math.hypot(
                self.current_x - self.reverse_start_x,
                self.current_y - self.reverse_start_y,
            )
            remaining = self.required_reverse_distance - reversed_distance

            if remaining <= self.REVERSE_SLOW_ZONE_M:
                twist.linear.x = self.REVERSE_SLOW_SPEED
            else:
                twist.linear.x = self.REVERSE_SPEED

            self.detail(
                f"Reversing and retracting: reversed={reversed_distance:.3f} m, "
                f"target={self.required_reverse_distance:.3f} m, "
                f"remaining={remaining:.3f} m, "
                f"line_frames={self.reverse_line_seen_count}.",
                throttle_duration_sec=1.0,
            )

            if reversed_distance >= max(
                0.0,
                self.required_reverse_distance
                - self.REVERSE_STOP_MARGIN_M,
            ):
                twist = self.stop_twist()
                self.stop_actuator_signal()
                if self.front_nan_reverse_used:
                    self.return_start_time = now
                    self.current_state = (
                        RobotState.OBSTACLE_FRONT_NAN_RETURN_FORWARD
                    )
                    self.say(
                        "Approach distance recovered. Driving forward to undo "
                        "the earlier 0.10 m front-NaN offset before restoring "
                        "the track yaw."
                    )
                else:
                    self.obstacle_turn_start_time = now
                    self.current_state = RobotState.OBSTACLE_RETURN_YAW
                    self.say(
                        "Reverse distance recovered. Restoring the original "
                        "main-track yaw before Pixy line reacquisition."
                    )
            elif now - self.return_start_time >= self.RETURN_TIMEOUT_SEC:
                twist = self.stop_twist()
                self.stop_actuator_signal()
                self.fail_obstacle_routine(
                    "Return timeout reached before the robot recovered "
                    "its approach distance."
                )

        elif (
            self.current_state
            == RobotState.OBSTACLE_FRONT_NAN_RETURN_FORWARD
        ):
            self.set_pulse_ms(self.ACTUATOR_RETRACT_PULSE_MS)
            dx = self.obstacle_start_x - self.current_x
            dy = self.obstacle_start_y - self.current_y
            distance_to_start = math.hypot(dx, dy)

            if distance_to_start <= self.RETURN_TOLERANCE_M:
                twist = self.stop_twist()
                self.stop_actuator_signal()
                self.obstacle_turn_start_time = now
                self.current_state = RobotState.OBSTACLE_RETURN_YAW
                self.say(
                    "The 0.10 m front-NaN offset has been recovered. "
                    "Restoring the original track yaw."
                )
            elif now - self.return_start_time >= self.RETURN_TIMEOUT_SEC:
                twist = self.stop_twist()
                self.stop_actuator_signal()
                self.fail_obstacle_routine(
                    "Timed out while undoing the front-NaN reverse offset."
                )
            else:
                desired_yaw = math.atan2(dy, dx)
                heading_error = self.wrap_angle(
                    desired_yaw - self.current_yaw
                )
                twist.angular.z = max(
                    min(
                        self.FRONT_NAN_HEADING_KP * heading_error,
                        self.FRONT_NAN_MAX_ANGULAR_SPEED,
                    ),
                    -self.FRONT_NAN_MAX_ANGULAR_SPEED,
                )
                if abs(heading_error) <= math.radians(12.0):
                    twist.linear.x = self.FRONT_NAN_RETURN_SPEED

                self.detail(
                    "Undoing front-NaN offset: "
                    f"distance={distance_to_start:.3f} m, "
                    f"heading_error={math.degrees(heading_error):.1f} deg.",
                    throttle_duration_sec=0.8,
                )

        elif self.current_state == RobotState.OBSTACLE_RETURN_YAW:
            if (
                now - self.obstacle_turn_start_time
                > self.OBSTACLE_TURN_TIMEOUT_SEC
            ):
                self.fail_obstacle_routine(
                    "Return-yaw turn made insufficient progress within "
                    f"{self.OBSTACLE_TURN_TIMEOUT_SEC:.1f} s."
                )
            elif not self.odom_is_fresh(now):
                self.fail_obstacle_routine(
                    "Odometry became unavailable while restoring the "
                    "main-track heading."
                )
            else:
                yaw_error = self.wrap_angle(
                    self.obstacle_start_yaw - self.current_yaw
                )
                remaining = abs(yaw_error)

                if remaining <= self.OBSTACLE_TURN_TOLERANCE_RAD:
                    twist = self.stop_twist()
                    self.finish_obstacle_routine()
                else:
                    if remaining <= self.OBSTACLE_TURN_CREEP_ZONE_RAD:
                        speed = self.OBSTACLE_TURN_CREEP_SPEED
                    elif remaining <= self.OBSTACLE_TURN_SLOW_ZONE_RAD:
                        speed = self.OBSTACLE_TURN_SLOW_SPEED
                    else:
                        speed = self.OBSTACLE_TURN_SPEED
                    twist.angular.z = math.copysign(speed, yaw_error)

        self.publish_motion(twist, alarm=False)

    def start_normal_obstacle_turn(
        self,
        selected_side: str,
        selected_distance: float,
    ) -> None:
        """Start the original 90-degree turn toward a finite-distance board."""
        self.save_reverse_corridor_for_selected_side(selected_side)
        self.turn_direction = selected_side
        self.front_align_stable_count = 0
        self.front_lost_count = 0
        self.queue_buzzer_pattern(self.BEEP_SIDE_CONFIRMED)

        turn_sign = 1.0 if selected_side == "LEFT" else -1.0
        self.obstacle_turn_target_yaw = self.wrap_angle(
            self.obstacle_start_yaw
            + turn_sign * math.radians(self.OBSTACLE_COARSE_TURN_DEG)
        )
        self.obstacle_turn_start_time = time.time()
        self.current_state = (
            RobotState.OBSTACLE_TURN_LEFT
            if selected_side == "LEFT"
            else RobotState.OBSTACLE_TURN_RIGHT
        )
        self.say(
            f"Board confirmed on {selected_side} at "
            f"{selected_distance:.2f} m. Turning "
            f"{self.OBSTACLE_COARSE_TURN_DEG:.0f} degrees using the original "
            "obstacle routine, with no pre-turn reverse."
        )

    def start_nan_inferred_obstacle_turn(self, side: str) -> None:
        """Use a repeated one-sided NaN pattern to select the original turn."""
        if side not in ("LEFT", "RIGHT"):
            self.fail_obstacle_routine(
                "NaN-inferred obstacle turn received an invalid side."
            )
            return

        self.save_reverse_corridor_for_selected_side(side)
        self.turn_direction = side
        self.front_align_stable_count = 0
        self.front_lost_count = 0
        self.close_board_indicator_active = True
        self.queue_buzzer_pattern(self.BEEP_NAN_CLOSE_BOARD)

        turn_sign = 1.0 if side == "LEFT" else -1.0
        self.obstacle_turn_target_yaw = self.wrap_angle(
            self.obstacle_start_yaw
            + turn_sign * math.radians(self.OBSTACLE_COARSE_TURN_DEG)
        )
        self.obstacle_turn_start_time = time.time()
        self.current_state = (
            RobotState.OBSTACLE_TURN_LEFT
            if side == "LEFT"
            else RobotState.OBSTACLE_TURN_RIGHT
        )
        self.say(
            f"Repeated one-sided NaN votes selected the nearer {side} board. "
            f"Turning {self.OBSTACLE_COARSE_TURN_DEG:.0f} degrees using the "
            "original obstacle routine, with no pre-turn reverse."
        )

    def clear_clearance_histories(self) -> None:
        """Empty every LiDAR sample buffer so the next scan batch starts clean."""
        self.left_clearance_samples.clear()
        self.right_clearance_samples.clear()
        self.left_nan_ratio_samples.clear()
        self.right_nan_ratio_samples.clear()
        self.rear_distance_samples.clear()
        self.rear_nan_ratio_samples.clear()

    def save_reverse_corridor_for_selected_side(self, side: str) -> None:
        """Remember the side that will become rear after the 90-degree turn."""
        if side == "LEFT":
            opposite_distances = self.right_clearance_samples
            opposite_nan_ratios = self.right_nan_ratio_samples
        else:
            opposite_distances = self.left_clearance_samples
            opposite_nan_ratios = self.left_nan_ratio_samples

        self.opposite_side_clearance_m = (
            float(median(opposite_distances))
            if opposite_distances
            else float("inf")
        )
        self.opposite_side_nan_ratio = (
            float(median(opposite_nan_ratios))
            if opposite_nan_ratios
            else 0.0
        )
        self.rear_self_distance_baseline_m = (
            float(median(self.rear_distance_samples))
            if self.rear_distance_samples
            else float("inf")
        )
        self.rear_self_nan_baseline = (
            float(median(self.rear_nan_ratio_samples))
            if self.rear_nan_ratio_samples
            else 0.0
        )
        self.say(
            f"Saved post-turn reverse corridor: selected={side}, "
            f"opposite_clearance={self.opposite_side_clearance_m:.2f} m, "
            f"opposite_NaN={self.opposite_side_nan_ratio:.2f}, "
            f"rear_self_distance={self.rear_self_distance_baseline_m:.2f} m, "
            f"rear_self_NaN={self.rear_self_nan_baseline:.2f}."
        )

    def evaluate_front_nan_reverse_corridor(
        self,
        remaining_motion_m: float,
    ) -> Tuple[bool, str]:
        """Combine pre-turn clearance with rear self-reading compensation."""
        required = (
            remaining_motion_m
            + self.REAR_OVERHANG_M
            + self.REAR_NAN_REVERSE_MARGIN_M
        )

        remembered_distance_clear = (
            math.isinf(self.opposite_side_clearance_m)
            or self.opposite_side_clearance_m >= required
        )
        remembered_nan_clear = (
            self.opposite_side_nan_ratio
            < self.REAR_NAN_INCREASE_BLOCK_RATIO
        )

        rear_matches_self = (
            math.isfinite(self.rear_dist)
            and math.isfinite(self.rear_self_distance_baseline_m)
            and abs(
                self.rear_dist - self.rear_self_distance_baseline_m
            )
            <= self.REAR_BASELINE_MATCH_TOLERANCE_M
        )
        live_distance_clear = (
            rear_matches_self
            or math.isinf(self.rear_dist)
            or self.rear_dist >= required
        )

        rear_nan_increase = max(
            0.0,
            self.rear_nan_ratio - self.rear_self_nan_baseline,
        )
        live_nan_clear = (
            rear_nan_increase
            < self.REAR_NAN_INCREASE_BLOCK_RATIO
        )

        clear = (
            remembered_distance_clear
            and remembered_nan_clear
            and live_distance_clear
            and live_nan_clear
        )
        status = (
            f"required={required:.2f} m, "
            f"remembered_opposite={self.opposite_side_clearance_m:.2f} m/"
            f"NaN {self.opposite_side_nan_ratio:.2f}, "
            f"live_rear={self.rear_dist:.2f} m/"
            f"NaN {self.rear_nan_ratio:.2f}, "
            f"rear_self_match={rear_matches_self}, "
            f"NaN_increase={rear_nan_increase:.2f}"
        )
        return clear, status

    def make_nan_side_vote(self) -> Optional[str]:
        """Vote only when NaNs are strongly concentrated on one side."""
        left_close = (
            self.left_nan_ratio >= self.NAN_SIDE_MIN_RATIO
            and self.left_nan_ratio - self.right_nan_ratio
            >= self.NAN_SIDE_ADVANTAGE
        )
        right_close = (
            self.right_nan_ratio >= self.NAN_SIDE_MIN_RATIO
            and self.right_nan_ratio - self.left_nan_ratio
            >= self.NAN_SIDE_ADVANTAGE
        )
        if left_close and not right_close:
            return "LEFT"
        if right_close and not left_close:
            return "RIGHT"
        return None

    def evaluate_nan_side_votes(self) -> Optional[str]:
        """Decide LEFT or RIGHT from the repeated 'board too close to measure' votes.

        Returns None unless one side wins by at least NAN_REQUIRED_VOTES and
        the other side is clearly behind. One stray NaN is never enough.
        """
        valid_votes = [
            side for side in self.nan_side_votes if side in ("LEFT", "RIGHT")
        ]
        if not valid_votes:
            return None

        counts = Counter(valid_votes)
        selected_side, selected_count = counts.most_common(1)[0]
        other_side = "RIGHT" if selected_side == "LEFT" else "LEFT"
        if (
            selected_count >= self.NAN_REQUIRED_VOTES
            and selected_count > counts.get(other_side, 0)
        ):
            return selected_side
        return None

    def obstacle_turn_speed_for_error(self, remaining: float) -> float:
        """Pick a turn speed from how much of the turn is left (three-step ramp).

        Fast far away, slow near the target, creep for the last bit. This stops
        the robot overshooting the 90 degree turn onto the board.
        """
        if remaining <= self.OBSTACLE_TURN_CREEP_ZONE_RAD:
            return self.OBSTACLE_TURN_CREEP_SPEED
        if remaining <= self.OBSTACLE_TURN_SLOW_ZONE_RAD:
            return self.OBSTACLE_TURN_SLOW_SPEED
        return self.OBSTACLE_TURN_SPEED

    def make_side_scan_vote(self) -> Optional[Tuple[str, float]]:
        """Vote for the nearer valid side, with a 2 cm noise-aware tie band."""
        left_valid = self.SIDE_MIN_RANGE_M <= self.left_side_dist <= self.SIDE_MAX_RANGE_M
        right_valid = self.SIDE_MIN_RANGE_M <= self.right_side_dist <= self.SIDE_MAX_RANGE_M

        if left_valid and right_valid:
            difference = abs(self.left_side_dist - self.right_side_dist)
            if difference < self.SIDE_BOTH_DISTANCE_MARGIN_M:
                return None
            if self.left_side_dist < self.right_side_dist:
                return "LEFT", self.left_side_dist
            return "RIGHT", self.right_side_dist

        if left_valid:
            return "LEFT", self.left_side_dist
        if right_valid:
            return "RIGHT", self.right_side_dist
        return None

    def evaluate_side_scan_votes(self) -> Optional[Tuple[str, float]]:
        """Decide which side the board is on from a batch of ordinary LiDAR scans.

        Returns (side, median distance), or None if the batch disagreed or the
        distances were spread too widely to be one solid object.
        """
        valid_votes = [vote for vote in self.side_scan_votes if vote is not None]
        if not valid_votes:
            return None

        counts = Counter(side for side, _ in valid_votes)
        selected_side, selected_count = counts.most_common(1)[0]
        if selected_count < self.SIDE_REQUIRED_VOTES:
            self.warn(
                f"Side vote batch rejected: votes={dict(counts)}, "
                f"need {self.SIDE_REQUIRED_VOTES}/{self.SIDE_SCAN_COUNT}."
            )
            return None

        distances = [
            distance
            for side, distance in valid_votes
            if side == selected_side
        ]
        spread = max(distances) - min(distances)
        if spread > self.SIDE_MAX_DISTANCE_SPREAD_M:
            self.warn(
                f"Side vote batch rejected: {selected_side} distance spread "
                f"{spread:.2f} m is too large."
            )
            return None

        return selected_side, float(median(distances))

    def enter_fail_stop(self, reason: str) -> None:
        """Latch the robot into a safe stop that only a human can clear.

        This is the last line of defence. Anything the robot is no longer sure
        about (queued barcodes, half-finished votes, the actuator signal) is
        cancelled, the wheels are stopped, the loud/bright fail-stop outputs
        start, and a recovery card is printed. The state machine will NOT leave
        OBSTACLE_FAIL_STOP until reset_fail_stop_callback() is called.
        """
        self.stop_actuator_signal()
        self.clear_command_queue()
        self.last_command_source_code = None
        self.reset_barcode_vote()
        self.sensor_pause_active = False
        self.sensor_pause_start_time = 0.0
        self.sensor_pause_state = None
        self.sensor_pause_missing = tuple()
        self.sensor_healthy_count = 0

        # Snapshot what the robot was doing BEFORE the state is overwritten
        # with OBSTACLE_FAIL_STOP. Without this the report would only ever be
        # able to say "it is in fail-stop", which tells nobody anything.
        self.fail_stop_count += 1
        state_at_fault = self.current_state
        self.fail_stop_state_at_fault = PHASE_LABELS.get(
            state_at_fault, state_at_fault
        )
        if self.odom_is_fresh():
            self.fail_stop_pose = (
                f"x={self.current_x:+.3f} m, y={self.current_y:+.3f} m, "
                f"heading={math.degrees(self.current_yaw):+.1f} deg"
            )
        else:
            self.fail_stop_pose = "unknown (odometry was not fresh)"

        self.fail_stop_enter_time = time.time()
        self.fail_stop_time_text = time.strftime(
            "%H:%M:%S", time.localtime(self.fail_stop_enter_time)
        )
        self.fail_stop_blink_state = False
        self.last_fail_stop_blink_time = 0.0
        self.last_fail_stop_buzzer_time = self.fail_stop_enter_time
        self.fail_stop_reason = reason
        self.last_fail_stop_reminder_time = self.fail_stop_enter_time
        self.current_state = RobotState.OBSTACLE_FAIL_STOP
        self.buzzer_queue.clear()
        self.queue_buzzer_pattern(self.BEEP_FAIL_STOP, gap_sec=0.14)
        self.write_fail_stop_html(reason)
        self.print_fail_stop_terminal_alert(reason)
        self.publish_motion(self.stop_twist(), alarm=True)
        self.get_logger().error(
            f"Controller entered fail-stop: {reason} After the stop hold, "
            "teleop may reposition the robot. Reset command: "
            f"{self.RESET_FAIL_STOP_COMMAND}"
        )

    def fail_obstacle_routine(self, reason: str) -> None:
        """Fail-stop with the reason tagged as coming from the obstacle routine."""
        self.enter_fail_stop(f"Obstacle routine: {reason}")

    def is_blind_reading(self, value: float, range_min: float) -> bool:
        """True if this sample means 'something is too close to measure'.

        Deliberately does NOT count +inf: infinity means 'no return within
        the maximum range', i.e. open space, which is the opposite of a
        board pressed up against the robot.
        """
        if math.isnan(value):
            return self.BLIND_VALUE_MODE in ("nan", "both")

        if math.isinf(value):
            return False

        if value <= self.BLIND_ZERO_EPSILON_M:
            return self.BLIND_VALUE_MODE in ("zero", "both")

        if 0.0 < value < range_min:
            return self.BLIND_VALUE_MODE == "both"

        return False

    def get_sector_nan_ratio(
        self,
        msg: LaserScan,
        center_angle: float,
        min_deg: float,
        max_deg: float,
    ) -> float:
        """Return the fraction of the sector that is 'too close to measure'."""
        min_rad = center_angle + math.radians(min_deg)
        max_rad = center_angle + math.radians(max_deg)
        range_min = float(getattr(msg, "range_min", 0.0) or 0.0)
        sample_count = 0
        nan_count = 0

        for i, raw_distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            in_sector = min_rad <= angle <= max_rad

            if min_rad < -math.pi:
                wrapped_min = min_rad + 2.0 * math.pi
                in_sector = angle >= wrapped_min or angle <= max_rad
            elif max_rad > math.pi:
                wrapped_max = max_rad - 2.0 * math.pi
                in_sector = angle >= min_rad or angle <= wrapped_max

            if not in_sector:
                continue

            sample_count += 1
            if self.is_blind_reading(float(raw_distance), range_min):
                nan_count += 1

        if sample_count == 0:
            return 0.0
        return nan_count / sample_count

    def get_sector_min_distance(
        self,
        msg: LaserScan,
        center_angle: float,
        min_deg: float,
        max_deg: float,
    ) -> float:
        """Closest valid LiDAR reading inside one angular sector, in metres.

        Readings below the sensor's own range_min are noise, not measurements,
        so they are dropped. Letting them through used to make front_dist jump
        to a silly small number and ruined the board tests.
        """
        min_rad = center_angle + math.radians(min_deg)
        max_rad = center_angle + math.radians(max_deg)
        # Anything below the sensor's own minimum range is not a measurement,
        # it is noise. Letting such values through made front_dist jump to a
        # bogus small number and corrupted the board-down test.
        floor_m = max(
            self.BLIND_ZERO_EPSILON_M,
            float(getattr(msg, "range_min", 0.0) or 0.0),
        )

        valid_ranges: List[float] = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))

            in_sector = min_rad <= angle <= max_rad

            # Handle a sector crossing the -pi/pi boundary.
            if min_rad < -math.pi:
                wrapped_min = min_rad + 2.0 * math.pi
                in_sector = angle >= wrapped_min or angle <= max_rad
            elif max_rad > math.pi:
                wrapped_max = max_rad - 2.0 * math.pi
                in_sector = angle >= min_rad or angle <= wrapped_max

            if not in_sector:
                continue

            if (
                not math.isnan(distance)
                and not math.isinf(distance)
                and distance >= floor_m
            ):
                valid_ranges.append(float(distance))

        if not valid_ranges:
            return float("inf")

        # Median of up to the five nearest points rejects isolated low spikes
        # while still reacting to a narrow board.
        valid_ranges.sort()
        nearest = valid_ranges[: min(5, len(valid_ranges))]
        return float(median(nearest))

    # =========================================================================
    # AIM WINDOW HELPERS  ("near the centre is good enough")
    # =========================================================================

    def board_hit_window_m(self, target: LidarBoardTarget) -> float:
        """How far off the board centre the pusher may land, in metres.

        Half the measured board width, minus a keep-off margin at each edge,
        clamped so a badly measured width cannot produce a silly window.
        In ALIGN_MODE=centre this collapses to the old strict behaviour.
        """
        if self.ALIGN_MODE == "centre":
            # Old behaviour, expressed in the same units: the angular
            # tolerance converted to a distance on the board face.
            reference = target.distance
            if not math.isfinite(reference) or reference <= 0.0:
                reference = self.STOP_DISTANCE_M
            return max(
                0.001,
                reference * math.sin(self.FINE_ALIGN_TOLERANCE_RAD),
            )

        width = target.width_m if math.isfinite(target.width_m) else 0.0
        usable = 0.5 * max(0.0, width) - self.BOARD_HIT_MARGIN_M
        return min(
            max(usable, self.BOARD_HIT_WINDOW_MIN_M),
            self.BOARD_HIT_WINDOW_MAX_M,
        )

    def aim_lateral_offset_m(
        self,
        target: LidarBoardTarget,
        aim_error: float,
    ) -> float:
        """Signed sideways miss of the pusher axis on the board face (m).

        Positive means the board centre is to the robot's LEFT of the pusher
        axis, i.e. the robot should turn left to reduce it.
        """
        reference = target.distance
        if not math.isfinite(reference) or reference <= 0.0:
            reference = target.near_distance
        if not math.isfinite(reference) or reference <= 0.0:
            reference = self.STOP_DISTANCE_M
        return reference * math.sin(aim_error)

    def aim_lands_on_board(
        self,
        target: LidarBoardTarget,
        aim_error: float,
    ) -> bool:
        """True if the pusher axis still crosses the board face at all.

        Used only by the last-resort valve in OBSTACLE_APPROACH: better an
        off-centre hit than a perfectly aimed timeout with the board still
        standing.
        """
        if math.isfinite(target.axis_distance):
            return True
        offset = self.aim_lateral_offset_m(target, aim_error)
        half_width = 0.5 * max(0.0, target.width_m)
        return abs(offset) <= max(0.0, half_width - 0.010)

    def board_face_distance_m(self, target: LidarBoardTarget) -> float:
        """Distance the pusher must travel through to reach the board face.

        Prefers the reading taken along the pusher axis. Falls back to the
        nearest cluster point when the axis misses the cluster.
        """
        if math.isfinite(target.axis_distance) and target.axis_distance > 0.0:
            return target.axis_distance
        if math.isfinite(target.near_distance) and target.near_distance > 0.0:
            return target.near_distance
        return self.STOP_DISTANCE_M

    def get_front_board_target(
        self,
        msg: LaserScan,
    ) -> Optional[LidarBoardTarget]:
        """Estimate the midpoint of the nearest coherent front LiDAR cluster."""
        points: List[Tuple[float, float]] = []

        for i, raw_distance in enumerate(msg.ranges):
            distance = float(raw_distance)
            if (
                not math.isfinite(distance)
                or distance < self.FRONT_MIN_RANGE_M
                or distance > self.FRONT_MAX_RANGE_M
            ):
                continue

            lidar_angle = msg.angle_min + i * msg.angle_increment
            relative_bearing = self.wrap_angle(
                lidar_angle - self.LIDAR_FRONT_ANGLE_RAD
            )

            if abs(relative_bearing) <= self.FRONT_CLUSTER_HALF_ANGLE_RAD:
                points.append((relative_bearing, distance))

        if not points:
            return None

        points.sort(key=lambda item: item[0])
        clusters: List[List[Tuple[float, float]]] = []
        current: List[Tuple[float, float]] = []

        for bearing, distance in points:
            if not current:
                current = [(bearing, distance)]
                continue

            previous_bearing, previous_distance = current[-1]
            if (
                abs(bearing - previous_bearing)
                <= self.FRONT_CLUSTER_MAX_ANGLE_GAP_RAD
                and abs(distance - previous_distance)
                <= self.FRONT_CLUSTER_MAX_RANGE_JUMP_M
            ):
                current.append((bearing, distance))
            else:
                clusters.append(current)
                current = [(bearing, distance)]

        if current:
            clusters.append(current)

        candidates: List[LidarBoardTarget] = []

        for cluster in clusters:
            if len(cluster) < self.FRONT_CLUSTER_MIN_POINTS:
                continue

            endpoint_count = min(2, max(1, len(cluster) // 3))
            first_group = cluster[:endpoint_count]
            last_group = cluster[-endpoint_count:]

            first_x = sum(
                distance * math.cos(bearing)
                for bearing, distance in first_group
            ) / len(first_group)
            first_y = sum(
                distance * math.sin(bearing)
                for bearing, distance in first_group
            ) / len(first_group)

            last_x = sum(
                distance * math.cos(bearing)
                for bearing, distance in last_group
            ) / len(last_group)
            last_y = sum(
                distance * math.sin(bearing)
                for bearing, distance in last_group
            ) / len(last_group)

            width_m = math.hypot(last_x - first_x, last_y - first_y)
            if not (
                self.FRONT_CLUSTER_MIN_WIDTH_M
                <= width_m
                <= self.FRONT_CLUSTER_MAX_WIDTH_M
            ):
                continue

            centre_x = 0.5 * (first_x + last_x)
            centre_y = 0.5 * (first_y + last_y)
            centre_distance = math.hypot(centre_x, centre_y)
            centre_bearing = math.atan2(centre_y, centre_x)
            near_distance = min(distance for _, distance in cluster)

            # ---------------------------------------------------------------
            # Distance to the board along the PUSHER AXIS (robot x-axis).
            #
            # The cluster is walked in bearing order as (x = forward,
            # y = left) points. Wherever the board face crosses y = 0 is the
            # spot the pusher will actually hit, so that x is the distance
            # the push has to cover. If the face never crosses y = 0 the
            # pusher axis misses this cluster and +inf is reported.
            # ---------------------------------------------------------------
            cartesian = [
                (distance * math.cos(bearing), distance * math.sin(bearing))
                for bearing, distance in cluster
            ]
            axis_distance = float("inf")
            for (x_a, y_a), (x_b, y_b) in zip(cartesian, cartesian[1:]):
                if (y_a <= 0.0 <= y_b) or (y_b <= 0.0 <= y_a):
                    span = y_b - y_a
                    if abs(span) < 1e-9:
                        crossing_x = 0.5 * (x_a + x_b)
                    else:
                        ratio = (0.0 - y_a) / span
                        crossing_x = x_a + ratio * (x_b - x_a)
                    if crossing_x > 0.0:
                        axis_distance = min(axis_distance, crossing_x)

            y_values = [y for _, y in cartesian]

            candidates.append(
                LidarBoardTarget(
                    distance=centre_distance,
                    bearing=centre_bearing,
                    point_count=len(cluster),
                    width_m=width_m,
                    near_distance=near_distance,
                    axis_distance=axis_distance,
                    left_edge_y=max(y_values),
                    right_edge_y=min(y_values),
                )
            )

        if not candidates:
            return None

        # After the deliberate 90-degree turn, the board should be the nearest
        # coherent front cluster. Centrality breaks close-distance ties.
        candidates.sort(
            key=lambda target: (
                target.near_distance,
                abs(target.bearing),
                -target.point_count,
            )
        )
        return candidates[0]

    # =========================================================================
    # GPIO, LED AND ACTUATOR
    # =========================================================================

    def gpio_write(self, pin: int, value: bool) -> None:
        """Set one GPIO pin high or low, doing nothing if GPIO is unavailable.

        Wrapped in try/except so the program still runs on a laptop with no
        Raspberry Pi attached (useful for dry testing).
        """
        if not self.gpio_ready:
            return

        try:
            GPIO.output(pin, GPIO.HIGH if value else GPIO.LOW)
        except Exception:
            pass

    def all_leds_off(self) -> None:
        """Switch off the left, right and stop LEDs."""
        self.gpio_write(self.LEFT_LED, False)
        self.gpio_write(self.RIGHT_LED, False)
        self.gpio_write(self.STOP_LED, False)

    def set_stop_led(self, on: bool = True) -> None:
        """Light only the red stop LED (left and right indicators off)."""
        self.gpio_write(self.LEFT_LED, False)
        self.gpio_write(self.RIGHT_LED, False)
        self.gpio_write(self.STOP_LED, on)

    def update_blink_state(self) -> None:
        """Toggle the slow blink flag every BLINK_INTERVAL seconds.

        Used by the ordinary lost-line alarm. The fail-stop uses its own, much
        faster blink so the two cannot be mixed up from across the room.
        """
        now = time.time()

        if now - self.last_blink_time >= self.BLINK_INTERVAL:
            self.blink_state = not self.blink_state
            self.last_blink_time = now

    def update_leds_from_twist(self, twist: Twist) -> None:
        """Drive the indicator LEDs from the velocity command being sent.

        Stopped -> red LED, turning left -> left LED, turning right -> right
        LED, driving straight -> both indicators on.
        """
        self.update_blink_state()

        linear_x = twist.linear.x
        angular_z = twist.angular.z

        stopped = abs(linear_x) < 0.01 and abs(angular_z) < 0.01
        turning_left = angular_z > self.TURN_LED_THRESHOLD
        turning_right = angular_z < -self.TURN_LED_THRESHOLD

        if stopped:
            self.set_stop_led(True)
        elif turning_left:
            self.gpio_write(self.RIGHT_LED, False)
            self.gpio_write(self.STOP_LED, False)
            self.gpio_write(self.LEFT_LED, self.blink_state)
        elif turning_right:
            self.gpio_write(self.LEFT_LED, False)
            self.gpio_write(self.STOP_LED, False)
            self.gpio_write(self.RIGHT_LED, self.blink_state)
        else:
            self.all_leds_off()

        # Keep a NaN-inferred selection visually unmistakable even when
        # the TurtleBot sound service is unavailable.
        if self.close_board_indicator_active:
            self.gpio_write(self.STOP_LED, self.blink_state)

    def set_pulse_ms(self, pulse_ms: float) -> None:
        """Send a servo/actuator pulse width in milliseconds via 50 Hz PWM.

        duty cycle % = pulse_ms / 20 ms * 100, because one 50 Hz period is 20 ms.
        Does nothing when ENABLE_ACTUATOR=0, which is how we dry-run the routine.
        """
        if not self.ENABLE_ACTUATOR:
            return

        if not self.gpio_ready or self.pwm is None:
            return

        duty_cycle = (pulse_ms / 20.0) * 100.0

        try:
            self.pwm.ChangeDutyCycle(duty_cycle)
        except Exception as exc:
            self.get_logger().warning(f"Actuator PWM command failed: {exc}")

    def stop_actuator_signal(self) -> None:
        """Set the actuator PWM duty cycle to zero (stop driving it)."""
        if not self.gpio_ready or self.pwm is None:
            return

        try:
            self.pwm.ChangeDutyCycle(0)
        except Exception:
            pass

    # =========================================================================
    # BUZZER
    # =========================================================================

    def request_sound(self, value: int) -> bool:
        """Ask the TurtleBot3 /sound service for one beep. True if it was sent.

        The reply is not needed, but an unclaimed future would sit in rclpy's
        pending-request table forever, so it is discarded on completion.
        """
        if self.sound_client is None:
            return False

        try:
            if self.sound_client.service_is_ready():
                self.sound_ready = True
                request = Sound.Request()
                request.value = int(value)
                future = self.sound_client.call_async(request)
                # The reply is of no interest, but an un-reaped future stays
                # in rclpy's pending-request table forever. Over a long run
                # that leaks; drop it as soon as it completes.
                future.add_done_callback(self._discard_sound_future)
                return True
            else:
                self.sound_ready = False
        except Exception as exc:
            self.sound_ready = False
            self.get_logger().debug(f"Buzzer request failed: {exc}")
        return False

    def _discard_sound_future(self, future) -> None:
        """Drop a finished /sound request so it cannot leak memory over a long run."""
        try:
            if self.sound_client is not None and hasattr(
                self.sound_client, "remove_pending_request"
            ):
                self.sound_client.remove_pending_request(future)
        except Exception:
            pass

    def queue_buzzer_pattern(
        self,
        values: Sequence[int],
        gap_sec: float = 0.18,
    ) -> None:
        """Schedule a sequence of beeps, spaced `gap_sec` apart.

        Queueing (instead of beeping immediately) keeps patterns from stepping
        on each other and keeps the control loop from blocking on the service.
        """
        start_time = time.time()
        if self.buzzer_queue:
            start_time = max(start_time, self.buzzer_queue[-1][0] + gap_sec)

        for index, value in enumerate(values):
            self.buzzer_queue.append(
                (start_time + index * gap_sec, int(value))
            )

    def update_buzzer_queue(self) -> None:
        """Play the next queued beep if its scheduled time has arrived."""
        if self.buzzer_queue and time.time() >= self.buzzer_queue[0][0]:
            _, value = self.buzzer_queue[0]
            if self.request_sound(value):
                self.buzzer_queue.popleft()

    def play_buzzer_once_throttled(self) -> None:
        """Beep at most once per BUZZER_INTERVAL, for the steady lost-line alarm."""
        if self.sound_client is None:
            return

        now = time.time()

        if now - self.last_buzzer_time < self.BUZZER_INTERVAL:
            return

        self.last_buzzer_time = now

        try:
            if self.sound_client.service_is_ready():
                self.sound_ready = True
                request = Sound.Request()
                request.value = int(self.BUZZER_SOUND_VALUE)
                self.sound_client.call_async(request)
            else:
                self.sound_ready = False
        except Exception as exc:
            self.sound_ready = False
            self.get_logger().debug(f"Buzzer call failed: {exc}")

    def alarm_outputs(self) -> None:
        """Ordinary alarm outputs: slow red blink plus an occasional single beep."""
        self.update_blink_state()

        self.gpio_write(self.LEFT_LED, False)
        self.gpio_write(self.RIGHT_LED, False)
        self.gpio_write(self.STOP_LED, self.blink_state)

        self.play_buzzer_once_throttled()

    def fail_stop_outputs(self) -> None:
        """Use outputs that cannot be confused with ordinary line loss."""
        now = time.time()

        if (
            self.last_fail_stop_blink_time <= 0.0
            or now - self.last_fail_stop_blink_time
            >= self.FAIL_STOP_BLINK_INTERVAL
        ):
            self.fail_stop_blink_state = not self.fail_stop_blink_state
            self.last_fail_stop_blink_time = now

        self.gpio_write(self.LEFT_LED, False)
        self.gpio_write(self.RIGHT_LED, False)
        self.gpio_write(self.STOP_LED, self.fail_stop_blink_state)

        if (
            not self.buzzer_queue
            and (
                self.last_fail_stop_buzzer_time <= 0.0
                or now - self.last_fail_stop_buzzer_time
                >= self.FAIL_STOP_BUZZER_REPEAT_SEC
            )
        ):
            self.last_fail_stop_buzzer_time = now
            self.queue_buzzer_pattern(self.BEEP_FAIL_STOP, gap_sec=0.12)

        # The big card scrolls off the screen while the robot sits there
        # beeping. Every FAIL_STOP_REMINDER_SEC, print a short reminder so
        # whoever walks over can always see the problem and the reset command
        # without having to scroll back up.
        if (
            self.fail_stop_reason
            and now - self.last_fail_stop_reminder_time
            >= self.FAIL_STOP_REMINDER_SEC
        ):
            self.last_fail_stop_reminder_time = now
            problem, _, _ = self.classify_fail_stop(self.fail_stop_reason)
            waiting_sec = now - self.fail_stop_enter_time
            print(
                "\n  "
                + self.colour(
                    f" STILL IN FAIL-STOP ({problem}) ", "1;97;41"
                )
                + self.colour(
                    f"  waiting {waiting_sec:.0f} s", "2;37"
                )
                + "\n    drive it clear with "
                + self.colour("W A S D X", "1;92")
                + " in teleop, then run:\n    "
                + self.colour(
                    " " + self.RESET_FAIL_STOP_COMMAND + " ", "1;30;106"
                )
                + "\n",
                flush=True,
            )

        # The watchdog also calls this method, so the distinctive pattern can
        # continue even if Pixy USB polling is delayed.
        self.update_buzzer_queue()

    # =========================================================================
    # FAIL-STOP REPORTING
    #
    # When the robot stops itself, whoever walks over to it needs four things
    # answered fast: what broke, what to check, what the robot is doing right
    # now, and exactly what to type to get going again. The methods below
    # build that report once and render it two ways -- as a colour-coded card
    # in the terminal, and as an HTML file that can be opened in a browser.
    # =========================================================================

    # Keyword -> (short problem type, plain-English meaning, what to check).
    # The reason strings are written by the state machine; matching on a few
    # keywords turns them into something a reader can act on. Order matters:
    # the first keyword found wins, so the most specific ones go first.
    FAIL_STOP_GUIDE: List[Tuple[Tuple[str, ...], str, str, Tuple[str, ...]]] = [
        (
            ("sensor data became stale", "sensor watchdog", "unavailable"),
            "SENSOR DROPOUT",
            "The LiDAR or the odometry stopped sending fresh data, so the "
            "robot could no longer trust where it was.",
            (
                "Is the LiDAR still spinning? Listen for the motor.",
                "Check the USB / cable to the LiDAR is properly seated.",
                "Is TERMINAL 1 (robot.launch.py) still running, "
                "or did it crash?",
                "Run 'ros2 topic hz /scan' and 'ros2 topic hz /odom' "
                "-- both should be steady.",
                "Low battery can brown out the LiDAR before anything else.",
            ),
        ),
        (
            ("no reliable left/right", "no valid front board cluster",
             "cluster was lost", "could not centre the board",
             "invalid side"),
            "BOARD NOT FOUND",
            "The LiDAR could not find a board it was confident about, or it "
            "lost the one it had. The robot refused to push at a guess.",
            (
                "Is the board actually standing where it should be?",
                "Is it inside the LiDAR's usable range, and not so close "
                "that the readings come back as NaN?",
                "Is anything else (a chair leg, a bag, a person) inside the "
                "scan and confusing the search?",
                "Check the board is tall enough to sit in the LiDAR plane.",
                "Re-run with CONSOLE_STYLE=debug to see the "
                "left/right distances being voted on.",
            ),
        ),
        (
            ("reverse blocked", "rear path became unsafe"),
            "NO SPACE BEHIND",
            "The robot wanted to back up to get a better view of the board, "
            "but the LiDAR said something was behind it, so it did not move.",
            (
                "Clear the space directly behind the robot.",
                "Remember the LED support plate sits behind the LiDAR and "
                "is partly seen by it -- check REAR_OVERHANG_M is right.",
                "Give the robot more room before barcode 2.",
            ),
        ),
        (
            ("exceeded its bounded envelope",),
            "APPROACH LIMIT REACHED",
            "The robot drove the full distance (or ran the full time) it was "
            "allowed to when approaching the board, without ever arriving. "
            "It stopped rather than keep driving blindly.",
            (
                "Was the board really there, or did it get knocked earlier?",
                "Is the measured LiDAR-to-pusher distance still correct?",
                "Check APPROACH_MAX_DISTANCE_M and APPROACH_TIMEOUT_SEC "
                "against how far the board actually is.",
                "Are the wheels slipping, or is a wheel catching?",
            ),
        ),
        (
            ("insufficient progress", "timed out", "timeout"),
            "MOVE DID NOT FINISH IN TIME",
            "A turn or a drive was given a time limit and did not finish "
            "inside it. Something stopped the robot moving as expected.",
            (
                "Is a wheel stuck, blocked, or off the ground?",
                "Check the battery -- a flat battery turns slowly.",
                "Is anything physically holding the robot back?",
                "If it is only just too slow, the matching TIMEOUT_SEC "
                "constant may need a little more room.",
            ),
        ),
        (
            ("state changed unexpectedly",),
            "INTERNAL SAFETY CHECK",
            "The program noticed its own state machine had moved somewhere "
            "it did not expect while it was paused. It stopped rather than "
            "carry on from an unknown position.",
            (
                "Note down what you were doing -- this one is worth "
                "reporting, not just resetting.",
                "Re-run with CONSOLE_STYLE=debug to capture the state "
                "sequence leading up to it.",
            ),
        ),
    ]

    def classify_fail_stop(
        self,
        reason: str,
    ) -> Tuple[str, str, Sequence[str]]:
        """Turn a raw fail-stop reason into (type, meaning, things to check).

        The state machine writes fairly technical reason strings. This turns
        them into something a person standing next to the robot can act on.
        """
        lowered = reason.lower()
        for keywords, label, meaning, checks in self.FAIL_STOP_GUIDE:
            if any(keyword in lowered for keyword in keywords):
                return label, meaning, checks

        return (
            "UNEXPECTED CONDITION",
            "The robot hit a condition it is not willing to drive through. "
            "The full message below says exactly which one.",
            (
                "Read the full message and find that text in the code.",
                "Re-run with CONSOLE_STYLE=debug for the numbers behind it.",
            ),
        )

    def print_fail_stop_terminal_alert(self, reason: str) -> None:
        """Print the colour-coded fail-stop recovery card in the terminal.

        A terminal cannot render real HTML, so the colour here comes from ANSI
        escape codes -- the same mechanism that makes 'ls --color' work. The
        same report is also written to an HTML file (see write_fail_stop_html)
        for when a browser is easier to read than a scrolling terminal.

        Set NO_COLOUR=1 to get plain text, e.g. when piping into a log file.
        """
        width = 72
        problem, meaning, checks = self.classify_fail_stop(reason)

        # --- small helpers so the layout below stays readable ---------------
        def banner(text: str) -> str:
            """One full-width line of white-on-red."""
            return self.colour(text.ljust(width), "1;97;41")

        def heading(number: int, text: str) -> str:
            """A numbered section divider, e.g. '-- 2. WHAT TO CHECK ------'."""
            label = f"-- {number}. {text} "
            return self.colour(label.ljust(width, "-"), "1;93")

        def wrap(text: str, indent: str, hanging: str = "") -> str:
            """Wrap long text to the card width so nothing runs off screen.

            `hanging` is the indent used for the 2nd and later lines, so a
            numbered point stays lined up under its own text instead of
            under the number.
            """
            follow = hanging or indent
            words = text.split()
            lines: List[str] = []
            current = indent
            for word in words:
                if len(current) + len(word) + 1 > width and current.strip():
                    lines.append(current)
                    current = follow + word
                elif current in (indent, follow):
                    current = current + word
                else:
                    current = current + " " + word
            if current.strip():
                lines.append(current)
            return "\n".join(lines)

        parts: List[str] = ["\n\n"]

        # --- header ---------------------------------------------------------
        parts.append(banner(""))
        parts.append("\n")
        parts.append(banner("   FAIL-STOP  --  THE ROBOT HAS STOPPED ITSELF"))
        parts.append("\n")
        parts.append(banner(""))
        parts.append(
            f"\n  fault #{self.fail_stop_count} at {self.fail_stop_time_text}\n"
        )

        # --- 1. what went wrong ---------------------------------------------
        parts.append("\n" + heading(1, "WHAT WENT WRONG") + "\n\n")
        parts.append(
            f"     Problem      : {self.colour(problem, '1;91')}\n"
            f"     While it was : {self.fail_stop_state_at_fault}\n"
            f"     Position     : {self.fail_stop_pose}\n\n"
        )
        parts.append(wrap(meaning, "     ") + "\n\n")
        parts.append(f"     {self.colour('Full message from the code:', '2;37')}\n")
        parts.append(wrap(reason, "       ") + "\n")

        # --- 2. what to check ------------------------------------------------
        parts.append("\n" + heading(2, "WHAT TO CHECK FIRST") + "\n\n")
        for index, check in enumerate(checks, start=1):
            parts.append(
                wrap(f"{index}. {check}", "     ", hanging="        ") + "\n"
            )

        # --- 3. current outputs ----------------------------------------------
        parts.append("\n" + heading(3, "WHAT THE ROBOT IS DOING RIGHT NOW") + "\n\n")
        parts.append(
            "     [ STOP ]  wheels stopped, actuator signal released\n"
            "     [ LED  ]  red LED flashing FAST\n"
            "               (a SLOW flash is only a lost line, not this)\n"
            "     [ BEEP ]  repeating rise-and-fall siren\n"
            f"     [ FREE ]  /cmd_vel released after "
            f"{self.FAIL_STOP_COMMAND_HOLD_SEC:.1f} s so you can drive it\n"
        )

        # --- 4. manual recovery ----------------------------------------------
        parts.append("\n" + heading(4, "HOW TO RECOVER (drive it yourself)") + "\n\n")
        parts.append(
            "     STEP 1   In TERMINAL 2, start keyboard control:\n"
            f"                {self.colour('ros2 run turtlebot3_teleop teleop_keyboard', '1;96')}\n"
            "\n"
            "     STEP 2   Drive it with the "
            + self.colour("W A S D X", "1;92")
            + " keys.\n"
            "              "
            + self.colour("The arrow keys do NOT work.", "1;91")
            + "\n\n"
            "                          "
            + self.colour("[ W ]", "1;92")
            + "   forward\n"
            "                  "
            + self.colour("[ A ]", "1;92")
            + "   "
            + self.colour("[ S ]", "1;92")
            + "   "
            + self.colour("[ D ]", "1;92")
            + "   A = turn left,  D = turn right\n"
            "                          "
            + self.colour("[ X ]", "1;92")
            + "   reverse\n"
            "\n"
            "                  "
            + self.colour("[ S ]", "1;92")
            + " or "
            + self.colour("[ SPACE ]", "1;92")
            + "  =  STOP\n"
            "\n"
            "              Careful: each press changes the target SPEED and\n"
            "              the robot keeps rolling. It does not stop when you\n"
            "              let go. Tap S or SPACE to bring it to a halt.\n"
            "\n"
            "     STEP 3   Put the robot back on the line, facing the way it\n"
            "              was travelling.\n"
            "\n"
            "     STEP 4   Press Ctrl-C in TERMINAL 2 to close teleop.\n"
            "              (If you leave it running it will fight this\n"
            "              program for control of /cmd_vel.)\n"
            "\n"
            "     STEP 5   In TERMINAL 3, run the reset command below.\n"
        )

        # --- 5. reset command --------------------------------------------
        parts.append("\n" + heading(5, "RESET COMMAND (copy the whole line)") + "\n\n")
        parts.append(
            "       "
            + self.colour(" " + self.RESET_FAIL_STOP_COMMAND + " ", "1;30;106")
            + "\n\n"
            "     After the reset the robot waits until the Pixy sees the line\n"
            "     for a few frames, then carries on by itself.\n"
        )

        if self.fail_stop_html_written_path:
            parts.append(
                "\n     Full colour report saved to:\n       "
                + self.colour(self.fail_stop_html_written_path, "4;96")
                + "\n"
            )

        parts.append(self.colour("-" * width, "1;91") + "\n")

        print("".join(parts), flush=True)

    def write_fail_stop_html(self, reason: str) -> None:
        """Save the same fail-stop report as an HTML file for a browser.

        The terminal is fine while you are standing at the laptop, but the
        card scrolls away and the colours cannot be screenshotted neatly for
        a report. This writes the identical information as a styled page.
        Turn it off with FAIL_STOP_HTML=0.
        """
        self.fail_stop_html_written_path = ""
        if not self.WRITE_FAIL_STOP_HTML:
            return

        def esc(text: str) -> str:
            """Escape the few characters that would break the HTML."""
            return (
                str(text)
                .replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;")
                .replace('"', "&quot;")
            )

        problem, meaning, checks = self.classify_fail_stop(reason)
        checklist = "\n".join(
            f"      <li>{esc(item)}</li>" for item in checks
        )
        stamp = time.strftime(
            "%d %b %Y, %H:%M:%S", time.localtime(self.fail_stop_enter_time)
        )

        page = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>TurtleBot3 fail-stop report</title>
<style>
  body {{ background:#111418; color:#e6e6e6; margin:0; padding:28px;
         font-family:"DejaVu Sans",Arial,sans-serif; line-height:1.55; }}
  .card {{ max-width:820px; margin:0 auto; background:#1b1f26;
           border-radius:10px; overflow:hidden;
           box-shadow:0 6px 24px rgba(0,0,0,.5); }}
  .top {{ background:#b3261e; color:#fff; padding:18px 26px; }}
  .top h1 {{ margin:0; font-size:21px; letter-spacing:.5px; }}
  .top p  {{ margin:6px 0 0; font-size:13px; opacity:.9; }}
  .body {{ padding:8px 26px 26px; }}
  h2 {{ color:#ffcf4d; font-size:14px; letter-spacing:1.5px;
        text-transform:uppercase; margin:26px 0 10px;
        border-bottom:1px solid #333a44; padding-bottom:6px; }}
  table {{ border-collapse:collapse; font-size:14px; }}
  td {{ padding:3px 16px 3px 0; vertical-align:top; }}
  td.k {{ color:#9aa4b2; white-space:nowrap; }}
  .tag {{ display:inline-block; background:#b3261e; color:#fff;
          padding:2px 10px; border-radius:11px; font-size:13px;
          font-weight:bold; }}
  .raw {{ background:#0e1116; border-left:3px solid #b3261e;
          padding:10px 14px; font-family:"DejaVu Sans Mono",monospace;
          font-size:13px; color:#ffb4ad; border-radius:0 5px 5px 0; }}
  ol, ul {{ margin:8px 0; padding-left:24px; }}
  li {{ margin:5px 0; }}
  code {{ background:#0e1116; padding:2px 7px; border-radius:4px;
          font-family:"DejaVu Sans Mono",monospace; font-size:13px;
          color:#7fd8ff; }}
  .cmd {{ display:block; background:#0e1116; color:#7fd8ff;
          padding:13px 16px; border-radius:6px; margin:10px 0;
          font-family:"DejaVu Sans Mono",monospace; font-size:13px;
          word-break:break-all; border:1px solid #2b3542; }}
  .keys {{ font-family:"DejaVu Sans Mono",monospace; font-size:15px;
           line-height:2; text-align:center; background:#0e1116;
           padding:16px; border-radius:6px; margin:12px 0; }}
  .key {{ display:inline-block; background:#2b3542; color:#9dffb0;
          border:1px solid #46525f; border-bottom-width:3px;
          border-radius:5px; padding:3px 11px; margin:0 3px;
          font-weight:bold; }}
  .warn {{ color:#ff8f85; font-weight:bold; }}
  .now li {{ list-style:none; }}
  .now b {{ color:#9dffb0; }}
  .foot {{ color:#7d8794; font-size:12px; margin-top:26px;
           border-top:1px solid #333a44; padding-top:12px; }}
</style>
</head>
<body>
<div class="card">

  <div class="top">
    <h1>FAIL-STOP &mdash; the robot has stopped itself</h1>
    <p>Fault #{self.fail_stop_count} &middot; {esc(stamp)}</p>
  </div>

  <div class="body">

    <h2>1. What went wrong</h2>
    <table>
      <tr><td class="k">Problem</td>
          <td><span class="tag">{esc(problem)}</span></td></tr>
      <tr><td class="k">While it was</td>
          <td>{esc(self.fail_stop_state_at_fault)}</td></tr>
      <tr><td class="k">Position</td>
          <td>{esc(self.fail_stop_pose)}</td></tr>
    </table>
    <p>{esc(meaning)}</p>
    <p class="raw">{esc(reason)}</p>

    <h2>2. What to check first</h2>
    <ol>
{checklist}
    </ol>

    <h2>3. What the robot is doing right now</h2>
    <ul class="now">
      <li><b>STOP</b> &nbsp;wheels stopped, actuator signal released</li>
      <li><b>LED</b> &nbsp;&nbsp;red LED flashing <b>fast</b>
          (a slow flash is only a lost line, not this)</li>
      <li><b>BEEP</b> &nbsp;repeating rise-and-fall siren</li>
      <li><b>FREE</b> &nbsp;<code>/cmd_vel</code> released after
          {self.FAIL_STOP_COMMAND_HOLD_SEC:.1f}&nbsp;s so you can drive it</li>
    </ul>

    <h2>4. How to recover (drive it yourself)</h2>
    <ol>
      <li>In <b>TERMINAL 2</b>, start keyboard control:
        <span class="cmd">ros2 run turtlebot3_teleop teleop_keyboard</span>
      </li>
      <li>Drive it with the <b>W A S D X</b> keys.
        <span class="warn">The arrow keys do NOT work.</span>
        <div class="keys">
          <span class="key">W</span> forward<br>
          <span class="key">A</span>
          <span class="key">S</span>
          <span class="key">D</span>
          &nbsp; A = turn left, D = turn right<br>
          <span class="key">X</span> reverse<br>
          <span class="key">S</span> or <span class="key">SPACE</span>
          = STOP
        </div>
        Each press changes the target <b>speed</b> and the robot keeps
        rolling &mdash; it does not stop when you let go. Tap
        <span class="key">S</span> or <span class="key">SPACE</span>
        to bring it to a halt.
      </li>
      <li>Put the robot back on the line, facing the way it was
          travelling.</li>
      <li>Press <code>Ctrl-C</code> in TERMINAL 2 to close teleop. If you
          leave it running it will fight this program for control of
          <code>/cmd_vel</code>.</li>
      <li>In <b>TERMINAL 3</b>, run the reset command below.</li>
    </ol>

    <h2>5. Reset command</h2>
    <span class="cmd">{esc(self.RESET_FAIL_STOP_COMMAND)}</span>
    <p>After the reset the robot waits until the Pixy sees the line for a
       few frames, then carries on by itself.</p>

    <p class="foot">
      Written automatically by the controller. Set
      <code>FAIL_STOP_HTML=0</code> to stop generating this file.
    </p>

  </div>
</div>
</body>
</html>
"""

        try:
            with open(self.FAIL_STOP_HTML_PATH, "w", encoding="utf-8") as page_file:
                page_file.write(page)
            self.fail_stop_html_written_path = self.FAIL_STOP_HTML_PATH
        except Exception as exc:
            # Never let a file-writing problem stop the robot from stopping.
            self.get_logger().debug(f"Could not write fail-stop HTML: {exc}")

    # =========================================================================
    # PIXY READING
    # =========================================================================

    def read_pixy_counts(self) -> Tuple[int, int, int]:
        """Poll the Pixy2 once and return (vectors, barcodes, intersections) counts.

        line_get_all_features() must be called first: it grabs one frame, and
        the three getters then read that same frame.
        """
        if not self.pixy_ready:
            return 0, 0, 0

        try:
            line_get_all_features()

            vector_count = line_get_vectors(100, self.vectors)
            barcode_count = line_get_barcodes(10, self.barcodes)
            intersection_count = line_get_intersections(100, self.intersections)

            return (
                max(0, int(vector_count)),
                max(0, int(barcode_count)),
                max(0, int(intersection_count)),
            )
        except Exception as exc:
            self.get_logger().warning(f"Pixy read failed: {exc}")
            return 0, 0, 0

    @staticmethod
    def make_vector_info(vec, index: int) -> VectorInfo:
        """Turn one raw Pixy vector into a VectorInfo with tidy geometry.

        Pixy image coordinates put y=0 at the TOP, so the point with the larger
        y is the end nearest the robot. We always store that as (bottom_x,
        bottom_y) so 'bottom' consistently means 'closest to the robot'.
        """
        x0 = float(int(vec.m_x0))
        y0 = float(int(vec.m_y0))
        x1 = float(int(vec.m_x1))
        y1 = float(int(vec.m_y1))
        flags = int(getattr(vec, "m_flags", 0))

        if y0 >= y1:
            bottom_x, bottom_y = x0, y0
            top_x, top_y = x1, y1
        else:
            bottom_x, bottom_y = x1, y1
            top_x, top_y = x0, y0

        dx = x1 - x0
        dy = y1 - y0

        return VectorInfo(
            index=index,
            x0=x0,
            y0=y0,
            x1=x1,
            y1=y1,
            bottom_x=bottom_x,
            bottom_y=bottom_y,
            top_x=top_x,
            top_y=top_y,
            center_x=0.5 * (x0 + x1),
            center_y=0.5 * (y0 + y1),
            length=math.hypot(dx, dy),
            y_span=abs(dy),
            x_span=abs(dx),
            angle_deg=math.degrees(math.atan2(dy, dx)),
            flags=flags,
        )

    def get_reliable_vectors(self, vector_count: int) -> List[VectorInfo]:
        """Filter the raw Pixy vectors down to ones long and sensible enough to use."""
        reliable: List[VectorInfo] = []

        for i in range(vector_count):
            try:
                vector = self.make_vector_info(self.vectors[i], i)
            except Exception:
                continue

            if vector.length < self.MIN_VECTOR_LENGTH_PX:
                continue

            reliable.append(vector)

        if self.DEBUG_VECTOR_LOG and reliable:
            text = " | ".join(
                (
                    f"#{v.index}:({v.x0:.0f},{v.y0:.0f})->"
                    f"({v.x1:.0f},{v.y1:.0f}), "
                    f"len={v.length:.1f}, flag={v.flags}"
                )
                for v in reliable
            )
            self.get_logger().info(f"Pixy vectors: {text}")

        return reliable

    def choose_best_follow_vector(
        self,
        vectors: List[VectorInfo],
    ) -> Tuple[Optional[VectorInfo], Optional[float]]:
        """Pick the vector to steer on and return (vector, pixel error).

        Preference goes to a long, upright vector starting near the bottom of
        the frame: that is the line the robot is actually sitting on. The error
        is how far that line is from the image centre, in pixels.
        """
        if not vectors:
            return None, None

        scored = []

        for vector in vectors:
            if vector.y_span < self.MIN_FOLLOW_Y_SPAN_PX:
                continue

            if vector.bottom_y < self.MIN_BOTTOM_Y_PX:
                continue

            follow_x = vector.x_at_y(self.TRACK_Y)
            error = follow_x - self.FRAME_CENTER_X

            if abs(error) > self.MAX_ACCEPTED_ERROR_PX:
                continue

            continuity_score = 0.0

            if self.last_good_line_x is not None:
                jump = abs(follow_x - self.last_good_line_x)

                if jump > self.MAX_LINE_X_JUMP_PX and len(vectors) == 1:
                    continue

                continuity_score = -0.45 * jump

            score = (
                2.0 * vector.bottom_y
                + 1.4 * vector.y_span
                + 0.6 * vector.length
                - 0.35 * abs(error)
                + continuity_score
            )

            scored.append((score, vector, error))

        if not scored:
            return None, None

        scored.sort(key=lambda item: item[0], reverse=True)
        return scored[0][1], scored[0][2]

    @staticmethod
    def angle_difference_deg(a: float, b: float) -> float:
        """Smallest angle between two undirected lines, 0..90 degrees.

        Undirected: a line at 10 deg and one at 190 deg are the same line, so
        both angles are folded into 0..180 first.
        """
        a %= 180.0
        b %= 180.0

        difference = abs(a - b)
        return min(difference, 180.0 - difference)

    @staticmethod
    def line_intersection(
        v1: VectorInfo,
        v2: VectorInfo,
    ) -> Optional[Tuple[float, float]]:
        """Where two infinite lines cross, or None if they are (nearly) parallel."""
        x1, y1, x2, y2 = v1.x0, v1.y0, v1.x1, v1.y1
        x3, y3, x4, y4 = v2.x0, v2.y0, v2.x1, v2.y1

        denominator = (
            (x1 - x2) * (y3 - y4)
            - (y1 - y2) * (x3 - x4)
        )

        if abs(denominator) < 1e-6:
            return None

        px = (
            (x1 * y2 - y1 * x2) * (x3 - x4)
            - (x1 - x2) * (x3 * y4 - y3 * x4)
        ) / denominator

        py = (
            (x1 * y2 - y1 * x2) * (y3 - y4)
            - (y1 - y2) * (x3 * y4 - y3 * x4)
        ) / denominator

        return px, py

    @staticmethod
    def point_segment_distance(
        px: float,
        py: float,
        vector: VectorInfo,
    ) -> float:
        """Shortest distance from a point to a vector treated as a line SEGMENT.

        Segment, not infinite line: this is how we check that a computed
        crossing point actually lies on the painted tape and is not a phantom
        far off the end of it.
        """
        ax, ay = vector.x0, vector.y0
        bx, by = vector.x1, vector.y1

        dx = bx - ax
        dy = by - ay
        denominator = dx * dx + dy * dy

        if denominator < 1e-6:
            return math.hypot(px - ax, py - ay)

        t = ((px - ax) * dx + (py - ay) * dy) / denominator
        t = max(0.0, min(1.0, t))

        closest_x = ax + t * dx
        closest_y = ay + t * dy

        return math.hypot(px - closest_x, py - closest_y)

    def crossing_from_vectors(
        self,
        vectors: List[VectorInfo],
    ) -> Tuple[bool, Optional[Tuple[float, float]], str]:
        """Decide whether the visible vectors really form a junction.

        Returns (is_crossing, crossing_point, reason). Two vectors must meet at
        a wide enough angle, cross inside the frame, and the crossing point must
        sit close to both segments. The reason string says why it was rejected,
        which is what makes junction problems debuggable.
        """
        if len(vectors) < 2:
            return False, None, "no_multi_vector"

        for i in range(len(vectors)):
            for j in range(i + 1, len(vectors)):
                first = vectors[i]
                second = vectors[j]

                angle_difference = self.angle_difference_deg(
                    first.angle_deg,
                    second.angle_deg,
                )

                if angle_difference < self.MIN_CROSSING_ANGLE_DEG:
                    continue

                point = self.line_intersection(first, second)

                if point is None:
                    continue

                px, py = point

                d1 = self.point_segment_distance(px, py, first)
                d2 = self.point_segment_distance(px, py, second)

                if (
                    d1 > self.CROSSING_SEGMENT_TOL_PX
                    or d2 > self.CROSSING_SEGMENT_TOL_PX
                ):
                    continue

                if abs(px - self.FRAME_CENTER_X) > self.INTERSECTION_CENTER_X_TOL_PX:
                    continue

                if not (
                    self.INTERSECTION_TRIGGER_Y_MIN
                    <= py
                    <= self.INTERSECTION_TRIGGER_Y_MAX
                ):
                    continue

                reason = (
                    f"vector_cross angle={angle_difference:.1f} "
                    f"point=({px:.1f},{py:.1f})"
                )
                return True, point, reason

        return False, None, "no_crossing"

    def read_pixy_frame(self) -> PixyFrame:
        """Read one complete Pixy2 frame and package it as a PixyFrame.

        This is the robot's only source of line, junction and barcode data. It
        is deliberately kept OUTSIDE the state lock, because the USB read can
        block and the watchdog must stay free to stop the wheels.
        """
        vector_count, barcode_count, intersection_count = self.read_pixy_counts()

        vectors = self.get_reliable_vectors(vector_count)
        best_vector, error = self.choose_best_follow_vector(vectors)

        raw_intersection = False
        crossing_point = None
        reason = "none"

        now = time.time()

        if now - self.last_intersection_time < self.INTERSECTION_COOLDOWN_SEC:
            reason = "cooldown"
        else:
            if intersection_count > 0:
                raw_intersection = True
                reason = "pixy_intersection_array"

            if (
                not raw_intersection
                and best_vector is not None
                and (best_vector.flags & 0x04)
            ):
                raw_intersection = True
                reason = "pixy_vector_flag"

            if not raw_intersection:
                raw_intersection, crossing_point, reason = (
                    self.crossing_from_vectors(vectors)
                )

        return PixyFrame(
            vector_count=vector_count,
            barcode_count=barcode_count,
            intersection_count=intersection_count,
            reliable_vectors=vectors,
            best_vector=best_vector,
            line_error=error,
            raw_intersection=raw_intersection,
            crossing_point=crossing_point,
            intersection_reason=reason,
        )

    # =========================================================================
    # STABILITY AND BARCODE MEMORY
    # =========================================================================

    def update_line_stability(self, frame: PixyFrame) -> None:
        """Track how many frames in a row the line has been missing."""
        if frame.best_vector is not None and frame.line_error is not None:
            self.line_lost_count = 0
            self.last_good_line_x = frame.line_error + self.FRAME_CENTER_X
        else:
            self.line_lost_count += 1

    def update_intersection_stability(self, raw: bool) -> bool:
        """Require INTERSECTION_CONFIRM_FRAMES frames in a row before believing a junction.

        One noisy frame should never trigger a turn, so a junction has to be
        seen repeatedly before it counts.
        """
        if raw:
            self.intersection_stable_count += 1
        else:
            self.intersection_stable_count = 0

        return (
            self.intersection_stable_count
            >= self.INTERSECTION_CONFIRM_FRAMES
        )

    def reset_barcode_vote(self) -> None:
        """Throw away the in-progress barcode vote and start again."""
        self.barcode_vote_codes.clear()
        self.barcode_vote_start_time = 0.0

    def barcode_vote_in_progress(self) -> bool:
        """True while barcode reads are being collected but not yet decided."""
        return bool(self.barcode_vote_codes)

    def clear_command_queue(self) -> None:
        """Drop every stored command (used by resets and fail-stop)."""
        self.pending_command = None
        self.queued_commands.clear()

    def latch_barcode(self, code: Optional[int]) -> None:
        """Latch a code and remember where the robot was when it did."""
        self.barcode_latched_code = code
        if code is None:
            self.barcode_latch_pose_valid = False
            return
        self.barcode_latch_x = self.current_x
        self.barcode_latch_y = self.current_y
        self.barcode_latch_pose_valid = self.odom_is_fresh()

    def distance_from_barcode_latch(self) -> float:
        """Metres driven since the last barcode was latched (0 if unknown).

        This is what makes the barcode 2 lockout spatial rather than time-based:
        the robot must physically move on before the same barcode can fire again.
        """
        if not self.barcode_latch_pose_valid or not self.odom_is_fresh():
            return 0.0
        return math.hypot(
            self.current_x - self.barcode_latch_x,
            self.current_y - self.barcode_latch_y,
        )

    def promote_queued_command(self) -> None:
        """Move the next stored command into the active slot after a turn."""
        if self.pending_command is not None or not self.queued_commands:
            return
        self.pending_command = self.queued_commands.popleft()
        self.say(
            f"Next stored command is now active: {self.pending_command} "
            f"({len(self.queued_commands)} still queued)."
        )

    def accept_barcode_vote(
        self,
        detected_code: int,
        votes: Sequence[int],
        now: float,
    ) -> None:
        # The cooldown exists to stop ONE physical barcode being accepted
        # twice. A repeat of the same code is held off; a different code is
        # a different barcode, so it is accepted straight away. This is what
        # allows two barcodes only a few centimetres apart to both register.
        """Final gate before a voted barcode becomes a real command.

        Applies the cooldowns and the barcode-2 spatial lockout. Returns True
        only if the code is genuinely new; this is what stops one physical
        barcode being executed twice as the robot rolls over it.
        """
        same_as_last = detected_code == self.last_command_source_code
        cooldown = (
            self.BARCODE_ACCEPT_COOLDOWN
            if same_as_last
            else self.BARCODE_ACCEPT_COOLDOWN_DIFFERENT
        )
        if cooldown > 0.0 and now - self.last_barcode_accept_time < cooldown:
            self.detail(
                f"Barcode vote {list(votes)} won by {detected_code}, but the "
                f"{cooldown:.2f} s same-code accept cooldown is still active."
            )
            return

        reacquiring_after_obstacle = (
            self.current_state == RobotState.OBSTACLE_REACQUIRE_LINE
        )
        reacquiring_after_turn = (
            self.current_state == RobotState.SNAP_BACK_TO_LINE
        )
        command = self.BARCODE_COMMANDS[detected_code]
        self.latch_barcode(detected_code)
        self.last_barcode_accept_time = now
        self.last_command_source_code = detected_code
        self.queue_buzzer_pattern(self.BEEP_BARCODE[detected_code])

        if command == "OBSTACLE":
            self.say(
                f"Barcode vote {list(votes)} confirmed Barcode 2. Stopping "
                "and starting the five-scan obstacle routine."
            )
            self.begin_obstacle_routine()
            return

        if command == "STOP":
            self.clear_command_queue()
            self.current_state = RobotState.STOP_COMMAND
            self.say(
                f"Barcode vote {list(votes)} confirmed Barcode 3. Stopping "
                "immediately."
            )
            return

        # A command read while another one is still waiting for its junction
        # is now QUEUED rather than discarded.
        if self.pending_command is None:
            self.pending_command = command
            queue_note = ""
        elif len(self.queued_commands) < max(0, self.MAX_QUEUED_COMMANDS - 1):
            self.queued_commands.append(command)
            queue_note = (
                f" Queued behind {self.pending_command} "
                f"({len(self.queued_commands)} waiting)."
            )
        else:
            self.warn(
                f"Barcode {detected_code} ({command}) could not be stored: "
                f"already holding {self.pending_command} plus "
                f"{len(self.queued_commands)} queued (limit "
                f"{self.MAX_QUEUED_COMMANDS}). Space the barcodes further "
                "apart or raise MAX_QUEUED_COMMANDS."
            )
            return

        if not (reacquiring_after_obstacle or reacquiring_after_turn):
            self.current_state = RobotState.LINE_FOLLOW_ARMED

        if reacquiring_after_obstacle:
            state_note = "Stored while finishing obstacle line recovery."
        elif reacquiring_after_turn:
            state_note = "Stored while centring the outgoing line after the turn."
        else:
            state_note = "Waiting for the next confirmed intersection."

        self.say(
            f"Barcode vote {list(votes)} confirmed Barcode {detected_code}: "
            f"stored {command}. {state_note}{queue_note}"
        )

    def required_barcode_votes(self, code: int) -> int:
        """How many identical reads this code needs before it is trusted.

        Normally a 2-of-3 majority. Inside the just-used Barcode 2 zone a
        non-2 code must be unanimous, because a cropped view of Barcode 2
        can decode as 0, 1 or 3.
        """
        if self.barcode_strict_vote_active and code != 2:
            return max(self.BARCODE_MIN_VOTES, self.BARCODE2_ZONE_MIN_VOTES)
        return self.BARCODE_MIN_VOTES

    def finalise_barcode_vote(self, now: float) -> None:
        """Close a barcode vote and accept the winner only on a clear majority.

        A code needs BARCODE_MIN_VOTES out of BARCODE_VOTE_SAMPLES reads and
        must beat the runner-up. Ties are discarded: doing nothing is far safer
        than turning the wrong way.
        """
        votes = list(self.barcode_vote_codes)
        self.reset_barcode_vote()
        if not votes:
            return

        counts = Counter(votes)
        winner, winner_count = counts.most_common(1)[0]
        runner_up_count = max(
            (count for code, count in counts.items() if code != winner),
            default=0,
        )

        required = self.required_barcode_votes(winner)

        if winner_count >= required and winner_count > runner_up_count:
            self.accept_barcode_vote(winner, votes, now)
            return

        self.warn(
            f"Barcode vote rejected: reads={votes}, counts={dict(counts)}; "
            f"need a strict {required}-vote majority"
            + (
                " (raised because the robot is still beside the just-used "
                "Barcode 2)."
                if required != self.BARCODE_MIN_VOTES
                else "."
            )
        )

    def update_barcode_memory(self, barcode_count: int) -> None:
        """Use up to three valid Pixy reads while preserving physical latches."""
        barcode_accept_states = (
            RobotState.LINE_FOLLOW,
            RobotState.LINE_FOLLOW_ARMED,
            RobotState.SNAP_BACK_TO_LINE,
            RobotState.OBSTACLE_REACQUIRE_LINE,
        )
        barcode_latch_update_states = barcode_accept_states + (
            RobotState.MOVE_TO_TURN_CENTER,
            RobotState.EXECUTE_TURN_ODOM,
        )

        if self.current_state not in barcode_latch_update_states:
            self.reset_barcode_vote()
            return

        detected_codes: List[int] = []
        if barcode_count > 0:
            for i in range(barcode_count):
                try:
                    code = int(self.barcodes[i].m_code)
                except Exception:
                    continue
                if code in self.BARCODE_COMMANDS and code not in detected_codes:
                    detected_codes.append(code)

        # Barcode 2 remains suppressed until it is both visually clear and the
        # robot has moved at least BARCODE2_REARM_DISTANCE_M away. Other codes
        # stay READABLE, but under a stricter vote (see below).
        barcode2_visible = 2 in detected_codes
        self.barcode_strict_vote_active = False
        if self.barcode2_rearm_required:
            # Require a COMPLETELY empty barcode frame to count as "clear".
            # A frame that decodes as 0/1/3 while parked on Barcode 2 is
            # quite possibly a misread of Barcode 2, so it must not be
            # allowed to count towards re-arming.
            if detected_codes:
                self.barcode2_clear_frames = 0
            else:
                self.barcode2_clear_frames += 1

            moved_from_barcode2 = (
                self.distance_from_obstacle_start()
                if self.odom_is_fresh()
                else 0.0
            )
            can_rearm_barcode2 = (
                not barcode2_visible
                and self.barcode2_clear_frames
                >= self.BARCODE2_REARM_CLEAR_FRAMES
                and moved_from_barcode2
                >= self.BARCODE2_REARM_DISTANCE_M
            )
            if can_rearm_barcode2:
                self.barcode2_rearm_required = False
                self.barcode2_clear_frames = 0
                if self.barcode_latched_code == 2:
                    self.latch_barcode(None)
                self.get_logger().info(
                    "Barcode 2 re-armed: the old barcode remained clear and "
                    f"the robot moved {moved_from_barcode2:.2f} m away."
                )
            else:
                # ---------------------------------------------------------
                # Barcode 2 itself is suppressed for the whole zone. THIS is
                # what stops the robot re-reading the same Barcode 2 when it
                # comes back from the obstacle and drives off again.
                #
                # Earlier revisions also stripped codes 0/1/3 here, on the
                # grounds that they might be misreads of Barcode 2. That
                # made a genuine barcode a few centimetres past Barcode 2
                # unreadable, because it sits well inside the 0.10 m zone
                # and is long out of view by the time the robot leaves it.
                #
                # Instead the other codes stay eligible and the vote is
                # tightened to unanimous while inside the zone, so a cropped
                # view of Barcode 2 still cannot latch a bogus turn.
                # ---------------------------------------------------------
                self.barcode_strict_vote_active = True
                if barcode2_visible:
                    detected_codes = [
                        code for code in detected_codes if code != 2
                    ]
                    self.detail(
                        "ignoring the just-used Barcode 2 "
                        f"(moved {moved_from_barcode2:.2f} m of "
                        f"{self.BARCODE2_REARM_DISTANCE_M:.2f} m)",
                        throttle=1.0,
                    )

        # Barcode 2 also has its own short post-obstacle blanking window.
        # It deliberately does NOT blank the other codes any more.
        if time.time() < self.barcode2_ignore_until:
            detected_codes = [code for code in detected_codes if code != 2]

        now = time.time()
        if (
            self.barcode_vote_in_progress()
            and now - self.barcode_vote_start_time
            >= self.BARCODE_VOTE_WINDOW_SEC
        ):
            self.finalise_barcode_vote(now)
            queue_full = (
                self.pending_command is not None
                and len(self.queued_commands)
                >= max(0, self.MAX_QUEUED_COMMANDS - 1)
            )
            if (
                queue_full
                or self.current_state not in barcode_accept_states
            ):
                return

        if not detected_codes:
            self.barcode_clear_count += 1
            if self.barcode_clear_count >= self.BARCODE_CLEAR_FRAMES:
                if not (
                    self.barcode2_rearm_required
                    and self.barcode_latched_code == 2
                ):
                    self.latch_barcode(None)
            return

        self.barcode_clear_count = 0

        # The latch also expires on distance travelled. Two barcodes close
        # together may never leave a gap of BARCODE_CLEAR_FRAMES empty Pixy
        # frames, so without this the second one could never be read.
        if (
            self.barcode_latched_code is not None
            and not (
                self.barcode2_rearm_required
                and self.barcode_latched_code == 2
            )
            and self.BARCODE_LATCH_CLEAR_DISTANCE_M > 0.0
            and self.distance_from_barcode_latch()
            >= self.BARCODE_LATCH_CLEAR_DISTANCE_M
        ):
            self.detail(
                f"Un-latching barcode {self.barcode_latched_code} after "
                f"{self.distance_from_barcode_latch():.3f} m of travel.",
                throttle=1.0,
            )
            self.latch_barcode(None)

        # STOP and OBSTACLE are unambiguous safety/action candidates. LEFT and
        # RIGHT in the same Pixy frame are rejected instead of being guessed.
        if 3 in detected_codes:
            detected_code = 3
        elif 2 in detected_codes:
            detected_code = 2
        elif len(detected_codes) == 1:
            detected_code = detected_codes[0]
        else:
            self.get_logger().warning(
                f"Ambiguous Pixy barcode frame {detected_codes}; ignoring this "
                "read while keeping any existing vote window open."
            )
            return

        # During movement into/through a junction, update only the latch. The
        # outgoing-track barcode becomes eligible in SNAP_BACK_TO_LINE.
        if self.current_state not in barcode_accept_states:
            self.reset_barcode_vote()
            return
        # A command already waiting for its junction no longer blocks reading
        # the next barcode: extra commands are queued in accept_barcode_vote.
        # Only a genuinely full queue stops the vote.
        if (
            self.pending_command is not None
            and len(self.queued_commands)
            >= max(0, self.MAX_QUEUED_COMMANDS - 1)
        ):
            self.reset_barcode_vote()
            return
        if now < self.barcode_ignore_until:
            self.reset_barcode_vote()
            return
        if detected_code == self.barcode_latched_code:
            self.reset_barcode_vote()
            return

        if not self.barcode_vote_in_progress():
            self.barcode_vote_start_time = now
        self.barcode_vote_codes.append(detected_code)

        counts = Counter(self.barcode_vote_codes)
        sealed_majority = (
            counts[detected_code] >= self.required_barcode_votes(detected_code)
            and counts[detected_code]
            > max(
                (
                    count
                    for code, count in counts.items()
                    if code != detected_code
                ),
                default=0,
            )
        )
        if (
            sealed_majority
            or len(self.barcode_vote_codes) >= self.BARCODE_VOTE_SAMPLES
        ):
            self.finalise_barcode_vote(now)

    # =========================================================================
    # MOTION HELPERS
    # =========================================================================

    @staticmethod
    def stop_twist() -> Twist:
        """A Twist with all velocities zero, i.e. 'stop the wheels'."""
        command = Twist()
        command.linear.x = 0.0
        command.angular.z = 0.0
        return command

    def execute_pd_line_follow(
        self,
        twist: Twist,
        error: float,
        requested_speed: Optional[float] = None,
    ) -> None:
        """Steer with a PD controller from the Pixy pixel error.

        P term (kp * error) pulls back towards the line; D term (kd * change in
        error) damps the swing so the robot does not weave. Forward speed is
        reduced automatically when the error is large, so tight curves are taken
        slowly instead of being cut.
        """
        derivative = error - self.prev_error
        steering = self.kp * error + self.kd * derivative

        angular_z = -steering
        angular_z = max(
            min(angular_z, self.MAX_TURN_SPEED),
            -self.MAX_TURN_SPEED,
        )

        base_speed = (
            self.FORWARD_SPEED
            if requested_speed is None
            else requested_speed
        )

        # Run at the slightly faster speed on a well-centred line, but reduce
        # speed smoothly toward CURVE_FORWARD_SPEED as the error becomes large.
        slowdown_ratio = min(
            1.0,
            abs(error) / max(self.CURVE_FULL_SLOW_ERROR_PX, 1.0),
        )
        adaptive_speed = (
            base_speed
            - slowdown_ratio
            * max(0.0, base_speed - self.CURVE_FORWARD_SPEED)
        )

        twist.linear.x = adaptive_speed
        twist.angular.z = angular_z
        self.prev_error = error

    def publish_motion(self, twist: Twist, alarm: bool = False) -> None:
        # This final gate prevents a concurrent callback from publishing an
        # old non-zero command after the watchdog or fail-stop has taken over.
        """The ONLY place /cmd_vel is published, plus the matching LED/buzzer output.

        Funnelling every command through one method means the safety overrides
        (sensor pause, fail-stop, barcode vote hold) cannot be bypassed by
        accident somewhere else in the state machine.
        """
        fail_stop_active = (
            self.current_state == RobotState.OBSTACLE_FAIL_STOP
        )
        if (
            self.sensor_pause_active
            or fail_stop_active
        ):
            twist = self.stop_twist()
            alarm = not fail_stop_active
        elif self.barcode_vote_in_progress():
            # A possible STOP/OBSTACLE barcode holds position while the vote
            # completes. LEFT/RIGHT voting only slows forward travel so the
            # remaining reads are more likely to stay in view.
            if any(code in (2, 3) for code in self.barcode_vote_codes):
                twist = self.stop_twist()
            elif twist.linear.x > self.BARCODE_VOTE_SPEED:
                twist.linear.x = self.BARCODE_VOTE_SPEED

        if fail_stop_active:
            self.fail_stop_outputs()
        elif alarm:
            self.alarm_outputs()
        else:
            self.update_leds_from_twist(twist)

        try:
            if rclpy.ok():
                self.cmd_vel_pub.publish(twist)
        except Exception:
            pass

    def enter_line_lost_alarm(self) -> None:
        """Stop and raise the ordinary lost-line alarm (recoverable, not a fail-stop)."""
        if self.current_state != RobotState.LINE_LOST_ALARM:
            self.get_logger().warning(
                "Line lost. Robot stopped; stop LED and buzzer alarm active."
            )

        self.current_state = RobotState.LINE_LOST_ALARM
        self.prev_error = 0.0
        self.reacquire_count = 0
        self.queue_buzzer_pattern(self.BEEP_FAILURE)

    def begin_obstacle_routine(self) -> None:
        """Start the board-toppling routine after barcode 2 has been confirmed.

        Saves the current pose so the robot can drive back to exactly here
        afterwards, and arms the spatial lockout that stops the same barcode 2
        being read again on the way out.
        """
        if not self.odom_is_fresh():
            self.enter_fail_stop(
                "Barcode 2 was confirmed, but fresh odometry was unavailable "
                "for saving the obstacle start pose."
            )
            return

        # Barcode 2 has already been consumed. Keep a dedicated spatial
        # lockout so the same physical barcode cannot retrigger after return.
        self.clear_command_queue()
        self.last_command_source_code = None
        self.latch_barcode(2)
        self.barcode2_rearm_required = True
        self.barcode2_clear_frames = 0

        self.obstacle_start_x = self.current_x
        self.obstacle_start_y = self.current_y
        self.obstacle_start_yaw = self.current_yaw
        self.obstacle_facing_yaw = self.current_yaw

        self.set_pulse_ms(self.ACTUATOR_RETRACT_PULSE_MS)
        self.obstacle_brake_start_time = 0.0
        self.actuator_extend_start_time = 0.0
        self.tackle_start_time = 0.0
        self.tackle_start_x = self.current_x
        self.tackle_start_y = self.current_y
        self.tackle_board_down_stable_count = 0
        self.tackle_contact_hold_start_time = 0.0
        self.tackle_stop_reason = ""
        self.tackle_required_push_m = 0.0
        self.tackle_board_distance_m = float("inf")
        self.tackle_last_push_distance = 0.0
        self.tackle_stall_count = 0
        self.tackle_contact_detected = False
        self.return_start_time = 0.0
        self.obstacle_turn_start_time = 0.0
        self.front_nan_reverse_start_time = 0.0
        self.approach_start_time = 0.0
        self.approach_aim_retry_count = 0
        self.reverse_line_seen_count = 0
        self.turn_direction = None
        self.side_scan_attempt = 1
        self.initial_scan_settle_start_time = time.time()
        self.side_scan_votes.clear()
        self.nan_side_votes.clear()
        self.clear_clearance_histories()
        self.opposite_side_clearance_m = float("inf")
        self.opposite_side_nan_ratio = 0.0
        self.rear_self_distance_baseline_m = float("inf")
        self.rear_self_nan_baseline = 0.0
        self.left_nan_ratio = 0.0
        self.right_nan_ratio = 0.0
        self.front_nan_ratio = 0.0
        self.rear_nan_ratio = 0.0
        self.front_nan_stable_count = 0
        self.front_nan_reverse_attempted = False
        self.front_nan_reverse_used = False
        self.front_nan_reverse_start_x = self.current_x
        self.front_nan_reverse_start_y = self.current_y
        self.front_nan_reverse_heading_yaw = self.current_yaw
        self.close_board_indicator_active = False
        self.front_align_stable_count = 0
        self.front_lost_count = 0
        self.front_board_target = None
        self.obstacle_turn_target_yaw = self.current_yaw
        self.sensor_pause_active = False
        self.sensor_pause_start_time = 0.0
        self.sensor_pause_state = None
        self.sensor_pause_missing = tuple()
        self.sensor_healthy_count = 0
        self.sensor_recovery_count = 0
        self.current_state = RobotState.OBSTACLE_SCAN_SETTLE

        # Stop immediately before the next LiDAR scan advances the routine.
        self.publish_motion(self.stop_twist(), alarm=False)
        self.say(
            "Obstacle routine started immediately. Saved current odometry "
            "position and waiting briefly for the robot to settle."
        )

    def finish_obstacle_routine(self) -> None:
        """Tidy up after the board routine and hand control back to line following."""
        self.stop_actuator_signal()
        self.close_board_indicator_active = False

        self.clear_command_queue()
        self.last_command_source_code = None
        # Preserve turn_direction while reacquiring the line so the search
        # direction can be chosen correctly. Clear it after line acquisition.
        self.prev_error = 0.0
        self.reacquire_count = 0
        self.last_intersection_time = time.time()

        # Blank BARCODE 2 ONLY for a moment. Blanking every code here used to
        # make a different barcode a few centimetres past Barcode 2
        # unreadable, because the robot drives through it during the window.
        self.barcode2_ignore_until = (
            time.time() + self.POST_OBSTACLE_BARCODE_IGNORE_SEC
        )

        # Do not clear Barcode 2 merely because Pixy saw several empty frames
        # while the robot was facing/reversing away from it. It must now clear
        # the barcode and physically move away before Barcode 2 is re-armed.
        self.latch_barcode(2)
        self.barcode2_rearm_required = True
        self.barcode2_clear_frames = 0

        self.queue_buzzer_pattern(self.BEEP_SUCCESS)
        self.sensor_recovery_count = 0
        self.current_state = RobotState.OBSTACLE_REACQUIRE_LINE
        self.say(
            "Obstacle routine complete. Reacquiring the track line."
        )

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================

    def control_loop(self) -> None:
        """Main timer callback: read the Pixy, then run one step of the state machine.

        The Pixy read happens before the lock is taken, on purpose: a slow USB
        transaction must never block the watchdog from stopping the robot.
        """
        self.update_buzzer_queue()

        if not self.pixy_ready:
            self.publish_motion(self.stop_twist(), alarm=True)
            return

        # Keep a potentially blocking Pixy USB transaction outside the FSM
        # lock so the independent watchdog can still issue a stop command.
        frame = self.read_pixy_frame()
        with self.state_lock:
            self.control_loop_from_frame(frame)

    def control_loop_from_frame(self, frame: PixyFrame) -> None:
        """One step of the state machine, using the Pixy frame just captured.

        This is the heart of the program. Each branch handles one RobotState:
        work out the right Twist for that state, decide whether to move to the
        next state, then publish exactly once at the end.
        """
        twist = self.stop_twist()

        self.update_line_stability(frame)
        self.update_barcode_memory(frame.barcode_count)

        confirmed_intersection = self.update_intersection_stability(
            frame.raw_intersection
        )

        self.log_status(frame, confirmed_intersection)

        if self.sensor_pause_active:
            self.publish_motion(twist, alarm=True)
            return

        # Fresh /scan callbacks own obstacle motion. During reverse, Pixy2 is
        # still checked here as an independent stop condition: once the track
        # line is reliably visible, stop reversing and resume PID tracking.
        if self.current_state == RobotState.OBSTACLE_RETURN:
            if frame.best_vector is not None and frame.line_error is not None:
                self.reverse_line_seen_count += 1
            else:
                self.reverse_line_seen_count = 0

            if (
                self.reverse_line_seen_count >= self.REVERSE_LINE_CONFIRM_FRAMES
                and not self.front_nan_reverse_used
            ):
                self.stop_actuator_signal()
                self.clear_command_queue()
                self.last_command_source_code = None
                self.reverse_line_seen_count = 0
                self.reacquire_count = 0
                self.line_lost_count = 0
                self.prev_error = 0.0
                self.last_intersection_time = time.time()
                self.obstacle_turn_start_time = time.time()
                self.current_state = RobotState.OBSTACLE_RETURN_YAW
                self.detail(
                    "Track line confirmed during reverse. Stopping reverse "
                    "and restoring the original main-track yaw first."
                )
                self.publish_motion(self.stop_twist(), alarm=False)
                return

        if self.current_state in self.obstacle_states():
            return

        if self.current_state == RobotState.OBSTACLE_FAIL_STOP:
            self.fail_stop_outputs()
            if (
                time.time() - self.fail_stop_enter_time
                <= self.FAIL_STOP_COMMAND_HOLD_SEC
            ):
                try:
                    if rclpy.ok():
                        self.cmd_vel_pub.publish(twist)
                except Exception:
                    pass
            return

        # ---------------------------------------------------------------------
        # Line-lost alarm
        # ---------------------------------------------------------------------

        if self.current_state == RobotState.LINE_LOST_ALARM:
            if frame.best_vector is not None and frame.line_error is not None:
                self.reacquire_count += 1
            else:
                self.reacquire_count = 0

            if self.reacquire_count >= self.LINE_REACQUIRE_FRAMES:
                self.say("Line reacquired. Resuming navigation.")

                self.reacquire_count = 0
                self.line_lost_count = 0
                self.prev_error = 0.0

                if self.pending_command:
                    self.current_state = RobotState.LINE_FOLLOW_ARMED
                else:
                    self.current_state = RobotState.LINE_FOLLOW
            else:
                self.publish_motion(twist, alarm=True)
                return

        # ---------------------------------------------------------------------
        # Permanent stop command
        # ---------------------------------------------------------------------

        if self.current_state == RobotState.STOP_COMMAND:
            self.set_stop_led(True)
            self.publish_motion(twist, alarm=False)
            return

        # ---------------------------------------------------------------------
        # Normal line-loss check
        # ---------------------------------------------------------------------

        if self.current_state in (
            RobotState.LINE_FOLLOW,
            RobotState.LINE_FOLLOW_ARMED,
        ):
            if (
                frame.best_vector is None
                and self.line_lost_count >= self.LINE_LOST_FRAMES
            ):
                self.enter_line_lost_alarm()
                self.publish_motion(twist, alarm=True)
                return

        # ---------------------------------------------------------------------
        # Normal line following
        # ---------------------------------------------------------------------

        if self.current_state == RobotState.LINE_FOLLOW:
            if frame.best_vector is not None and frame.line_error is not None:
                self.execute_pd_line_follow(twist, frame.line_error)

        # ---------------------------------------------------------------------
        # Command stored; wait for intersection
        # ---------------------------------------------------------------------

        elif self.current_state == RobotState.LINE_FOLLOW_ARMED:
            if confirmed_intersection and self.pending_command is not None:
                self.say(
                    f"Intersection confirmed by {frame.intersection_reason}. "
                    f"Command={self.pending_command}."
                )

                self.last_intersection_time = time.time()
                self.intersection_stable_count = 0
                self.start_distance_move()

                self.current_state = RobotState.MOVE_TO_TURN_CENTER

                twist.linear.x = self.SLOW_FORWARD_SPEED
                twist.angular.z = 0.0
            elif frame.best_vector is not None and frame.line_error is not None:
                self.execute_pd_line_follow(
                    twist,
                    frame.line_error,
                    requested_speed=self.ARMED_FORWARD_SPEED,
                )

        # ---------------------------------------------------------------------
        # Move robot centre into intersection
        # ---------------------------------------------------------------------

        elif self.current_state == RobotState.MOVE_TO_TURN_CENTER:
            if not self.odom_is_fresh():
                self.warn(
                    "Fresh odometry unavailable; holding before command "
                    "execution."
                )
            elif (
                time.time() - self.move_start_time
                > self.MOVE_CENTER_TIMEOUT_SEC
            ):
                self.enter_fail_stop(
                    "Move-to-intersection-centre timed out before travelling "
                    f"{self.FORWARD_DISTANCE_AFTER_INTERSECTION_M:.2f} m."
                )
                return
            elif (
                self.distance_from_move_start()
                < self.FORWARD_DISTANCE_AFTER_INTERSECTION_M
            ):
                twist.linear.x = self.SLOW_FORWARD_SPEED
                twist.angular.z = 0.0
            else:
                command = self.pending_command

                if command == "LEFT":
                    self.start_odom_turn("LEFT")
                    self.current_state = RobotState.EXECUTE_TURN_ODOM
                    self.say(
                        "At intersection centre. Starting LEFT turn."
                    )

                elif command == "RIGHT":
                    self.start_odom_turn("RIGHT")
                    self.current_state = RobotState.EXECUTE_TURN_ODOM
                    self.say(
                        "At intersection centre. Starting RIGHT turn."
                    )

                elif command == "STOP":
                    self.clear_command_queue()
                    self.last_command_source_code = None
                    self.current_state = RobotState.STOP_COMMAND
                    self.say(
                        "STOP barcode command executed at intersection."
                    )

                else:
                    self.warn(
                        "Invalid command. Returning to line following."
                    )
                    self.pending_command = None
                    self.last_command_source_code = None
                    self.promote_queued_command()
                    self.current_state = (
                        RobotState.LINE_FOLLOW_ARMED
                        if self.pending_command is not None
                        else RobotState.LINE_FOLLOW
                    )

        # ---------------------------------------------------------------------
        # Odometry-controlled left/right turn
        # ---------------------------------------------------------------------

        elif self.current_state == RobotState.EXECUTE_TURN_ODOM:
            if not self.odom_is_fresh():
                self.warn(
                    "Fresh odometry unavailable during turn; stopping."
                )
            else:
                turned = self.turned_angle_abs()
                elapsed = time.time() - self.turn_start_time

                target_not_reached = turned < (
                    self.TURN_TARGET_RAD - self.TURN_TOLERANCE_RAD
                )

                if target_not_reached:
                    if elapsed >= self.MAX_TURN_TIME_SEC:
                        self.enter_fail_stop(
                            "Normal intersection turn timed out before "
                            f"reaching its target; measured "
                            f"{math.degrees(turned):.1f} degrees."
                        )
                        return
                    if self.turn_direction == "LEFT":
                        twist.angular.z = self.TURN_SPEED
                    elif self.turn_direction == "RIGHT":
                        twist.angular.z = -self.TURN_SPEED
                else:
                    self.say(
                        f"Turn completed: {math.degrees(turned):.1f} degrees."
                    )

                    # The previous LEFT/RIGHT command has now been consumed.
                    # Release pending_command before centring so a barcode on
                    # the outgoing track can be captured immediately.
                    self.pending_command = None
                    self.last_command_source_code = None
                    self.last_barcode_accept_time = 0.0
                    # Pull the next stored command (if any) into the active
                    # slot so back-to-back junctions work.
                    self.promote_queued_command()
                    self.current_state = RobotState.SNAP_BACK_TO_LINE
                    self.prev_error = 0.0

        # ---------------------------------------------------------------------
        # Reacquire line after turn
        # ---------------------------------------------------------------------

        elif self.current_state == RobotState.SNAP_BACK_TO_LINE:
            if frame.best_vector is not None and frame.line_error is not None:
                error = frame.line_error
                derivative = error - self.prev_error
                steering = self.kp * error + self.kd * derivative

                twist.linear.x = self.CENTERING_SPEED
                twist.angular.z = max(min(-steering, 0.22), -0.22)

                self.prev_error = error

                if abs(error) <= 6.0:
                    self.turn_direction = None

                    # The original intersection cooldown started before the
                    # turn and has already elapsed. Do not restart it here,
                    # because that would hide another junction only a few
                    # centimetres along the outgoing track.
                    self.barcode_ignore_until = (
                        time.time() + self.BARCODE_IGNORE_AFTER_COMMAND_SEC
                    )
                    self.current_state = (
                        RobotState.LINE_FOLLOW_ARMED
                        if self.pending_command is not None
                        else RobotState.LINE_FOLLOW
                    )

                    if self.pending_command is not None:
                        self.say(
                            "Line centred after turn. Resuming with nearby "
                            f"{self.pending_command} barcode already stored."
                        )
                    else:
                        self.last_command_source_code = None
                        self.say(
                            "Line centred after turn. Resuming normal tracking."
                        )
            else:
                if self.turn_direction == "LEFT":
                    twist.angular.z = 0.8
                elif self.turn_direction == "RIGHT":
                    twist.angular.z = -0.8

        # ---------------------------------------------------------------------
        # Obstacle: find the line again after returning
        # ---------------------------------------------------------------------

        elif self.current_state == RobotState.OBSTACLE_REACQUIRE_LINE:
            if frame.best_vector is not None and frame.line_error is not None:
                error = frame.line_error
                self.reacquire_count += 1

                derivative = error - self.prev_error
                steering = self.kp * error + self.kd * derivative

                # The old code was limited to 0.22 rad/s and waited until the
                # line error was within 6 px. In the supplied log, err=32.1,
                # so it remained in this state and ignored the next barcode.
                if abs(error) >= 20.0:
                    twist.linear.x = self.OBSTACLE_REACQUIRE_LARGE_ERROR_SPEED
                else:
                    twist.linear.x = self.OBSTACLE_REACQUIRE_SPEED

                twist.angular.z = max(
                    min(
                        -steering,
                        self.OBSTACLE_REACQUIRE_MAX_ANGULAR_SPEED,
                    ),
                    -self.OBSTACLE_REACQUIRE_MAX_ANGULAR_SPEED,
                )
                self.prev_error = error
            else:
                self.reacquire_count = 0
                twist.linear.x = 0.0

                # The original yaw has already been restored, so only a small
                # Pixy search rotation should be needed.
                if self.turn_direction == "LEFT":
                    twist.angular.z = -self.OBSTACLE_REACQUIRE_SEARCH_SPEED
                elif self.turn_direction == "RIGHT":
                    twist.angular.z = self.OBSTACLE_REACQUIRE_SEARCH_SPEED
                else:
                    twist.angular.z = self.OBSTACLE_REACQUIRE_SEARCH_SPEED

            if (
                self.reacquire_count
                >= self.OBSTACLE_REACQUIRE_CONFIRM_FRAMES
            ):
                self.reacquire_count = 0
                self.line_lost_count = 0
                self.prev_error = 0.0
                self.turn_direction = None

                self.current_state = (
                    RobotState.LINE_FOLLOW_ARMED
                    if self.pending_command is not None
                    else RobotState.LINE_FOLLOW
                )

                self.say(
                    "Track line confirmed after obstacle routine. "
                    "Resuming faster PID tracking; barcode reading is active."
                )

        self.publish_motion(twist, alarm=False)

    def front_distance_difference(self) -> float:
        """How unevenly the front-left and front-right beams see the board.

        Near zero means the robot is square to the board. Large means it is
        looking at the board at an angle and should keep aligning.
        """
        if (
            math.isinf(self.front_left_dist)
            or math.isinf(self.front_right_dist)
        ):
            return float("inf")

        return abs(self.front_left_dist - self.front_right_dist)

    # =========================================================================
    # LOGGING
    # =========================================================================

    def log_status_friendly(self, frame: PixyFrame, now: float) -> None:
        """One short, colour-coded line per second.

        Example:
            [ 12.4s] following the line          line OK   err  -3  next: -
        """
        phase = PHASE_LABELS.get(self.current_state, self.current_state)

        if self.current_state == RobotState.OBSTACLE_FAIL_STOP:
            phase_colour = "1;91"
        elif self.current_state in self.obstacle_states():
            phase_colour = "1;95"
        elif self.current_state == RobotState.LINE_LOST_ALARM:
            phase_colour = "1;93"
        else:
            phase_colour = "1;92"

        if frame.best_vector is not None and frame.line_error is not None:
            line_text = self.colour("line OK ", "92")
            error_text = f"err {frame.line_error:5.1f}"
        else:
            line_text = self.colour("NO LINE ", "91")
            error_text = "err   ---"

        pending = self.pending_command or "-"

        health = []
        if not self.odom_is_fresh(now):
            health.append("odom stale")
        if self.scan_ready and not self.scan_is_fresh(now):
            health.append("lidar stale")
        if self.sensor_pause_active:
            health.append("PAUSED")
        health_text = (
            "  " + self.colour(", ".join(health), "1;93") if health else ""
        )

        elapsed = now - self.node_start_time
        print(
            f"  [{elapsed:6.1f}s] "
            f"{self.colour(phase.ljust(34), phase_colour)} "
            f"{line_text} {error_text}  next: {pending}"
            f"{health_text}",
            flush=True,
        )

    def log_status(
        self,
        frame: PixyFrame,
        confirmed_intersection: bool,
    ) -> None:
        """Print one status update, at most once per STATUS_LOG_INTERVAL seconds.

        Friendly mode prints a short plain-English line; debug mode prints the
        full numbers.
        """
        now = time.time()

        if now - self.last_status_log_time < self.STATUS_LOG_INTERVAL:
            return

        self.last_status_log_time = now

        if not self.VERBOSE:
            self.log_status_friendly(frame, now)
            return

        if frame.best_vector is None or frame.line_error is None:
            line_text = "line=False"
        else:
            line_text = f"line=True err={frame.line_error:.1f}"

        if self.odom_ready and self.last_odom_time > 0.0:
            odom_age = max(0.0, now - self.last_odom_time)
            odom_text = (
                f"odom={'fresh' if self.odom_is_fresh(now) else 'stale'} "
                f"age={odom_age:.2f}s"
            )
        else:
            odom_text = "odom=not_ready"

        if self.scan_ready:
            scan_age = (
                max(0.0, now - self.last_scan_time)
                if self.last_scan_time > 0.0
                else float("inf")
            )
            if self.front_board_target is None:
                target_text = "target=none"
            else:
                target_text = (
                    f"target={self.front_board_target.distance:.2f}m/"
                    f"{self.front_board_target.bearing_deg:.1f}deg/"
                    f"{self.front_board_target.width_m:.2f}m"
                )

            scan_text = (
                f"scan={'fresh' if self.scan_is_fresh(now) else 'stale'} "
                f"age={scan_age:.2f}s, front={self.front_dist:.2f}, "
                f"front_center={self.front_center_dist:.2f}, "
                f"left={self.left_side_dist:.2f}, "
                f"right={self.right_side_dist:.2f}, "
                f"front_nan={self.front_nan_ratio:.2f}, "
                f"rear={self.rear_dist:.2f}, "
                f"rear_nan={self.rear_nan_ratio:.2f}, "
                f"{target_text}, "
                f"side_scans={len(self.side_scan_votes)}/{self.SIDE_SCAN_COUNT}"
            )
        else:
            scan_text = "scan=not_ready"

        self.get_logger().info(
            f"state={self.current_state}, "
            f"pending={self.pending_command}, "
            f"{line_text}, "
            f"int_raw={frame.raw_intersection}, "
            f"int_ok={confirmed_intersection}, "
            f"reason={frame.intersection_reason}, "
            f"counts(v/b/i)="
            f"{frame.vector_count}/{frame.barcode_count}/"
            f"{frame.intersection_count}, "
            f"{odom_text}, {scan_text}, "
            f"barcode_votes={self.barcode_vote_codes}, "
            f"sensor_pause={self.sensor_pause_active}, "
            f"b2_rearm_lock={self.barcode2_rearm_required}, "
            f"b2_clear={self.barcode2_clear_frames}/"
            f"{self.BARCODE2_REARM_CLEAR_FRAMES}"
        )

    # =========================================================================
    # CLEANUP
    # =========================================================================

    def stop_robot_safely(self) -> None:
        """Publish zero velocity a few times so a dropped message cannot leave it moving."""
        stop = self.stop_twist()

        for _ in range(3):
            try:
                if rclpy.ok():
                    self.cmd_vel_pub.publish(stop)
                time.sleep(0.05)
            except Exception:
                break

    def cleanup(self) -> None:
        """Shut everything down tidily: wheels stopped, actuator off, LEDs off, GPIO freed.

        Always run this on exit. Skipping it can leave the actuator powered or
        an LED lit after the program has gone.
        """
        try:
            self.get_logger().info("Stopping robot and cleaning up.")
        except Exception:
            pass

        self.stop_robot_safely()
        self.stop_actuator_signal()
        self.all_leds_off()

        if self.gpio_ready and self.pwm is not None:
            try:
                self.pwm.stop()
            except Exception:
                pass

        if GPIO_AVAILABLE and self.gpio_ready:
            try:
                GPIO.cleanup()
            except Exception:
                pass

        if PIXY_AVAILABLE:
            try:
                if hasattr(pixy, "close"):
                    pixy.close()
            except Exception:
                pass


def main(args=None) -> None:
    """Start ROS, spin the controller, and always clean up on the way out.

    A MultiThreadedExecutor with 4 threads is used so Pixy polling, LiDAR,
    odometry and the watchdog each get their own thread and cannot starve
    one another. With a single-threaded executor a slow Pixy read would
    delay the watchdog, which is exactly what must not happen.
    """
    rclpy.init(args=args)
    node = TurtleBot3CombinedController()
    # Four threads keep Pixy polling, LiDAR, odometry and the independent
    # watchdog from starving one another.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up while the ROS context is still valid.
        try:
            node.cleanup()
        except Exception as exc:
            print(f"Cleanup warning: {exc}")

        try:
            executor.remove_node(node)
            executor.shutdown()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
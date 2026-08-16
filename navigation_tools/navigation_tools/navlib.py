#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from dataclasses import dataclass
from typing import Optional, Union

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus as ActionGoalStatus
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Pose2D, PoseArray, PoseStamped
from pumas_interfaces.action import GazeHead, PumasNav
from pumas_interfaces.msg import Joints, StartAndEndJoints
from pumas_interfaces.srv import ParamReadWrite
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32, Float32MultiArray
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

default_arm_pose = {
    'arm_flex_joint': -0.26,  # default is 0.0
    'arm_lift_joint': 0.0,
    'arm_roll_joint': -1.57,
    'wrist_flex_joint': -1.57,
    'wrist_roll_joint': 0.0,
    'head_pan_joint': 0.0,
    'head_tilt_joint': np.deg2rad(0.0),
}

# Failure reasons that only the client can know about: the goal never reached
# mvn_pln, or navlib gave up on its own. Everything mvn_pln itself decides is
# passed through verbatim in NavStatus.message / .final_state_name instead --
# navlib deliberately does not define a second vocabulary for those.
CLIENT_ERROR_NONE = ''
CLIENT_ERROR_SERVER_UNAVAILABLE = 'server_unavailable'
CLIENT_ERROR_REJECTED = 'rejected'
CLIENT_ERROR_TIMEOUT = 'timeout'
CLIENT_ERROR_STOP_SIGNAL = 'stop_signal'

# Reverse lookup of the OUTCOME_* constants declared in PumasNav.action, derived
# from the generated message class so the names still have exactly one
# definition site (the .action file).
_OUTCOME_NAMES = {
    getattr(PumasNav.Result, _n): _n[len('OUTCOME_'):]
    for _n in dir(PumasNav.Result)
    if _n.startswith('OUTCOME_')
}


def outcome_name(outcome: int) -> str:
    """Human-readable name of a PumasNav.Result OUTCOME_* value, for logging."""
    return _OUTCOME_NAMES.get(outcome, f'OUTCOME({outcome})')


@dataclass(frozen=True)
class NavStatus:
    """Immutable snapshot of the current navigation state.

    Everything that comes from mvn_pln is passed through **unmodified**: navlib
    does not re-classify it, so the state vocabulary has exactly one definition
    site (mvn_pln_node.cpp / PumasNav.action). Read it any time, including while
    a goal is running, via ``NavModule.nav_status``.

    Beware: ``near_goal_reached`` is NOT a "stopped short of the goal" flag.
    mvn_pln raises it on an ordinary arrival too, as soon as the robot is within
    its ``proximity_criterion`` parameter (0.2 m in navigation.launch.xml). Use
    ``outcome`` to tell arrival kinds apart.
    """

    # --- ROS 2 action layer lifecycle (action_msgs/GoalStatus STATUS_*) ---
    goal_status: int
    active: bool
    done: bool

    # --- PumasNav.Feedback, verbatim (mvn_pln) ---
    state_name: str
    remaining_distance: float
    collision_risk: bool
    near_goal_reached: bool
    feedback_message: str
    # Settings in force for this goal, as mvn_pln resolved them. They arrive on
    # the feedback, so they only become valid once the first one lands.
    proximity_criterion: float  # radius that raises near_goal_reached
    min_reach_goal_dist: float  # no-replan radius; 0 = gate off
    goal_distance: float        # stop-short distance; 0 = full arrival

    # --- PumasNav.Result, verbatim (mvn_pln); only set once done ---
    success: bool
    outcome: int  # PumasNav.Result.OUTCOME_* -- machine-readable reason, set by
    # mvn_pln at every terminal branch. outcome_name() renders it.
    message: str
    final_state_name: str  # last state_name seen when the result arrived,
    # i.e. the termination tag mvn_pln published itself

    # --- /navigation/status, verbatim (mvn_pln) ---
    server_status: int  # actionlib_msgs/GoalStatus ACTIVE/SUCCEEDED/ABORTED
    server_status_text: str

    # --- navlib's own failures, kept apart from anything server-sent ---
    client_error: str  # one of the CLIENT_ERROR_* constants

    # --- incidental ---
    goal: Optional[Pose2D]
    elapsed: float

    @property
    def outcome_name(self) -> str:
        return outcome_name(self.outcome)

    @property
    def succeeded(self) -> bool:
        return self.done and self.success and not self.client_error

    @property
    def failed(self) -> bool:
        return self.done and not self.succeeded


class NavModule:
    """Navigation Module for the robot"""

    def __init__(self, node: Optional[Union[str, Node]] = None):

        context = rclpy.get_default_context()
        if not context.ok():
            rclpy.init()

        if node is None:
            self._node: Node = rclpy.create_node('nav_module_node')
            self._external_node = False
        elif isinstance(node, str):
            self._node: Node = rclpy.create_node(node)
            self._external_node = False
        elif isinstance(node, Node):
            self._node: Node = node
            self._external_node = True
        else:
            raise TypeError(
                "NavModule.__init__: 'node' must be of type str or rclpy.node.Node")

        # Everything below may be reached through __getattr__ delegation, so it
        # must be assigned after _node exists (see __getattr__).
        self._status_lock = threading.RLock()
        self._executor = None
        self._spin_thread = None
        self._callback_group = ReentrantCallbackGroup()

        self.marker = Marker()
        self.marker_num = 0

        self.robot_stop = False

        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None
        self.motion_execution_time = 0.0  # for motion_synth arm/head reached time
        self.via_points = None  # list of Pose2D the path should pass through

        # Publishers
        self.pub_marker = self.create_publisher(Marker, '/nav_goal_marker', 10)

        # Relative holonomic move (go_rel): base-frame [x, y, yaw].
        self.pub_goal_rel_pose = self.create_publisher(
            Float32MultiArray, '/simple_move/goal_rel_pose', 10
        )
        self.pub_robot_stop = self.create_publisher(
            Empty, '/navigation/stop', 10)

        # Recovery-pose publishers: when motion_synth is NOT used, the body is
        self.pub_arm_goal = self.create_publisher(
            Float32MultiArray, '/hardware/arm/goal_pose', 10)
        self.pub_torso_goal = self.create_publisher(
            Float32, '/hardware/torso/goal_pose', 10)
        self.pub_head_goal_pose = self.create_publisher(
            Float32MultiArray, '/hardware/head/goal_pose', 10
        )

        self.create_subscription(Empty, '/stop', self.callback_stop, 10)

        # Service Clients
        self.param_rw_client = self.create_client(
            ParamReadWrite, '/param_read_write',
            callback_group=self._callback_group)

        # pumas_nav action cli
        self.nav_action_client = ActionClient(
            self._node, PumasNav, '/pumas_nav',
            callback_group=self._callback_group)
        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None

        self._action_feedback = None
        self._near_goal_callback = None
        self._feedback_callback = None

        # [s] How often nav_feedback_callback() repeats an info line while the
        # state does not change. 0.0 = every message (30 Hz), None = only on
        # state changes (the old behaviour).
        self.feedback_log_period = 2.0
        self._last_feedback_log = 0.0

        # Bumped by reset_action_state(); callbacks carry the generation they
        # were registered under and drop out if it no longer matches.
        self._generation = 0

        # Status fields, all guarded by _status_lock. reset_action_state()
        # defines and clears every one of them.
        self._goal_response_event = threading.Event()
        self._result_event = threading.Event()
        self.reset_action_state()

        # Gaze action client
        self.gaze_action_client = ActionClient(
            self._node, GazeHead, '/gaze_head',
            callback_group=self._callback_group)
        self._gaze_goal_handle = None
        self._gaze_active = False

        # Relative move (go_rel): command goes out on pub_goal_rel_pose, and
        self.create_subscription(
            GoalStatus, '/simple_move/goal_reached', self._move_goal_reached_callback, 10
        )
        self._move_done = False
        self._move_success = False
        # Only true while go_rel() is waiting; see _move_goal_reached_callback.
        self._go_rel_active = False

        # mvn_pln also narrates itself on /navigation/status. This is the only
        # window into the stretches where the action publishes no feedback at
        # all (collision recovery, waiting for temporal obstacles, potential
        # fields wait, final angle correction).
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            GoalStatus, '/navigation/status', self._nav_status_callback, status_qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self._node)

        # Spin in the background so callbacks keep firing outside of the
        # blocking helpers -- that is what makes nav_status readable from
        # another thread while the robot is driving.
        self._start_spin_thread()

        self._load_default_arm_pose()  # call once here -> arm moves default pose

        self.get_logger().info('NavModule.->initialized')

    def __getattr__(self, name):
        # Only called when normal lookup fails. Guard _node itself, otherwise a
        # lookup before __init__ assigns it recurses forever.
        if name == '_node':
            raise AttributeError(name)
        return getattr(self._node, name)

    # ------------------------------------------------------------------
    # Executor
    # ------------------------------------------------------------------

    def _start_spin_thread(self):
        """Spin our own node in a daemon thread (internal-node mode only).

        When the caller passed in their own Node they are responsible for
        spinning it, exactly as before.
        """
        if self._external_node:
            return

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def shutdown(self):
        """Stop the background executor. Safe to call more than once."""
        if self._executor is None:
            return
        self._executor.shutdown()
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
        self._executor.remove_node(self._node)
        self._executor = None
        self._spin_thread = None

    def _timeout_now(self) -> float:
        """Seconds for timeout arithmetic. Deliberately NOT a ROS clock.

        Everything navlib times is a duration -- "how long have I been waiting
        for this result" -- never an instant that has to line up with somebody
        else's timeline. For that, a monotonic count is the only source with no
        failure mode:

        * ROS system time (use_sim_time false) is CLOCK_REALTIME, which chrony
          is allowed to *step* (``makestep``, typically shortly after boot). A
          step moves every deadline derived from it, so a timeout fires
          instantly or effectively never.
        * ROS time (use_sim_time true) stops when the simulator pauses and jumps
          when a bag loops. A paused simulator turns every timeout into a hang,
          which is worse than the thing the timeout was protecting against.

        time.monotonic() cannot be stepped, paused or rewound.

        The cost is that under a simulator running far from 1.0 real-time factor
        a timeout is counted in wall seconds rather than simulated ones. That
        only shows up as a timeout firing early, which is visible and harmless,
        and NavModule is not launched with the stack anyway -- nothing sets
        use_sim_time on the node it creates, so a ROS clock here would read
        system time under simulation regardless.

        Message stamps are a separate matter and still use the node clock (see
        send_nav_action_goal); mvn_pln does not read them -- it restamps
        everything itself -- so they are for RViz only.
        """
        return time.monotonic()

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """Block until `future` completes. Something else must be spinning the
        node -- either our background thread or the caller's executor."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(timeout=timeout_sec)

    def call_param_rw(
        self,
        node_name,
        param_name,
        param_value: str = '',
        write: bool = False,
        timeout_sec: float = 3.0,
    ):
        req = ParamReadWrite.Request()
        req.node_name = node_name
        req.param_name = param_name
        req.write = write
        req.value = param_value

        future = self.param_rw_client.call_async(req)

        if not self._wait_for_future(future, timeout_sec):
            self.get_logger().error('NavModule.->param_read_write call timed out')
            return None

        if future.result() is not None:
            return future.result().param_value
        else:
            self.get_logger().error('NavModule.->param_read_write service call failed')
            return None

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def reset_action_state(self):
        self._goal_handle = None
        self._result_future = None
        self._send_goal_future = None
        self._action_feedback = None
        self._goal_response_event.clear()
        self._result_event.clear()
        # Let the first feedback of a new goal print immediately.
        self._last_feedback_log = 0.0

        with self._status_lock:
            # Invalidate every callback still in flight for the previous goal.
            # mvn_pln preempts on a new goal, so the old goal's result (an abort
            # carrying OUTCOME_PREEMPTED) lands AFTER this reset and would
            # otherwise mark the fresh goal as already finished.
            self._generation += 1

            self._action_sent = False
            self._action_done = False
            self._action_success = False
            self._action_near_goal = False
            self._action_message = ''
            self._action_outcome = PumasNav.Result.OUTCOME_UNKNOWN
            self._goal_status = ActionGoalStatus.STATUS_UNKNOWN

            self._state_name = ''
            self._final_state_name = ''
            self._remaining_distance = 0.0
            self._collision_risk = False
            self._feedback_message = ''

            # /navigation/status is transient_local, so a stale latched message
            # from the previous goal is delivered on subscribe. Clearing here
            # keeps it from being read as the current goal's narration.
            self._server_status = GoalStatus.PENDING
            self._server_status_text = ''

            self._client_error = CLIENT_ERROR_NONE
            self._goal_pose2d = None
            self._proximity_criterion = 0.0
            self._min_reach_goal_dist = 0.0
            self._goal_distance = 0.0
            self._start_time = self._timeout_now()

    @property
    def nav_status(self) -> NavStatus:
        """Snapshot of the current navigation state. Safe to call from any
        thread, including while a goal is running."""
        with self._status_lock:
            return NavStatus(
                goal_status=self._goal_status,
                active=self._action_sent and not self._action_done,
                done=self._action_done,
                state_name=self._state_name,
                remaining_distance=self._remaining_distance,
                collision_risk=self._collision_risk,
                near_goal_reached=self._action_near_goal,
                feedback_message=self._feedback_message,
                proximity_criterion=self._proximity_criterion,
                min_reach_goal_dist=self._min_reach_goal_dist,
                goal_distance=self._goal_distance,
                success=self._action_success,
                outcome=self._action_outcome,
                message=self._action_message,
                final_state_name=self._final_state_name,
                server_status=self._server_status,
                server_status_text=self._server_status_text,
                client_error=self._client_error,
                goal=self._goal_pose2d,
                elapsed=self._timeout_now() - self._start_time,
            )

    @property
    def is_navigating(self) -> bool:
        with self._status_lock:
            return self._action_sent and not self._action_done

    @property
    def nav_state_name(self) -> str:
        """mvn_pln's own feedback state_name, unmodified."""
        with self._status_lock:
            return self._state_name

    @property
    def nav_message(self) -> str:
        """mvn_pln's own result.message, unmodified."""
        with self._status_lock:
            return self._action_message

    @property
    def nav_outcome(self) -> int:
        """mvn_pln's own result.outcome (PumasNav.Result.OUTCOME_*).

        Stays OUTCOME_UNKNOWN while running, and when the goal never reached
        mvn_pln at all -- check ``nav_status.client_error`` for that case.
        """
        with self._status_lock:
            return self._action_outcome

    @property
    def remaining_distance(self) -> float:
        with self._status_lock:
            return self._remaining_distance

    @property
    def collision_risk(self) -> bool:
        with self._status_lock:
            return self._collision_risk

    @property
    def proximity_criterion(self) -> float:
        """mvn_pln's near-goal radius [m] for the running goal, as reported on
        the feedback. 0.0 until the first feedback arrives."""
        with self._status_lock:
            return self._proximity_criterion

    def _set_client_error(self, reason: str):
        with self._status_lock:
            self._client_error = reason

    def _nav_status_callback(self, msg):
        with self._status_lock:
            self._server_status = msg.status
            self._server_status_text = msg.text

    # ------------------------------------------------------------------
    # PumasNav action callbacks
    # ------------------------------------------------------------------

    def _is_stale(self, generation: int) -> bool:
        """True once a newer goal has superseded the one this callback belongs
        to. Every action callback bails out on it, so a preempted goal cannot
        write into the status of the goal that replaced it."""
        with self._status_lock:
            return generation != self._generation

    def nav_feedback_callback(self, feedback_msg, generation: int):
        if self._is_stale(generation):
            return

        feedback = feedback_msg.feedback
        self._action_feedback = feedback

        with self._status_lock:
            state_changed = feedback.state_name != self._state_name
            self._state_name = feedback.state_name
            self._remaining_distance = feedback.remaining_distance
            self._collision_risk = feedback.collision_risk
            self._feedback_message = feedback.message
            self._proximity_criterion = feedback.proximity_criterion
            self._min_reach_goal_dist = feedback.min_reach_goal_dist
            self._goal_distance = feedback.goal_distance
            fired_near_goal = self._action_near_goal
            if feedback.near_goal_reached:
                self._action_near_goal = True

        if feedback.near_goal_reached and not fired_near_goal:
            if self._near_goal_callback is not None:
                try:
                    self._near_goal_callback(feedback)
                except Exception as e:
                    self.get_logger().error(
                        f'NavModule.->near_goal_callback failed: {e}')

        if self._feedback_callback is not None:
            try:
                self._feedback_callback(self.nav_status)
            except Exception as e:
                self.get_logger().error(
                    f'NavModule.->feedback_callback failed: {e}')

        # WAIT_FOR_MOVE_FINISHED arrives at 30 Hz, so the firehose stays on
        # debug. Announcing state changes only is too quiet the other way: a
        # long path spends 20+ s inside that one state and the log jumps
        # straight from dist=2.4 to dist=0.2 with no sign of progress in
        # between. Also emit a line every feedback_log_period seconds so the
        # distance keeps ticking down. Set feedback_log_period to 0.0 for every
        # message, or None for state changes only.
        line = (
            f'Nav feedback: state={feedback.state_name}, '
            f'dist={feedback.remaining_distance:.3f}, '
            f'near={feedback.near_goal_reached}, '
            f'collision={feedback.collision_risk}, '
            f'msg={feedback.message}'
        )
        period = self.feedback_log_period
        now = time.monotonic()
        due = period is not None and (now - self._last_feedback_log) >= period
        if state_changed or due:
            self._last_feedback_log = now
            self.get_logger().info(line)
        else:
            self.get_logger().debug(line)

    def nav_result_callback(self, future, generation: int):
        response = future.result()
        result = response.result

        if self._is_stale(generation):
            # Normal after a preempt: mvn_pln aborts the goal we walked away
            # from with OUTCOME_PREEMPTED. Log it, never let it touch status.
            self.get_logger().info(
                f'NavModule.->Ignoring result of superseded goal: '
                f'outcome={outcome_name(result.outcome)}, msg="{result.message}"'
            )
            return

        with self._status_lock:
            # Snapshot the state_name we were in when the result landed: for
            # STOP_SHORT / GOAL_RELAXED / GOAL_NOT_CONVERGING / NEAR_GOAL_ABORT
            # / NEAR_GOAL_ACCEPTED mvn_pln publishes that tag as its last
            # feedback right before terminating.
            self._final_state_name = self._state_name
            self._goal_status = response.status
            self._action_done = True
            self._action_success = bool(result.success)
            self._action_near_goal = bool(result.near_goal_reached)
            self._action_message = result.message
            self._action_outcome = result.outcome

        self.get_logger().info(
            f'NavModule.->PumasNav result: success={result.success}, '
            f'outcome={outcome_name(result.outcome)}, '
            f'final_state={self._final_state_name}, msg="{result.message}"'
        )
        self._result_event.set()

    def nav_goal_response_callback(self, future, generation: int):
        goal_handle = future.result()

        if self._is_stale(generation):
            # Superseded before the server even answered. Terminate the orphan
            # so it does not keep driving alongside the goal that replaced it.
            if goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('NavModule.->PumasNav goal rejected')
            with self._status_lock:
                self._action_done = True
                self._action_success = False
                self._action_message = 'Goal rejected'
                self._client_error = CLIENT_ERROR_REJECTED
                self._goal_status = ActionGoalStatus.STATUS_UNKNOWN
            self._goal_response_event.set()
            self._result_event.set()
            return

        self.get_logger().info('NavModule.->PumasNav goal accepted')
        self._goal_handle = goal_handle
        with self._status_lock:
            self._goal_status = ActionGoalStatus.STATUS_ACCEPTED
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(
            lambda f: self.nav_result_callback(f, generation))
        self._goal_response_event.set()

    def cancel_nav_action(self):
        if self._goal_handle is not None:
            self.get_logger().warn('NavModule.->Canceling PumasNav goal')
            self._goal_handle.cancel_goal_async()

    def _move_goal_reached_callback(self, msg):
        # "-1" is simple_move's sentinel for every NON-PATH goal, not a private
        # channel for go_rel: mvn_pln drives simple_move the same way for its
        # own in-place steps, most importantly the final angle correction
        # (mvn_pln_node.cpp SM_WAIT_FOR_ANGLE_CORRECTED). Matching on the id
        # alone therefore fires in the middle of every go_abs/nav_goal and ends
        # the wait with _action_outcome still OUTCOME_UNKNOWN -- the caller sees
        # success with no reason. Only a go_rel we actually sent may land here.
        if not self._go_rel_active:
            return
        if msg.goal_id.id != '-1':
            return
        self._move_done = True
        self._move_success = msg.status == GoalStatus.SUCCEEDED
        with self._status_lock:
            self._server_status = msg.status
            self._action_done = True
            self._action_success = self._move_success
        self._result_event.set()

    # ------------------------------------------------------------------
    # Gaze
    # ------------------------------------------------------------------

    def _gaze_goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('NavModule.->GazeHead goal rejected')
            self._gaze_active = False
            return
        self._gaze_goal_handle = goal_handle
        self._gaze_active = True
        self.get_logger().info('NavModule.->GazeHead goal accepted, gaze active')

    def start_gaze(self, gaze_tf_name: str = '', restore_head=None) -> bool:
        if not self.gaze_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('NavModule.->GazeHead action server not available')
            return False

        goal_msg = GazeHead.Goal()
        goal_msg.gaze_tf_name = gaze_tf_name
        if restore_head is not None:
            goal_msg.restore_head = True
            goal_msg.restore_pan = float(restore_head[0])
            goal_msg.restore_tilt = float(restore_head[1])

        self._gaze_goal_handle = None
        self._gaze_active = False

        send_future = self.gaze_action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._gaze_goal_response_callback)
        self.get_logger().info(
            f"NavModule.->GazeHead goal sent (tf='{gaze_tf_name or 'default'}')")
        return True

    def cancel_gaze(self):
        if self._gaze_goal_handle is not None:
            self.get_logger().info('NavModule.->Canceling GazeHead goal')
            self._gaze_goal_handle.cancel_goal_async()
        self._gaze_goal_handle = None
        self._gaze_active = False

    # ------------------------------------------------------------------
    # Runtime parameters
    # ------------------------------------------------------------------

    def _set_move_head(self, enabled: bool, sync: bool = True):
        value = 'true' if enabled else 'false'
        for node_name in ('head_controller', 'simple_move'):
            if sync:
                self.call_param_rw(
                    node_name=node_name,
                    param_name='move_head',
                    param_value=value,
                    write=True,
                )
            else:
                req = ParamReadWrite.Request()
                req.node_name = node_name
                req.param_name = 'move_head'
                req.write = True
                req.value = value
                self.param_rw_client.call_async(req)
        self.get_logger().info(
            f'NavModule.->move_head set to {value} (head_controller, simple_move)'
        )

    def _set_use_point_cloud(self, enabled: bool):
        value = 'true' if enabled else 'false'

        self.call_param_rw(
            node_name='potential_fields',
            param_name='use_point_cloud',
            param_value=value,
            write=True,
        )
        self.call_param_rw(
            node_name='map_augmenter',
            param_name='use_point_cloud',
            param_value=value,
            write=True,
        )

        self.get_logger().info(f'NavModule.->use_point_cloud set to {value}')

    def _set_omni_goal_yaw_align(self, enabled: bool):
        # Toggle simple_move's last-mile omni (yaw_correction_omni_behavior) at
        # runtime, the same way _set_move_head toggles move_head. When enabled
        # the robot slides onto the goal pose while yawing onto its orientation
        # over the final yaw_correction_omni_distance metres, instead of arriving
        # and turning in place.
        value = 'true' if enabled else 'false'
        self.call_param_rw(
            node_name='simple_move',
            param_name='yaw_correction_omni_behavior',
            param_value=value,
            write=True,
        )
        self.get_logger().info(f'NavModule.->omni_goal_yaw_align set to {value}')

    def callback_stop(self, msg):
        self.robot_stop = True

    def get_global_pose_from_tf(self, target_frame='map', source_frame='base_footprint',
                                timeout_sec=2.0):
        """Return (x, y, yaw) of source_frame in target_frame, or None.

        Retries for up to timeout_sec because the first call right after
        NavModule() reliably loses a startup race: the listener has only a few
        tens of milliseconds of history, so the latest time the whole
        map -> odom -> base_footprint chain has in common can still fall before
        the earliest sample of one of its links. tf2 reports that as
        "Lookup would require extrapolation into the past", which reads like a
        broken TF tree but is just a buffer that has not filled yet -- it clears
        itself within a second. Pass timeout_sec=0.0 for a single attempt.

        The retry is ours on purpose: tf2's own ``timeout=`` argument waits on a
        default-constructed ``rclpy.clock.Clock()`` (see tf2_ros/buffer.py
        can_transform), i.e. CLOCK_REALTIME, and aborts the wait on any backward
        jump larger than 3 s -- exactly the chrony step _timeout_now() exists to
        be immune to. Asking for the latest transform with no tf2 timeout keeps
        the deadline ours.
        """
        deadline = self._timeout_now() + max(0.0, timeout_sec)
        last_ex = None

        while True:
            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time()
                )

                x = trans.transform.translation.x
                y = trans.transform.translation.y
                q = trans.transform.rotation
                euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
                yaw = euler[2]

                # self.get_logger().info(
                #    f"NavModule.->TF Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
                # )
                return x, y, yaw

            except TransformException as ex:
                last_ex = ex
                if self._timeout_now() >= deadline:
                    break
                # An extrapolation error is raised without waiting (the chain
                # exists, only the timestamps do not line up), so pace the retry
                # ourselves instead of spinning.
                time.sleep(0.05)

        self.get_logger().warn(
            f'NavModule.->TF lookup failed: {target_frame} -> {source_frame}: {last_ex}'
        )
        return None

    def pose_stamped2pose_2d(self, pose_stamped):
        pose2d = Pose2D()
        pose2d.x = pose_stamped.pose.position.x
        pose2d.y = pose_stamped.pose.position.y
        orientation = pose_stamped.pose.orientation
        euler = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        pose2d.theta = euler[2]
        return pose2d

    def _load_default_arm_pose(self, arm_node='/arm_controller', timeout_sec=3.0):
        """Load the default/recovery arm pose from the arm_controller node's
        params (set via <param> in its launch <node> block):

            arm_default_pose : double[5]

        One list, ordered by that node's arm_default_names, which defaults to
        [arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll] -- index 0 is the
        torso/lift and 1..4 are the arm joints (arm_node.cpp publish_default_pose).
        There is no separate torso parameter; asking for one used to make this
        whole read fail, because rclcpp's get_parameters service silently drops
        names that are not declared, so the response came back one value short.

        Head is reset to 0 (as in ROS1). If the params cannot be read (node not
        up / service timeout), fall back to the module-level default_arm_pose.
        Stored in self.default_arm_pose and used for the recovery pose and as
        the base for motion_synth start/end poses.
        """
        pose = dict(default_arm_pose)  # fallback

        cli = self._node.create_client(
            GetParameters, f'{arm_node}/get_parameters',
            callback_group=self._callback_group)
        if cli.wait_for_service(timeout_sec=timeout_sec):
            req = GetParameters.Request()
            req.names = ['arm_default_pose']
            future = cli.call_async(req)

            ok = self._wait_for_future(future, timeout_sec)

            res = future.result() if ok else None
            if res is not None and len(res.values) >= 1:
                arm = list(res.values[0].double_array_value)
                if len(arm) >= 5:
                    pose['arm_lift_joint'] = float(arm[0])
                    pose['arm_flex_joint'] = float(arm[1])
                    pose['arm_roll_joint'] = float(arm[2])
                    pose['wrist_flex_joint'] = float(arm[3])
                    pose['wrist_roll_joint'] = float(arm[4])
                    pose['head_pan_joint'] = 0.0
                    pose['head_tilt_joint'] = 0.0
                    self.get_logger().info(
                        f'NavModule.->default_arm_pose from {arm_node}: {pose}')
                else:
                    self.get_logger().warn(
                        f'NavModule.->arm_default_pose has {len(arm)} values, '
                        'expected 5; using module default'
                    )
            else:
                self.get_logger().warn(
                    f'NavModule.->failed to read {arm_node} params; using module default'
                )
        else:
            self.get_logger().warn(
                f'NavModule.->{arm_node}/get_parameters not available; using module default'
            )

        self.default_arm_pose = pose

    def create_arm_joint_goal(self, joint_poses):
        joint_poses = {**self.default_arm_pose, **(joint_poses or {})}
        joints = Joints()
        joints.arm_lift_joint = joint_poses['arm_lift_joint']
        joints.arm_flex_joint = joint_poses['arm_flex_joint']
        joints.arm_roll_joint = joint_poses['arm_roll_joint']
        joints.wrist_flex_joint = joint_poses['wrist_flex_joint']
        joints.wrist_roll_joint = joint_poses['wrist_roll_joint']
        joints.head_pan_joint = joint_poses['head_pan_joint']
        joints.head_tilt_joint = joint_poses['head_tilt_joint']
        return joints

    def _send_default_arm_pose(self, include_head: bool = True):
        """Move the body to default_arm_pose (used when motion_synth is off).

        Uses self.default_arm_pose (loaded from ROS params at launch, falling
        back to the module-level default_arm_pose).
        """
        pose = self.default_arm_pose
        lift = Float32()
        lift.data = float(pose['arm_lift_joint'])
        self.pub_torso_goal.publish(lift)

        arm = Float32MultiArray()
        arm.data = [
            float(pose['arm_flex_joint']),
            float(pose['arm_roll_joint']),
            float(pose['wrist_flex_joint']),
            float(pose['wrist_roll_joint']),
        ]
        self.pub_arm_goal.publish(arm)

        if include_head:
            head = Float32MultiArray()
            head.data = [
                float(pose['head_pan_joint']),
                float(pose['head_tilt_joint']),
            ]
            self.pub_head_goal_pose.publish(head)

        self.get_logger().info('NavModule.->No motion_synth: moving body to default_arm_pose')

    def send_nav_action_goal(
        self,
        goal: Pose2D,
        goal_distance: float = 0.0,
        min_reach_goal_dist: float = 0.0,
    ) -> bool:
        # Clear first so a goal that never leaves the client still reports a
        # clean status instead of the previous goal's leftovers.
        self.reset_action_state()

        if not self.nav_action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('NavModule.->PumasNav action server not available')
            with self._status_lock:
                self._action_sent = True
                self._goal_pose2d = goal
                self._action_done = True
                self._action_success = False
                self._action_message = 'Action server unavailable'
                self._client_error = CLIENT_ERROR_SERVER_UNAVAILABLE
            self._goal_response_event.set()
            self._result_event.set()
            return False

        goal_msg = PumasNav.Goal()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._node.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(goal.x)
        goal_pose.pose.position.y = float(goal.y)
        goal_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(goal.theta))
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        goal_msg.goal = goal_pose

        # Stop-short distance handled server-side by mvn_pln (<= 0 disables).
        goal_msg.goal_distance = (
            float(goal_distance) if goal_distance and goal_distance > 0 else 0.0
        )

        # Near-goal replan suppression: mvn_pln stops instead of replanning once
        # it would replan from within this distance of the goal. 0 leaves the
        # server's min_reach_goal_dist parameter in charge.
        goal_msg.min_reach_goal_dist = (
            float(min_reach_goal_dist)
            if min_reach_goal_dist and min_reach_goal_dist > 0
            else 0.0
        )

        # TODO
        # goal_msg.patience = True

        arm_goal = StartAndEndJoints()
        arm_goal.has_arm_start_pose = False
        arm_goal.has_arm_end_pose = False

        if self.motion_synth_start_pose is not None:
            arm_goal.has_arm_start_pose = True
            arm_goal.start_pose = self.create_arm_joint_goal(
                self.motion_synth_start_pose)

        if self.motion_synth_end_pose is not None:
            arm_goal.has_arm_end_pose = True
            arm_goal.end_pose = self.create_arm_joint_goal(
                self.motion_synth_end_pose)

        arm_goal.motion_execution_time = float(self.motion_execution_time)
        goal_msg.arm_joints = arm_goal
        goal_msg.use_arm = arm_goal.has_arm_start_pose or arm_goal.has_arm_end_pose

        # Via points (empty -> mvn_pln plans a plain start->goal path).
        via_array = PoseArray()
        via_array.header.frame_id = 'map'
        via_array.header.stamp = self._node.get_clock().now().to_msg()
        if self.via_points is not None:
            for vp in self.via_points:
                pose = Pose()
                pose.position.x = float(vp.x)
                pose.position.y = float(vp.y)
                pose.position.z = 0.0
                q = quaternion_from_euler(0.0, 0.0, float(vp.theta))
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                via_array.poses.append(pose)
        goal_msg.via_points = via_array

        with self._status_lock:
            self._action_sent = True
            self._goal_pose2d = goal
            # Tag every callback of this attempt; reset_action_state() above
            # already moved the generation on, so callbacks left over from the
            # goal this one preempts no longer match.
            generation = self._generation
        self.marker_plot(goal_pose)

        self._send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda fb: self.nav_feedback_callback(
                fb, generation),
        )
        self._send_goal_future.add_done_callback(
            lambda f: self.nav_goal_response_callback(f, generation))
        return True

    def handle_robot_stop(self):
        self.pub_robot_stop.publish(Empty())

    def marker_plot(self, goal):
        self.marker.header.frame_id = 'map'
        self.marker.header.stamp = self._node.get_clock().now().to_msg()
        self.marker.ns = 'goal_markers'
        self.marker.id = self.marker_num
        self.marker_num += 1
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose = goal.pose
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.pub_marker.publish(self.marker)

    def initialize_before_new_goal(self, send_stop=True):
        """Clear per-goal state before sending the next goal.

        send_stop cancels the running goal first. It defaults to False now that
        mvn_pln preempts: sending the new goal replaces the old one inside
        handle_accepted, with no gap where the robot has stopped and nothing is
        planned yet. Cancelling first only adds an extra halt (and a race, since
        the cancel does not take effect until the server's next 30 Hz tick).
        Pass send_stop=True to force the old goal down before starting over.
        """
        self.robot_stop = False

        if send_stop and self._goal_handle is not None:
            self.cancel_nav_action()
        self.motion_synth_start_pose = None
        self.motion_synth_end_pose = None
        self.motion_execution_time = 0.0
        self.via_points = None

        # Cancel any leftover gaze from a previous goal
        if self._gaze_active:
            self.cancel_gaze()
            self._set_move_head(True)

    # ------------------------------------------------------------------
    # go_abs
    # ------------------------------------------------------------------

    def go_abs_async(
        self,
        goal: Pose2D,
        goal_distance=None,
        motion_synth=False,
        near_goal_callback=None,
        feedback_callback=None,
        omni_goal_yaw_align=None,
        min_reach_goal_dist=None,
        accept_timeout: float = 5.0,
    ) -> bool:
        """Send an absolute goal and return once mvn_pln has accepted it.

        Returns True if the goal was accepted. Poll ``nav_status`` /
        ``is_navigating`` afterwards, and call ``wait_for_result()`` to block
        until the run ends (that is also what restores gaze/move_head and
        publishes the stop on failure).
        """
        self.get_logger().info(
            f'NavModule.->Go Absolute Goal(Action): x={goal.x}, y={goal.y}, theta={goal.theta}'
        )

        self.robot_stop = False
        self._near_goal_callback = near_goal_callback
        self._feedback_callback = feedback_callback

        # Per-goal opt-in/out of the last-mile omni goal-yaw alignment. None
        # leaves the simple_move launch default untouched.
        if omni_goal_yaw_align is not None:
            self._set_omni_goal_yaw_align(bool(omni_goal_yaw_align))

        if not motion_synth:
            self.motion_synth_start_pose = None
            self.motion_synth_end_pose = None

        # goal_distance (stop-short) is now honored server-side by mvn_pln: it
        # halts the robot and succeeds within goal_distance of the goal, so the
        # client only forwards the value and waits for the action result.
        if not self.send_nav_action_goal(
            goal,
            goal_distance=(
                goal_distance if goal_distance and goal_distance > 0 else 0.0),
            min_reach_goal_dist=(
                min_reach_goal_dist
                if min_reach_goal_dist and min_reach_goal_dist > 0
                else 0.0),
        ):
            return False

        if not self._goal_response_event.wait(timeout=accept_timeout):
            self.get_logger().warn(
                'NavModule.->Timeout waiting for PumasNav goal response')
            return False

        return self._goal_handle is not None

    def wait_for_result(self, timeout: float = 0.0) -> bool:
        """Block until the in-flight goal terminates.

        timeout: seconds; 0 (or None) waits forever, matching go_abs/go_rel.
        Returns the plain bool the existing callers expect; the reason lives in
        ``nav_status``.
        """
        deadline = None
        if timeout:
            deadline = self._timeout_now() + float(timeout)

        result = False
        timed_out = False
        while rclpy.ok() and not self.robot_stop:
            if self._result_event.wait(timeout=0.1):
                result = self._action_success
                break
            if deadline is not None and self._timeout_now() >= deadline:
                timed_out = True
                break

        if self.robot_stop:
            self.cancel_nav_action()
            self._set_client_error(CLIENT_ERROR_STOP_SIGNAL)
            result = False
        elif timed_out:
            self.get_logger().warn('NavModule.->Timeout waiting for PumasNav result')
            self.cancel_nav_action()
            self._set_client_error(CLIENT_ERROR_TIMEOUT)
            with self._status_lock:
                self._action_done = True
            result = False

        if not result:
            self.handle_robot_stop()

        # Ensure gaze is cancelled and move_head restored after nav ends
        if self._gaze_active:
            self.get_logger().info(
                'NavModule.->Nav ended: canceling gaze (fallback), restoring move_head'
            )
            self.cancel_gaze()
            self._set_move_head(True)

        return result

    def go_abs(
        self,
        goal: Pose2D,
        timeout,
        goal_distance=None,
        motion_synth=False,
        near_goal_callback=None,
        feedback_callback=None,
        omni_goal_yaw_align=None,
        min_reach_goal_dist=None,
    ) -> bool:
        if not self.go_abs_async(
            goal,
            goal_distance=goal_distance,
            motion_synth=motion_synth,
            near_goal_callback=near_goal_callback,
            feedback_callback=feedback_callback,
            omni_goal_yaw_align=omni_goal_yaw_align,
            min_reach_goal_dist=min_reach_goal_dist,
        ):
            self.handle_robot_stop()
            return False

        return self.wait_for_result(timeout)

    # ------------------------------------------------------------------
    # go_rel
    # ------------------------------------------------------------------

    def go_rel(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, timeout: float = 0) -> bool:
        """Relative holonomic move in the robot (base) frame.

        x: forward(+)/back(-) [m], y: left(+)/right(-) [m], yaw: rotate [rad].
        navlib only publishes [x, y, yaw] on /simple_move/goal_rel_pose; the
        motion is controlled by simple_move. Waits for /simple_move/goal_reached
        and returns True on SUCCEEDED (False on ABORTED/timeout/stop).

        ``nav_status`` is populated too: state_name is 'GO_REL', server_status
        carries simple_move's raw actionlib_msgs/GoalStatus code, and goal holds
        the requested base-frame offset (not a map pose).
        """
        self.robot_stop = False
        self._move_done = False
        self._move_success = False

        self.reset_action_state()
        with self._status_lock:
            self._action_sent = True
            self._state_name = 'GO_REL'
            self._goal_pose2d = Pose2D(x=float(x), y=float(y), theta=float(yaw))

        # Opens the gate in _move_goal_reached_callback. Set after
        # reset_action_state() so a stale goal_reached still in flight from the
        # previous move cannot be mistaken for this one's.
        self._go_rel_active = True

        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(yaw)]
        self.pub_goal_rel_pose.publish(msg)
        self.get_logger().info(
            f'NavModule.->go_rel: x={x:.3f} m, y={y:.3f} m, yaw={yaw:.3f} rad')

        deadline = self._timeout_now() + float(timeout) if timeout else None

        result = False
        timed_out = False
        try:
            while rclpy.ok() and not self.robot_stop:
                if self._result_event.wait(timeout=0.1):
                    result = self._move_success
                    break
                if deadline is not None and self._timeout_now() >= deadline:
                    timed_out = True
                    break
        finally:
            # Close the gate however we leave, so mvn_pln's own "-1" handshakes
            # during a later go_abs/nav_goal are ignored again.
            self._go_rel_active = False

        if timed_out:
            self.get_logger().warn('NavModule.->go_rel: timeout waiting for goal_reached')
            self._set_client_error(CLIENT_ERROR_TIMEOUT)
            with self._status_lock:
                self._action_done = True

        if self.robot_stop:
            # Stop simple_move via /navigation/stop (it subscribes there).
            self.handle_robot_stop()
            self._set_client_error(CLIENT_ERROR_STOP_SIGNAL)
            with self._status_lock:
                self._action_done = True
            result = False

        return result

    # ------------------------------------------------------------------
    # nav_goal
    # ------------------------------------------------------------------

    def nav_goal_async(
        self,
        goal,
        motion_synth_pose=None,
        goal_distance=None,
        use_point_cloud=True,
        gaze_point=False,
        via_points=None,
        near_goal_callback=None,
        feedback_callback=None,
        omni_goal_yaw_align=None,
        min_reach_goal_dist=None,
        accept_timeout: float = 5.0,
    ) -> bool:
        """Non-blocking nav_goal: returns once mvn_pln accepted the goal.

        Poll ``nav_status`` while ``is_navigating``, then call
        ``wait_for_result()`` -- it performs the gaze/move_head teardown that
        the blocking ``nav_goal()`` does for you.
        """
        # No cancel first: mvn_pln preempts, so the new goal takes over the
        # running one directly (see initialize_before_new_goal).
        self.initialize_before_new_goal(send_stop=False)

        # Via points (list of Pose2D) the path should pass through in order.
        # None/empty -> normal start->goal planning.
        self.via_points = list(via_points) if via_points else None

        if motion_synth_pose is not None:
            self.get_logger().info('NavModule.->Motion Synth Nav Goal with Pose Config')
            self._set_use_point_cloud(False)

            if 'start' in motion_synth_pose:
                self.motion_synth_start_pose = motion_synth_pose['start']
            if 'goal' in motion_synth_pose:
                self.motion_synth_end_pose = motion_synth_pose['goal']
            self.motion_execution_time = float(
                motion_synth_pose.get('execution_time', 0.0))

        else:
            self.get_logger().info('NavModule.->Standard Nav Goal')
            if gaze_point is not False:
                self._set_use_point_cloud(False)
            else:
                self._set_use_point_cloud(use_point_cloud)
            self._send_default_arm_pose(include_head=(gaze_point is False))

        if gaze_point is not False:
            tf_name = gaze_point if isinstance(gaze_point, str) else ''
            restore_head = None
            if self.motion_synth_end_pose is not None:
                restore_head = (
                    self.motion_synth_end_pose.get('head_pan_joint', 0.0),
                    self.motion_synth_end_pose.get('head_tilt_joint', 0.0),
                )
            if self.start_gaze(tf_name, restore_head=restore_head):
                self._set_move_head(False)

        return self.go_abs_async(
            goal,
            goal_distance=goal_distance,
            motion_synth=True,
            near_goal_callback=near_goal_callback,
            feedback_callback=feedback_callback,
            omni_goal_yaw_align=omni_goal_yaw_align,
            min_reach_goal_dist=min_reach_goal_dist,
            accept_timeout=accept_timeout,
        )

    def nav_goal(
        self,
        goal,
        timeout,
        motion_synth_pose=None,
        goal_distance=None,
        use_point_cloud=True,
        gaze_point=False,
        via_points=None,
        near_goal_callback=None,
        feedback_callback=None,
        omni_goal_yaw_align=None,
        min_reach_goal_dist=None,
    ):
        if not self.nav_goal_async(
            goal,
            motion_synth_pose=motion_synth_pose,
            goal_distance=goal_distance,
            use_point_cloud=use_point_cloud,
            gaze_point=gaze_point,
            via_points=via_points,
            near_goal_callback=near_goal_callback,
            feedback_callback=feedback_callback,
            omni_goal_yaw_align=omni_goal_yaw_align,
            min_reach_goal_dist=min_reach_goal_dist,
        ):
            self.handle_robot_stop()
            if self._gaze_active:
                self.cancel_gaze()
                self._set_move_head(True)
            return False

        return self.wait_for_result(timeout)


if __name__ == '__main__':
    rclpy.init()
    nav = NavModule()

    goal = Pose2D(x=1.0, y=3.7, theta=0.0)
    goal = Pose2D(x=0.8, y=3.44, theta=0.0)
    start_pose = {
        'arm_lift_joint': 0.0,
        'arm_flex_joint': np.deg2rad(0.0),
        'arm_roll_joint': np.deg2rad(0.0),
        'wrist_flex_joint': np.deg2rad(-90.0),
        'wrist_roll_joint': 0.0,
        'head_pan_joint': 0.0,
        'head_tilt_joint': np.deg2rad(0.0),
    }
    goal_pose = {
        'arm_lift_joint': 0.4,
        'arm_flex_joint': np.deg2rad(-90.0),
        'arm_roll_joint': np.deg2rad(0.0),
        'wrist_flex_joint': np.deg2rad(-90.0),
        'wrist_roll_joint': 0.0,
        'head_pan_joint': 0.0,
        'head_tilt_joint': np.deg2rad(60.0),
    }
    ms_config = {
        'start': start_pose,
        'goal': goal_pose,
        'execution_time': 0.5,
    }
    ms_config = None

    # for gaze node test
    gaze_tf = 'gaze_point'
    gaze_tf = False

    via_points = [
        # Pose2D(x=2.6, y=3.9, theta=0.0),
        Pose2D(x=0.9, y=0.9, theta=0.0),
        # Pose2D(x=0.0, y=1.0, theta=0.0),
    ]
    # via_points = None

    goal = Pose2D(x=0.5, y=3.6, theta=0.0)
    goal = Pose2D(x=0.0, y=2.67, theta=3.14)
    goal = Pose2D(x=0.0, y=0.0, theta=0.0)


    # Async + polling: nav_status is readable while the robot is driving.
    accepted = nav.nav_goal_async(
        goal,
        motion_synth_pose=ms_config,
        goal_distance=None,
        gaze_point=gaze_tf,
        via_points=via_points,
    )
    if not accepted:
        nav.get_logger().error(
            f'NavStatus.->goal not accepted: client_error='
            f'"{nav.nav_status.client_error}"')
    else:
        while nav.is_navigating:
            st = nav.nav_status
            # proximity_criterion / min_reach_goal_dist / goal_distance ride
            # along on the feedback, so the settings mvn_pln is actually using
            # are readable right here next to the numbers they explain.
            nav.get_logger().info(
                f'NavStatus.->{st.state_name} rest={st.remaining_distance:.2f} '
                f'near={st.near_goal_reached} (radius {st.proximity_criterion:.2f}) '
                f'collision={st.collision_risk} '
                f'no_replan={st.min_reach_goal_dist:.2f} '
                f'stop_short={st.goal_distance:.2f} '
                f'srv="{st.server_status_text}"'
            )
            time.sleep(0.2)

        success = nav.wait_for_result(timeout=0)
        st = nav.nav_status
        nav.get_logger().info(
            f'NavStatus.->success={success} outcome={st.outcome_name} '
            f'final_state="{st.final_state_name}" msg="{st.message}" '
            f'client_error="{st.client_error}" elapsed={st.elapsed:.1f}s'
        )

        if st.outcome == PumasNav.Result.OUTCOME_NO_PATH:
            nav.get_logger().warn('NavStatus.->no path; try another goal')
        elif st.outcome == PumasNav.Result.OUTCOME_STOP_SHORT:
            nav.get_logger().info('NavStatus.->stopped short on purpose')

    # blocking form (unchanged API, still returns a plain bool)
    """
    success = nav.nav_goal(
        goal,
        motion_synth_pose=ms_config,
        timeout=0,
        goal_distance=None,
        gaze_point=gaze_tf,
        via_points=via_points,
        feedback_callback=lambda st: nav.get_logger().info(st.state_name),
    )
    if not success:
        nav.get_logger().warn(f'NavStatus.->{nav.nav_status.message}')
    """

    # test rel
    """
    ok = nav.go_rel(
        x=0.8,
        y=0.0,
        yaw=np.deg2rad(00),
        timeout=120,
    )
    """

    nav.shutdown()

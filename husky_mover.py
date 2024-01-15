#!/usr/bin/python3

import rospy, tf
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from docopt import docopt


class Position:
    """Position abstraction."""

    def __init__(self, x, y, err):
        self.x = x
        self.y = y

        # Allowed error to position.
        self.err = err


class Config:
    """Config abstraction. There should be no magical values anywhere."""

    def __init__(self):

        # Just the node name.
        self.node_name = "mover"

        # Own frequency. We're not linked to topic update frequency.
        # It makes sense to keep this reasonably high to ensure updates
        # have been applied.
        self.dt = 0.5

        # Wait for new update after waiting dt for change application.
        self.dt_new_update = 0.001

        # Allowed distance to target at end.
        self.target_position_err = 0.5

        # Husky move speed (twist.x).
        self.move_forward_speed = 1

        # Husky angular speed.
        self.turn_speed = 1

        # Max acceptable error for theta.
        self.theta_max_err = 0.5

        # Use to dampen theta error for larger distances.
        self.theta_dampen_by_distance_factor = 0.1


class Context:
    """Context to keep common vars and methods."""

    def __init__(self, twist_publisher, target_position=None, config=None):
        self.config = config or Config()

        # This is where we want to go.
        self.target_position = target_position

        # This is our twist publisher.
        self.twist_publisher = twist_publisher

        # Latest odometry from topic subscriber.
        self.latest_odometry = None

        # Latest distance to target calculated.
        self.distance_to_target = 1

        # Keep track of updates somewhat.
        # After waiting for positional change
        # we want an explicit way to tell that
        # a new(!) update has been applied.
        # This can get messy otherwise and 
        # we don't want to operate on old data.
        self.updated = False

    def set_odometry(self, odometry):
        """Set odometry in subscriber."""
        self.updated = True
        self.latest_odometry = odometry

    def latest_pose(self):
        """Get latest pose."""
        self.updated = False
        if self.latest_odometry:
            return self.latest_odometry.pose.pose


class Helpers:
    """Just a helper class. Kept in this file for simplicity.
    Minimise to see what's actually important."""

    @staticmethod
    def init_node(context):
        """Helper: init node."""
        rospy.init_node(context.config.node_name)

    @staticmethod
    def odometry_subscriber(context, odometry_data):
        """Helper: odometry subscriber callback."""
        context.set_odometry(odometry_data)

    @staticmethod
    def subscribe_to_odometry(context):
        """Helper: subscribe to odometry."""
        rospy.Subscriber(
            "/odometry/filtered", Odometry, 
            lambda odometry_data: Helpers.odometry_subscriber(context, odometry_data))

    @staticmethod
    def build_twist(x, ang_z):
        """Helper: build twist."""
        twist = Twist()
        twist.linear.x = x
        twist.angular.z = ang_z
        return twist

    @staticmethod
    def build_publisher(endpoint, cls, queue_size=1):
        """Helper: build publisher."""
        return rospy.Publisher(endpoint, cls, queue_size=queue_size)

    @staticmethod
    def build_twist_publisher(queue_size=1):
        """Helper: build twist publisher."""
        return Helpers.build_publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size=queue_size)

    @staticmethod
    def build_context(twist_publisher):
        """Helper: build context."""
        return Context(twist_publisher)

    @staticmethod
    def shutdown(reason):
        """Helper: shutdown."""
        print(reason)
        rospy.signal_shutdown(reason)


def angle(x, y, epsilon=0.001):
    """Calculate angle in [-pi, pi] range via arctan."""

    # Just to avoid div 0 errors.
    if x == 0:
        x = epsilon
    if abs(x) < epsilon:
        if x < 0:
            x = -epsilon
        else:
            x = epsilon
    if x > 0:
        return math.atan(y / x)
    else:
        t = math.atan(abs(y / x))
        if y >= 0:
            return math.pi - t
        else:
            return t - math.pi


def calculate_twist(context):
    """Calculate next twist to be applied."""

    config = context.config

    # These variables control our resulting movement.
    # move_forward moves our husky.
    # turn indicates which direction to turn in (r/s).
    move_forward = config.move_forward_speed
    turn = 0

    pose = context.latest_pose()
    if not pose:
        return None

    x = pose.position.x
    y = pose.position.y
    z = pose.orientation.z
    w = pose.orientation.w

    # Get target position.
    target_x = context.target_position.x
    target_y = context.target_position.y

    # Relative to husky.
    dx = target_x - x
    dy = target_y - y

    # Have we arrived? If so, echo and quit.
    distance = math.sqrt(dx * dx + dy * dy)
    context.distance_to_target = distance
    if distance < config.target_position_err:
        Helpers.shutdown(f"I've arrived. Distance to target: {distance:.4f}")

    else:

        # Need to transform current orientation from quaternion.
        # theta will be in range [-pi, pi], indicates polar orientation.
        (_, _, theta_actual) = tf.transformations.euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w])

        # Calculate target angle to target using arctan.
        # Range [-pi, pi]. This is where we want to go.
        # Since the husky moves during turning, we need
        # to recalculate after each partial turn.
        theta_target = angle(dx, dy)

        # Close enough.
        theta_err = min(config.theta_dampen_by_distance_factor * distance, config.theta_max_err)

        # I want to handle boundary conditions.
        # E.g. theta actual close to 0 and 2pi.
        # And pick theta_target closest to theta_actual.

        # Convert to [0, 2*pi].
        theta_actual += math.pi
        theta_target += math.pi

        # Closest might be target + pi.
        theta_target_2 = theta_target + 2 * math.pi
        if abs(theta_actual - theta_target_2) < abs(theta_actual - theta_target):
            theta_target = theta_target_2

        theta_diff = theta_target - theta_actual

        if abs(theta_diff) > theta_err:

            if theta_diff < 0:
                print("Turning right.")
                turn = -config.turn_speed
            else:
                print("Turning left.")
                turn = config.turn_speed
        else:
            print("Not turning.")

        print()
        print(f"distance:     {distance:04.2f}")
        print(f"theta_diff:   {theta_diff:04.2f}")
        print(f"theta_target: {theta_target:04.2f}")
        print(f"theta_actual: {theta_actual:04.2f}")
        return Helpers.build_twist(move_forward, turn)


if __name__ == "__main__":

    args = docopt(
        """Usage: husky_mover.py -- <target_x> <target_y>

        <target_x>      Specify target x coordinate.
        <target_y>      Specify target y coordinate.

        """)

    try:
        context = Helpers.build_context(Helpers.build_twist_publisher())
        config = context.config

        target_x = float(args['<target_x>'])
        target_y = float(args['<target_y>'])
        target_err = config.target_position_err

        Helpers.init_node(context)
        Helpers.subscribe_to_odometry(context)
        context.target_position = Position(target_x, target_y, target_err)

        i = 0
        while True:
            i += 1
            print("\n\n" + 20 * "-")
            print(f"i={i:06}\n")
            print("target:")
            print(f"  x: {target_x:.2f}")
            print(f"  y: {target_y:.2f}")
            print(context.latest_pose())
            print()

            twist = calculate_twist(context)
            if twist:
                context.twist_publisher.publish(twist)
            
            # Want to ensure updates during debugging.
            # We're linking update rate to distance.
            # Publish rate is adapted as we move closer.
            if context.distance_to_target > 2 * config.target_position_err:
                rospy.sleep(config.dt / context.distance_to_target)
            else:
                rospy.sleep(config.dt)

            context.updated = False
            while not context.updated:
                rospy.sleep(config.dt_new_update)

    except rospy.exceptions.ROSException:
        pass

    print("All done.")
    exit(0)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Objects;


/** Keeps track of the robot's desired path and makes it follow it accurately.
 */
public class NavigationTeleOP {
    public enum rotationDirection {CLOCKWISE, COUNTERCLOCKWISE};

    public static enum Action {NONE, SLIDES_LOW, SLIDES_HIGH,};
    //Makes actions of the Robot that can be used anywhere within the folder.

    //**AUTONOMOUS CONSTANTS**
    public enum MovementMode {FORWARD_ONLY, BACKWARD_ONLY, STRAFE_LEFT, STRAFE_RIGHT};

    //Movements within the robot Autonomous Mode
    public final double STRAFE_ACCELERATION = 0.1; //Number indicated Inches per second squared
    public final double ROTATE_ACCELERATION = 0.1; //Number indicated Radians per second squared
    public final double SPEED_FACTOR = 0.7; //Speed of the robot when all motors are set to full power
    static final double STRAFE_RAMP_DISTANCE = 4; //Number indicated Inches
    //static final double ROTATION_RAMP_DISTANCE = Math.PI / 2; //NOT BEING USED
    static final double MIN_STRAFE_POWER = 0.3; //Sets the strafe power to 3/10th power
    static final double MAX_STRAFE_POWER = 0.5; //Sets the strafe power to 5/10th power
    static final double STRAFE_CORRECITON_POWER = 0.3; //idk what this means
    static final double STRAFE_SLOW = 0.1; //idk what this means
    static final double MAX_ROTATION_POWER = 0.3; //sets the rotation power to 3/10th, power
    static final double MIN_ROTATION_POWER = 0.03; //sets the rotation power to 3/100th power (why?)
    static final double ROTATION_CORRECTION_POWER = 0.04; //idk what this means

    // Accepted amounts of deviation between the robot's desired position and actual position.
    static final double EPSILON_ANGLE = 0.35;
    //   static final int NUM_CHECK_FRAMES = 5; //The number of frames to wait after a rotate or travelLinear call in order to check for movement from momentum.

    //Distances between where the robot extends/retracts the linear slides and where it opens the claw.
    static final double ROTATION_TIME = 1050; //???
    static final double FLOAT_EPSILON = 0.001; //????
    // The number of frames to wait after a rotate or travelLinear call in order to check for movement from momentum.
    static final int NUM_CHECK_FRAMES = 5;
    static final double JOYSTICK_DEAD_ZONE_SIZE = 0.08; //Sets the joystick deadzone to 0.08.
    static final double EPSILON_LOC = 10;


    //**TELEOP CONSTANTS**
    static final double MOVEMENT_MAX_POWER = 1; //Sets the maximum power to full power. (Full power is between 0 - 1)
    static final double ROTATION_POWER = 0.5; //Sets the maximum rotation power 1/2 full power
    static final double REDUCED_ROTATION_POWER = 0.2; //Lets the minimum rotation power to 1/5th full power
    static final double SLOW_MOVEMENT_SCALE_FACTOR = 0.3; //idk what this means
    static final double MEDIUM_MOVEMENT_SCALE_FACTOR = 0.6; //idk what this means


    //**INSTANCE ATTRIBUTES**//
    public double[] wheel_speeds = {0.95, 1, -1, -0.97}; //Back left, Back right, Front left, Front right. Temporary Note: currently FR from -0.90 to -0.92
    public double strafePower; //This is for Tele-Op ONLY.

    /*
     First position in this ArrayList is the first position that robot is planning to go to.
     This condition must be maintained (positions should be deleted as the robot travels)
     NOTE: a position is both a location and a rotation.
     NOTE: this can be changed to a stack later if appropriate (not necessary for speed, just correctness).
     */
    public ArrayList<Position> path; //List of positions that the robot will go into WHEN IT IS IN AUTOMOTOUS MODE.
    public int pathIndex; //Index of the path array list.

    /**
     * @param path          positions of where the robot is traveling to in auton
     * @param allianceColor alliance color on what team we are on (which is either red or blue)
     * @param startingSide  the starting side on where our robot is starting from (on the field)
     * @param movementMode  the movement within the robot
     */
    public NavigationTeleOP(ArrayList<Position> path, RobotManager.AllianceColor allianceColor, RobotManager.StartingSide startingSide, MovementMode movementMode) {
        this.path = path;
        pathIndex = 0;
    }

    /** Updates the strafe power according to movement mode and gamepad 1 left trigger.
     * |Teleop| |Non Blocking|
     * @param gamepads
     * @param robot
     */
    public void updateStrafePower(GamepadWrapper gamepads, Robot robot) {

        AnalogValues analogValues = gamepads.getAnalogValues();
        //Limits the output values between 0 - 1. 0 = no power, 1 = full power
        double distance = Range.clip(Math.sqrt(Math.pow(analogValues.gamepad1LeftStickX, 2) + Math.pow(analogValues.gamepad1LeftStickY, 2)), 0, 1);


        if (distance <= JOYSTICK_DEAD_ZONE_SIZE) {
            strafePower = SLOW_MOVEMENT_SCALE_FACTOR; //Set as 0.3 (3/10th full power)
        } else {
            strafePower = distance + MOVEMENT_MAX_POWER; //Set as 1 (full power)
        }
        //Pre-sets robot slide states at what speed.
        if (robot.desiredSlidesState == Robot.SlidesState.HIGH && robot.slides.getPower() == 0) {
            strafePower *= SLOW_MOVEMENT_SCALE_FACTOR; //Set as o.3
        } else if (robot.desiredSlidesState == Robot.SlidesState.MEDIUM && robot.slides.getPower() == 0) {
            strafePower *= SLOW_MOVEMENT_SCALE_FACTOR; //Set as o.3
        } else if (robot.desiredSlidesState == Robot.SlidesState.LOW && robot.slides.getPower() == 0) {
            strafePower *= SLOW_MOVEMENT_SCALE_FACTOR; //Set as o.3
        }
    }

    /**
     * DEGREE      ALT + 2 4 8
     * Moves the robot straight in one of the cardinal directions or at a 45 degree angle.
     * NOTE: ALL CONTROLLER MOVEMENTS ARE USING A PS5 CONTROLLER.
     * |Teleop| |Non Blocking|
     *
     * @param forward  moving robot forward. Using the UP arrow on the DPAD
     * @param backward moving robot backwards. Using the DOWN arrow on the DPAD
     * @param left     moving robot to the left. Using the LEFT arrow on the DPAD
     * @param right    moving robot to the right. Using the RIGHT arrow on the DPAD
     * @param robot
     * @return whether any of the DPAD buttons were pressed
     */
    public boolean moveStraight(GamepadWrapper gamepads, boolean forward, boolean backward, boolean left, boolean right, Robot robot) {
        double direction;
        if (forward || backward) {
            if (left) {//moves left at 45° (or Northwest)
                direction = -Math.PI * 0.25;
            } else if (right) { //moves right at 45° (or Northeast)
                direction = Math.PI * 0.75;
            } else {//moving forward
                direction = -Math.PI * 0.5;
            }
            if (backward) { //invert the forward to just backwards
                direction *= -1;
            }
        } else if (left) { //default direction. Set as 0
            direction = 0;
        } else if (right) {
            direction = Math.PI;
        } else {
            return false;
        }
        setDriveMotorPowers(direction, strafePower, 0.0, robot, false);
        return true;
    }

    /** Changes drivetrain motor inputs based off the controller inputs
     * |Teleop| |Non Blocking|
     * @param gamepads
     * @param robot
     */
    public void moveJoystick(GamepadWrapper gamepads, Robot robot) {
        //Uses left joystick to go forward, and right joystick to turn.
        // NOTE: right-side drivetrain motor inputs don't have to be negated because their directions will be reversed
        //       upon initialization.

        double turn = gamepads.gamepad1.right_stick_x;
        double rotationPower = ROTATION_POWER;
        if (Math.abs(turn) < JOYSTICK_DEAD_ZONE_SIZE) {
            turn = 0;
        }
        if (gamepads.getButtonState(GamepadWrapper.DriverAction.REDUCED_CLOCKWISE)) {
            rotationPower = REDUCED_ROTATION_POWER;
            turn = -1;
        }
        if (gamepads.getButtonState(GamepadWrapper.DriverAction.REDUCED_COUNTER_CLOCKWISE)) {
            rotationPower = REDUCED_ROTATION_POWER;
            turn = -1;
        }
        double moveDirection = Math.atan2(gamepads.gamepad1.left_stick_y, gamepads.gamepad1.left_stick_x);
        if (Math.abs(moveDirection) < Math.PI / 12) {
            moveDirection = 0;
        } else if (Math.abs(moveDirection - Math.PI / 2) < Math.PI / 12) {
            moveDirection = Math.PI / 2;
        } else if (Math.abs(moveDirection - Math.PI) % Math.PI < Math.PI / 12) {
            moveDirection = Math.PI;
        } else if (Math.abs(moveDirection + Math.PI / 2) < Math.PI / 12) {
            moveDirection = Math.PI / 2;
        } else {
            moveDirection = moveDirection;
        }

        setDriveMotorPowers(moveDirection, strafePower, turn * rotationPower, robot, false);
    }

    /** Sets drive motor powers to make the robot move a certain way.
     *
     *  @param strafeDirection the direction in which the robot should strafe.
     *  @param power the speed at which the robot should strafe. Must be in the interval [-1, 1]. Set this to zero if
     *               you only want the robot to rotate.
     *  @param turn the speed at which the robot should rotate (clockwise). Must be in the interval [-1, 1]. Set this to
     *              zero if you only want the robot to strafe.
     */
    public void setDriveMotorPowers(double strafeDirection, double power, double turn, Robot robot, boolean debug) {
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.telemetry.addData("turn %.2f", turn);
        robot.telemetry.addData("current strafe direction", strafeDirection);
        if (Math.abs(power - 0) < FLOAT_EPSILON && Math.abs(turn - 0) < FLOAT_EPSILON) {
            stopMovement(robot);
            robot.telemetry.addData("stopping", "YES");
        } else {
            robot.telemetry.addData("stopping", "NO");
        }
        double sinMoveDirection = Math.sin(strafeDirection);
        double cosMoveDirection = Math.cos(strafeDirection);

        double powerSet1 = sinMoveDirection + cosMoveDirection;
        double powerSet2 = sinMoveDirection - cosMoveDirection;
        double[] rawPowers = scaleRange(powerSet1, powerSet2);
    }

    /**
     * @param robot
     */
    public void stopMovement(Robot robot) {
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.frontLeft.setPower(0.0);
        robot.frontRight.setPower(0.0);
        robot.rearLeft.setPower(0.0);
        robot.rearRight.setPower(0.0);
    }

    /**preserves the ratio between a and b while restricting them to the range [-1, 1]
     * @param a value to be scaled
     * @param b value to be scaled
     * @return an array containing the scaled versions of a and b
     */

    public double[] scaleRange(double a, double b) {
        double max = Math.max(Math.abs(a), Math.abs(b));
        return new double[] {a / max, b / max};
    }


}
//Coders: Tyler M.
//Alumni Help: Stephen D.
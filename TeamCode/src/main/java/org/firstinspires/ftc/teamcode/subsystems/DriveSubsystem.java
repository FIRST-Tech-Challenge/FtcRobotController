package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * Mecanum Drive Subsystem
 * Owns the four drive motors. Only this class touches them in TeleOp.
 * OpModes tell it where to go with drive(forward, strafe, turn).
 *
 * WHEN TO USE IT
 *   This subsystem is for TELEOP. In AUTO, Pedro Pathing's Follower drives these same four
 *   motors, and only ONE class may control a motor. Never use both in the same OpMode.
 *   (Pedro's motor names and directions must match the ones in this file.)
 *
 * THE CONVENTION (the same one Pedro Pathing, Pinpoint and our turret use)
 *   Looking down from above, with the robot's front at the top:
 *
 *                 +forward
 *                    ^
 *      +strafe  <--  O  -->  -strafe        (strafe LEFT is POSITIVE)
 *      (left)        |        (right)
 *
 *   +turn spins the robot COUNTERCLOCKWISE (toward the left). -turn spins it clockwise.
 *   Every input is a number from -1.0 to 1.0.
 *   The controller's sticks use the opposite signs, so the OpMode flips them (see DriveAlpha).
 *
 * HOW THE WHEELS MAKE THE MOVES (mecanum wheels, rollers in an X when viewed from above)
 *   leftFront  = forward - strafe - turn
 *   rightFront = forward + strafe + turn
 *   leftRear   = forward + strafe - turn
 *   rightRear  = forward - strafe + turn
 *   If any wheel would be asked for more than 100%, all four wheels are scaled down together,
 *   so the robot still moves in the direction the driver asked for, just a little slower.
 *
 * ACCELERATION LIMIT (so a slammed stick cannot spike the battery)
 *   When four drive motors are told to go from stopped to full power in one instant, they all pull
 *   a huge burst of current at once. The battery voltage sags for a moment, and a sag is the
 *   usual cause of a hub, the Pinpoint or the camera blinking. A tall robot can also tip when it
 *   lurches. So drive() lets each wheel's power RISE only as fast as MAX_POWER_CHANGE_PER_SECOND
 *   (0 to full takes about 0.2 seconds, which a driver barely feels). SLOWING DOWN is never
 *   limited: letting go of the stick stops the robot at once, and a wheel that has to reverse
 *   goes to zero first and then ramps up the other way. LIMIT_ACCELERATION switches it off.
 *
 * STATE (what we told it to do) is the last drive() inputs, the power scale, and the wheel powers.
 * There are no sensors here, so there is no STATUS and no update() to call.
 *
 * TWO WAYS TO DRIVE
 *   drive(forward, strafe, turn)  is ROBOT-CENTRIC: "forward" means the way the robot is facing.
 *   driveFieldCentric(forward, strafe, turn, headingDegrees)  is FIELD-CENTRIC: "forward" means
 *   AWAY FROM THE DRIVER, no matter which way the robot is turned. It needs the robot's heading,
 *   which the OpMode gets from the Pinpoint and hands in as a number. This class never reads the
 *   Pinpoint itself (a subsystem owns only its own hardware).
 *
 * Robot configuration (on the Driver Station) for ALL three robots. The names MUST match:
 *   Motor name: left_front
 *   Motor name: left_rear
 *   Motor name: right_front
 *   Motor name: right_rear
 */
public class DriveSubsystem {

    // ------------------------------------------------------------------
    // Hardware names. These MUST match the robot configuration exactly.
    //
    // The four names and the four directions below are PUBLIC on purpose. In Autonomous,
    // pedro/Constants.java reads them to set up Pedro's drivetrain, so TeleOp and Auto always
    // use the SAME motors and directions and each one is typed in only ONE place: here.
    // ------------------------------------------------------------------
    public static final String LEFT_FRONT_MOTOR_NAME = "left_front";
    public static final String LEFT_REAR_MOTOR_NAME = "left_rear";
    public static final String RIGHT_FRONT_MOTOR_NAME = "right_front";
    public static final String RIGHT_REAR_MOTOR_NAME = "right_rear";

    // ------------------------------------------------------------------
    // Directions. TEST THESE on the robot with the wheels off the floor!
    // ------------------------------------------------------------------
    // The motors on the two sides face opposite ways, so one side usually needs REVERSE.
    // These start the same as the FTC sample code, which assumes the wheels turn the same way
    // as the motor shaft.
    // Test: each wheel must push the robot FORWARD when it is given positive power.
    // If a wheel spins backward, flip THAT motor's line between FORWARD and REVERSE.
    public static final DcMotor.Direction LEFT_FRONT_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction LEFT_REAR_DIRECTION = DcMotor.Direction.REVERSE;
    public static final DcMotor.Direction RIGHT_FRONT_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_REAR_DIRECTION = DcMotor.Direction.FORWARD;

    // ------------------------------------------------------------------
    // Driving settings
    // ------------------------------------------------------------------
    // BRAKE makes the robot stop quickly and lets it hold its spot, which helps when parking.
    // But the robot is TALL (up to 29 inches, with a turret and shooter on top), so stopping
    // very suddenly from full speed could tip it. If that happens, try FLOAT. (The limit on how
    // fast the power can RISE is below. It does not slow down stopping.)
    private static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
            DcMotor.ZeroPowerBehavior.BRAKE;

    // The most power any wheel will ever get (0.0 to 1.0). 1.0 is full speed.
    // Lower this for the first tests, or if the robot is too fast for new drivers.
    private static final double MAX_DRIVE_POWER = 1.0;

    // Slow mode makes the robot move at this fraction of full speed (0.4 = 40%), for careful
    // lining up. Raise it if slow mode feels too slow, lower it if it is still too fast.
    private static final double SLOW_MODE_POWER_SCALE = 0.4;

    // The speed fraction when slow mode is OFF. 1.0 means full speed.
    private static final double NORMAL_POWER_SCALE = 1.0;

    // ------------------------------------------------------------------
    // Acceleration limit (see ACCELERATION LIMIT at the top of this file)
    // ------------------------------------------------------------------
    // true = a wheel's power may only RISE as fast as MAX_POWER_CHANGE_PER_SECOND allows.
    // false = the wheels get exactly what the driver asked for, at once (how it worked before).
    private static final boolean LIMIT_ACCELERATION = true;

    // How fast one wheel's power may rise, in "power" per second (power goes from 0.0 to 1.0).
    //   5.0 -> 0 to full takes 0.20 seconds. Gentle. Most drivers will not notice it.
    //   10.0 -> 0.10 seconds. Barely there.
    //   2.5 -> 0.40 seconds. Clearly softer, and the robot feels heavy.
    // TODO (issue #88): Let the driver try it. If the robot feels sluggish, raise this. If the battery
    // still dips (see the POWER lines on the screen), or the robot still tips, lower it.
    private static final double MAX_POWER_CHANGE_PER_SECOND = 5.0;

    // The most time one step may count for, in seconds. If a loop stalls for a moment, we do not
    // want the next loop to make up for it with one big jump in power.
    private static final double MAX_RAMP_STEP_SECONDS = 0.05;

    // ------------------------------------------------------------------
    // Hardware and data. Only this class changes these.
    // ------------------------------------------------------------------
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;

    // 1.0 is full speed. Slow mode lowers it (see setSlowMode).
    private double powerScale = NORMAL_POWER_SCALE;

    // Times the steps of the acceleration limit: how long since drive() was last called.
    private final ElapsedTime accelerationTimer = new ElapsedTime();

    // What we last asked for, kept so telemetry can show it.
    private double forwardInput = 0.0;
    private double strafeInput = 0.0;
    private double turnInput = 0.0;
    private double leftFrontPower = 0.0;
    private double rightFrontPower = 0.0;
    private double leftRearPower = 0.0;
    private double rightRearPower = 0.0;

    /*
     * Setup. Gets the four motors from the robot configuration and sets them up.
     * The robot starts stopped.
     */
    public DriveSubsystem(HardwareMap hardwareMap) {
        // The names MUST match the robot configuration exactly.
        leftFrontMotor = hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR_NAME);
        leftRearMotor = hardwareMap.get(DcMotor.class, LEFT_REAR_MOTOR_NAME);
        rightFrontMotor = hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR_NAME);
        rightRearMotor = hardwareMap.get(DcMotor.class, RIGHT_REAR_MOTOR_NAME);

        setUpHardware();
    }

    /*
     * Sets directions and modes, then makes sure every wheel is stopped.
     */
    private void setUpHardware() {
        leftFrontMotor.setDirection(LEFT_FRONT_DIRECTION);
        leftRearMotor.setDirection(LEFT_REAR_DIRECTION);
        rightFrontMotor.setDirection(RIGHT_FRONT_DIRECTION);
        rightRearMotor.setDirection(RIGHT_REAR_DIRECTION);

        // We control the wheels with raw power. Pinpoint keeps track of where the robot is,
        // so we do not need the drive motors' encoders for anything here.
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        leftRearMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        rightFrontMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        rightRearMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    // ==================================================================
    // ACTIONS: the OpMode tells the drivetrain what to do
    // ==================================================================

    /*
     * Drive the robot. Each number is from -1.0 to 1.0:
     *   forward: + drives forward,        - drives backward
     *   strafe:  + slides LEFT,           - slides right
     *   turn:    + spins COUNTERCLOCKWISE (left), - spins clockwise (right)
     * Call this every loop. Send all zeros (or call stop()) to stop.
     *
     * The wheel powers are worked out first, then scaled down together if any wheel would be
     * over 100%, and then slowed by the power scale (slow mode) and MAX_DRIVE_POWER.
     */
    public void drive(double forward, double strafe, double turn) {
        forwardInput = Range.clip(forward, -1.0, 1.0);
        strafeInput = Range.clip(strafe, -1.0, 1.0);
        turnInput = Range.clip(turn, -1.0, 1.0);

        // Combine the three moves into a power for each wheel.
        double leftFront = forwardInput - strafeInput - turnInput;
        double rightFront = forwardInput + strafeInput + turnInput;
        double leftRear = forwardInput + strafeInput - turnInput;
        double rightRear = forwardInput - strafeInput + turnInput;

        // Find the biggest power any wheel was asked for.
        double largestPower = Math.abs(leftFront);
        largestPower = Math.max(largestPower, Math.abs(rightFront));
        largestPower = Math.max(largestPower, Math.abs(leftRear));
        largestPower = Math.max(largestPower, Math.abs(rightRear));

        // If a wheel was asked for more than 100%, shrink ALL four by the same amount.
        // That keeps the wheels in the same proportion, so the robot still goes where we asked.
        if (largestPower > 1.0) {
            leftFront = leftFront / largestPower;
            rightFront = rightFront / largestPower;
            leftRear = leftRear / largestPower;
            rightRear = rightRear / largestPower;
        }

        // Apply slow mode and the top speed limit. These are the powers the driver WANTS.
        double scale = powerScale * MAX_DRIVE_POWER;
        double leftFrontWanted = leftFront * scale;
        double rightFrontWanted = rightFront * scale;
        double leftRearWanted = leftRear * scale;
        double rightRearWanted = rightRear * scale;

        // How long since the last call, so the limit is the same speed no matter how long a loop
        // takes. The timer always runs, even when the limit is off.
        double secondsSinceLastDrive = Math.min(accelerationTimer.seconds(), MAX_RAMP_STEP_SECONDS);
        accelerationTimer.reset();

        // Do not let any wheel's power jump up faster than the limit allows. The wheel powers we
        // sent LAST loop (leftFrontPower and the others) are where each wheel starts from.
        if (LIMIT_ACCELERATION) {
            double biggestStep = MAX_POWER_CHANGE_PER_SECOND * secondsSinceLastDrive;
            leftFrontWanted = rampToward(leftFrontPower, leftFrontWanted, biggestStep);
            rightFrontWanted = rampToward(rightFrontPower, rightFrontWanted, biggestStep);
            leftRearWanted = rampToward(leftRearPower, leftRearWanted, biggestStep);
            rightRearWanted = rampToward(rightRearPower, rightRearWanted, biggestStep);
        }

        setWheelPowers(leftFrontWanted, rightFrontWanted, leftRearWanted, rightRearWanted);
    }

    /*
     * One wheel's step of the acceleration limit. It answers: "the wheel is at currentPower and the
     * driver wants wantedPower, so what power do we send now?"
     *   - Slowing down, or stopping: allowed AT ONCE. (Stopping must never feel slow.)
     *   - Speeding up: only biggestStep more than last time.
     *   - Wanting to go the OTHER way: the wheel goes to zero first, then speeds up the other way,
     *     so the motor is never asked to flip from forward to backward in one jump.
     */
    private double rampToward(double currentPower, double wantedPower, double biggestStep) {
        double startingPower = currentPower;
        // Stopped, or the wheel wants to reverse: start from zero.
        if (startingPower * wantedPower <= 0.0) {
            startingPower = 0.0;
        }
        // Slowing down (or the same speed): no limit.
        if (Math.abs(wantedPower) <= Math.abs(startingPower)) {
            return wantedPower;
        }
        // Speeding up: take a step, but never past what the driver wants.
        double step = Math.min(Math.abs(wantedPower) - Math.abs(startingPower), biggestStep);
        return startingPower + Math.signum(wantedPower) * step;
    }

    /*
     * Drive FIELD-CENTRIC. The forward and strafe numbers are from the DRIVER's point of view:
     *   forward: + moves the robot AWAY from the driver,   - toward the driver
     *   strafe:  + moves the robot to the driver's LEFT,   - to the driver's right
     *   turn:    + spins COUNTERCLOCKWISE (left), - spins clockwise (right). Turning is the same in
     *            both ways of driving, so it is passed straight through.
     * headingDegrees is how far the robot is turned from "facing away from the driver", with
     * COUNTERCLOCKWISE (left) positive, the same as the Pinpoint's heading. When the robot faces
     * away from the driver the heading is 0 and this drives exactly like drive(). The OpMode works
     * out the heading (the Pinpoint's heading plus any offset), so this class does not need to know
     * where the driver is standing.
     * Call this every loop. Send all zeros (or call stop()) to stop.
     *
     * HOW IT WORKS: the stick direction is turned by MINUS the robot's heading, so that
     * "away from the driver" becomes a direction measured from the robot's own front. Then the
     * ordinary drive() does the rest, so the wheel math, the slow mode and the top speed limit are
     * exactly the same as robot-centric driving.
     *   robotForward =  forward * cos(heading) + strafe * sin(heading)
     *   robotStrafe  = -forward * sin(heading) + strafe * cos(heading)
     * CHECK: the robot is turned 90 degrees to the left (heading +90) and the driver pushes
     * straight ahead (forward 1, strafe 0). Then robotForward = 0 and robotStrafe = -1: the robot
     * slides to ITS right, which is away from the driver. Correct.
     */
    public void driveFieldCentric(double forward, double strafe, double turn, double headingDegrees) {
        double headingRadians = Math.toRadians(headingDegrees);
        double cosHeading = Math.cos(headingRadians);
        double sinHeading = Math.sin(headingRadians);

        double robotForward = forward * cosHeading + strafe * sinHeading;
        double robotStrafe = -forward * sinHeading + strafe * cosHeading;

        // Turning a stick that is pushed into a corner can make one number bigger than 1.0 (the
        // sticks are square, and turning a square makes it stick out). drive() would cut each
        // number to 1.0 on its own, which bends the direction. So shrink both together first.
        double largest = Math.max(Math.abs(robotForward), Math.abs(robotStrafe));
        if (largest > 1.0) {
            robotForward = robotForward / largest;
            robotStrafe = robotStrafe / largest;
        }

        drive(robotForward, robotStrafe, turn);
    }

    /*
     * How hard the robot is really being told to spin right now, from -1.0 to 1.0. + is
     * COUNTERCLOCKWISE (left), - is clockwise (right), the same as drive(). It is worked out from
     * the wheel powers we actually sent, so it includes slow mode, MAX_DRIVE_POWER, the acceleration
     * limit (the spin takes a moment to build up, and this number builds up with it), and the
     * small shrink when a wheel would pass 100%. The TeleOp uses it to tell the turret how fast to
     * follow the spin, so the turret should not start turning before the robot does.
     *
     * HOW: from the wheel math at the top, (rightFront - leftFront + rightRear - leftRear) adds up
     * to 4 times the turn, and the forward and strafe parts cancel out.
     */
    public double getTurnPower() {
        return (rightFrontPower - leftFrontPower + rightRearPower - leftRearPower) / 4.0;
    }

    /*
     * Slow mode. true slows every wheel to SLOW_MODE_POWER_SCALE of full speed, for careful
     * lining up. false goes back to full speed. It stays set until you change it, so call it
     * every loop with the driver's button (or only when the driver changes it).
     */
    public void setSlowMode(boolean slowModeOn) {
        if (slowModeOn) {
            powerScale = SLOW_MODE_POWER_SCALE;
        } else {
            powerScale = NORMAL_POWER_SCALE;
        }
    }

    /*
     * Send a RAW power (-1.0 to 1.0) to each wheel. This skips the mecanum math, the
     * normalizing, and slow mode. drive() uses it to send its answer to the motors, and the
     * wheel test in DriveAlpha uses it to spin one wheel at a time.
     * Positive power should push the robot FORWARD (see the direction notes at the top).
     */
    public void setWheelPowers(double leftFront, double rightFront,
                               double leftRear, double rightRear) {
        leftFrontPower = Range.clip(leftFront, -1.0, 1.0);
        rightFrontPower = Range.clip(rightFront, -1.0, 1.0);
        leftRearPower = Range.clip(leftRear, -1.0, 1.0);
        rightRearPower = Range.clip(rightRear, -1.0, 1.0);

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightRearMotor.setPower(rightRearPower);
    }

    /*
     * Stop all four wheels. Safe to call every loop.
     */
    public void stop() {
        forwardInput = 0.0;
        strafeInput = 0.0;
        turnInput = 0.0;
        setWheelPowers(0.0, 0.0, 0.0, 0.0);
    }

    // ==================================================================
    // QUESTIONS: the OpMode asks the drivetrain what is happening
    // ==================================================================

    /*
     * Adds the drivetrain's own data to the telemetry. The SDK sends telemetry to the
     * Driver Station after every loop(), so the OpMode has nothing else to call.
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Drive inputs", "forward %.2f   strafe %.2f   turn %.2f",
                forwardInput, strafeInput, turnInput);
        telemetry.addData("Power scale", "%.0f%%", powerScale * 100);
        if (LIMIT_ACCELERATION) {
            telemetry.addData("Acceleration limit", "ON  (0 to full takes %.2f s)", 1.0 / MAX_POWER_CHANGE_PER_SECOND);
        } else {
            telemetry.addData("Acceleration limit", "off");
        }
        telemetry.addData("Front left/right", "%.2f, %.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Rear  left/right", "%.2f, %.2f", leftRearPower, rightRearPower);
    }
}

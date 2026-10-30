/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.robot.opmode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot with Mecanum
 * Wheels for the 2026-2027 FIRST® Tech Challenge. On top of a mecanum wheel drivetrain, it uses
 * one motor driving an intake roller, two servos which pull elements out of corners, and a high-speed
 * launcher motor.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 *
 * Pedro Pathing debug telemetry: the bottom of the telemetry shows what Pedro Pathing's localizer
 * (the Pinpoint) measures, checks it against the way the driver is steering, and compares the
 * robot's top speed with the Foresight tuning in Constants. Pedro only watches here; the sticks
 * still drive the motors directly. It also runs during INIT, so you can push the robot by hand and
 * watch the pose change. Press BACK to reset the pose and the checks.
 */

@TeleOp(name = "Snap Teleop", group = "_Compete")
//@Disabled
public class TeleopWSnap extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;


    /*
     * These two variables are used to control the velocity of the launcher motor.
     * They are both in encoder ticks per second. The motors we use in the FIRST Tech Challenge
     * have encoders with a resolution of 28 ticks per revolution. We can convert this to RPM
     * by dividing the value by 28, to get to revolutions per second, before multiplying by 60
     * to get revolutions per minute.
     * We pass the target velocity variable to our motor to set the goal. We use the min velocity
     * in the launch() function to only run the windmill servo when the motor is spinning fast
     * enough to make a successful throw.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1250; //2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 1200; //2571 RPM
    public final int LAUNCHER_REVERSE_VELOCITY = 625; // half of target; reverse speed for unjamming


    /*
     * These four variables store the power we need to apply to the motors. In other cases, we may
     * choose to declare these variables inside the mecanumDrive() function, instead we declare them
     * here so that we can access them in our main loop for telemetry.
     */
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // Create a variable to set to the intake.
    double intakePower;

    /*
     * Heading snap. Tapping X turns the robot to face 90 degrees and tapping B turns it to face
     * 270 degrees, measured by the Pinpoint (counterclockwise from the way the robot faced at INIT,
     * or when BACK was last pressed). The driver can still drive and strafe during the turn.
     * Moving the right stick cancels the snap. snapTarget is null when no snap is running.
     */
    private Double snapTarget = null;
    private static final double SNAP_KP = 1.2;               // turn power per radian of error
    private static final double SNAP_MIN_POWER = 0.08;       // enough to keep turning near the end
    private static final double SNAP_MAX_POWER = 0.7;
    private static final double SNAP_TOLERANCE_DEG = 2.0;

    /*
     * Pedro Pathing debug. These stay null if Pedro can't start (for example, no Pinpoint in the
     * robot configuration); the teleop still drives, and telemetry shows why.
     */
    private Localizer localizer = null;
    private GoBildaPinpointDriver pinpoint = null;
    private String pedroError = null;
    private VoltageSensor battery = null;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // Fastest straight-line speeds seen at full stick, to compare with the Foresight tuning
    private double peakForwardSpeed;
    private double peakStrafeSpeed;

    // Results of checking that odometry agrees with the direction the driver is steering
    private String forwardCheck;
    private String strafeCheck;
    private String turnCheck;

    // Stick must be past this (and the other sticks near center) to count as a clean test move
    private static final double CHECK_STICK = 0.5;
    private static final double FULL_STICK = 0.9;
    private static final double OTHER_STICKS_CENTERED = 0.2;
    // The robot must be moving at least this fast for a direction check to count
    private static final double CHECK_SPEED_IN_PER_S = 5.0;
    private static final double CHECK_TURN_RAD_PER_S = 0.5;
    // Measured top speed this far from the Foresight value means the tuning is likely stale
    private static final double SPEED_MISMATCH_PERCENT = 15.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Start Pedro Pathing first, the same way autonomous does, so we test the real
         * configuration in Constants. Building it sets up the drive motors too, so we do it before
         * our own motor setup below; that way our directions and brake mode are the ones that stick.
         */
        try {
            Follower follower = Constants.create(hardwareMap);
            localizer = follower.localizer;
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        } catch (Exception e) {
            localizer = null;
            pedroError = e.getMessage();
        }
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            battery = sensor;
            break;
        }
        resetPedroChecks();

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step. The drive motor names come from Constants.drivetrainConfig, so they are shared
         * with Pedro Pathing and the other opmodes.
         */
        MecanumConfig drive = Constants.drivetrainConfig;
        leftFrontDrive = hardwareMap.get(DcMotor.class, drive.frontLeftName.get());
        rightFrontDrive = hardwareMap.get(DcMotor.class, drive.frontRightName.get());
        leftBackDrive = hardwareMap.get(DcMotor.class, drive.backLeftName.get());
        rightBackDrive = hardwareMap.get(DcMotor.class, drive.backRightName.get());
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");

        /*
         * To drive forward, most robots need the motors on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. The directions also come from Constants.drivetrainConfig,
         * so if a wheel spins the wrong way, fix it there and every opmode picks it up.
         */
        leftFrontDrive.setDirection(drive.frontLeftDirection.get());
        rightFrontDrive.setDirection(drive.frontRightDirection.get());
        leftBackDrive.setDirection(drive.backLeftDirection.get());
        rightBackDrive.setDirection(drive.backRightDirection.get());

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 12.5));

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        /*
         * Much like our drivetrain motors, we set the right intake servo to reverse so that both
         * servos work to pull elements into the intake.
         */
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        /*
         * Show Pedro Pathing telemetry before START, so you can check the Pinpoint is ready and
         * push the robot by hand to see the pose change.
         */
        telemetry.addData("Status", "Initialized");
        addPedroTelemetry(0, 0, 0);
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called mecanumDrive. The mecanumDrive function takes the input from
         * the joysticks, and applies power to the drive motors to move the robot as requested
         * by the driver. Moving the left joystick forwards/back moves all motors forwards/back,
         * moving the right joystick left/right rotates the robot clockwise or counterclockwise,
         * and moving the left joystick left moves the motors in the right way to create a sideways
         * "strafe" movement. Combinations of these inputs can be used to create more complex maneuvers.
         * Note, moving the joystick forward on most gamepads results in a negative signal, so
         * we invert it before passing it to the function.
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, snapRotate(gamepad1.right_stick_x));

        /*
         * Set the intake power variable to equal the right trigger, minus the left trigger.
         * Each trigger outputs a signal from 0-1, with 0 as fully released, and 1 fully depressed.
         * This gives us proportional control of the intake speed. The speed increases as we pull
         * the right trigger further. It's occasionally helpful to be able to reverse the intake,
         * so we also factor in the left trigger. If the left trigger is fully depressed,
         * the intakePower variable will be -1. If the right trigger is fully depressed, the variable
         * will be 1. If the driver pulls both triggers, the intake will remain off.
         * We use this technique (creating a variable, and setting it to our control inputs) to
         * allow us to avoid setting the same motors/servos power more than once per loop. That can
         * create erratic behavior.
         */
        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        launch();

        /*
         * Here we set our intake motor and servos to their intake power. The order of operations
         * here is important though. The gamepad triggers define the starting point for the intake
         * power variable in each loop of our code, but inside our launch function we also sometimes
         * change the intake power. So we need to give our launch function a chance to modify the
         * variable before we write it to our motor and servos.
         */

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        /*
         * Show motor powers on the Driver Station via telemetry.
         */
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Snap (X=90, B=270)", snapTarget == null
                ? "off" : String.format("turning to %.0f deg", Math.toDegrees(snapTarget)));

        addPedroTelemetry(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    /*
     * Returns the turn power to use this loop. Normally that's just the driver's right stick, but
     * while a snap is running we turn toward snapTarget instead. Like mecanumDrive, positive
     * rotate is clockwise, while Pedro's heading is counterclockwise, so the sign is flipped.
     */
    double snapRotate(double stickRotate) {
        if (localizer == null) {
            return stickRotate; // no odometry, so no snap
        }
        if (gamepad1.x) {
            snapTarget = Math.toRadians(90);
        } else if (gamepad1.b) {
            snapTarget = Math.toRadians(270);
        }
        if (Math.abs(stickRotate) > OTHER_STICKS_CENTERED) {
            snapTarget = null; // the driver took over
        }
        if (snapTarget == null) {
            return stickRotate;
        }

        // Shortest way around: wrap the error into -180..180 degrees
        double error = snapTarget - localizer.pose().heading();
        error = Math.atan2(Math.sin(error), Math.cos(error));
        if (Math.abs(error) < Math.toRadians(SNAP_TOLERANCE_DEG)) {
            snapTarget = null;
            return 0;
        }

        double power = SNAP_KP * Math.abs(error);
        power = Math.max(SNAP_MIN_POWER, Math.min(SNAP_MAX_POWER, power));
        return -Math.signum(error) * power;
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
         * Send calculated power to wheels
         */
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /*
     * Pedro Pathing debug telemetry. forward, strafe and rotate are what the driver is asking for
     * (same as mecanumDrive: forward, right and clockwise are positive), so we can check that the
     * odometry agrees with the way the robot is being driven.
     *
     * Pedro measures the robot with x forward, y to the LEFT, and heading counterclockwise, so
     * strafing right should read a negative y speed and turning right a negative turn rate.
     */
    void addPedroTelemetry(double forward, double strafe, double rotate) {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        telemetry.addLine();
        telemetry.addLine("--- Pedro Pathing (BACK resets) ---");
        if (localizer == null) {
            telemetry.addData("Pedro", "NOT RUNNING: %s", pedroError);
            return;
        }

        if (gamepad1.back) {
            localizer.setPose(Pose.zero());
            resetPedroChecks();
        }
        localizer.update();
        Pose pose = localizer.pose();
        Twist twist = localizer.twist();

        /*
         * READY is good. CALIBRATING should only last a moment after INIT; keep the robot still.
         * Any FAULT status means a pod is unplugged or the Pinpoint isn't wired right.
         */
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());

        /*
         * Push the robot a measured distance (for example 48 inches along a tile seam) and check
         * x or y changes by the same amount. If not, the pod type in Constants is wrong.
         */
        telemetry.addData("Pose", "x %.1f in, y %.1f in, heading %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Speed", "fwd %.1f, left %.1f in/s, turn %.0f deg/s",
                twist.vx, twist.vy, Math.toDegrees(twist.omega));

        /*
         * Direction checks. These assume the robot physically moves the way the stick says; if it
         * doesn't, fix the motor directions first (StarterBot Hardware Test helps).
         */
        boolean onlyForward = Math.abs(forward) > CHECK_STICK
                && Math.abs(strafe) < OTHER_STICKS_CENTERED && Math.abs(rotate) < OTHER_STICKS_CENTERED;
        boolean onlyStrafe = Math.abs(strafe) > CHECK_STICK
                && Math.abs(forward) < OTHER_STICKS_CENTERED && Math.abs(rotate) < OTHER_STICKS_CENTERED;
        boolean onlyTurn = Math.abs(rotate) > CHECK_STICK
                && Math.abs(forward) < OTHER_STICKS_CENTERED && Math.abs(strafe) < OTHER_STICKS_CENTERED;

        if (onlyForward && Math.abs(twist.vx) > CHECK_SPEED_IN_PER_S) {
            forwardCheck = Math.signum(twist.vx) == Math.signum(forward)
                    ? "OK"
                    : "WRONG: odometry says the other way. Flip xPodDirection in Constants";
        }
        if (onlyStrafe && Math.abs(twist.vy) > CHECK_SPEED_IN_PER_S) {
            strafeCheck = Math.signum(twist.vy) == -Math.signum(strafe)
                    ? "OK"
                    : "WRONG: odometry says the other way. Flip yPodDirection in Constants";
        }
        if (onlyTurn && Math.abs(twist.omega) > CHECK_TURN_RAD_PER_S) {
            turnCheck = Math.signum(twist.omega) == -Math.signum(rotate)
                    ? "OK"
                    : "WRONG: heading turns the other way. Is the Pinpoint mounted upside down?";
        }
        telemetry.addData("Forward check", forwardCheck);
        telemetry.addData("Strafe check", strafeCheck);
        telemetry.addData("Turn check", turnCheck);

        /*
         * Foresight check. The Foresight tuner measured the robot's top speed at full power. If
         * the robot is now much faster or slower at full stick (new motors, wheels, gearing,
         * weight, or motor directions changed since tuning), Foresight's numbers are stale.
         */
        if (Math.abs(forward) > FULL_STICK && Math.abs(strafe) < OTHER_STICKS_CENTERED
                && Math.abs(rotate) < OTHER_STICKS_CENTERED) {
            peakForwardSpeed = Math.max(peakForwardSpeed, Math.abs(twist.vx));
        }
        if (Math.abs(strafe) > FULL_STICK && Math.abs(forward) < OTHER_STICKS_CENTERED
                && Math.abs(rotate) < OTHER_STICKS_CENTERED) {
            peakStrafeSpeed = Math.max(peakStrafeSpeed, Math.abs(twist.vy));
        }

        ForesightConfig foresight = Constants.foresightConfig;
        telemetry.addLine("--- Foresight ---");
        telemetry.addData("Top fwd speed", compareSpeed(peakForwardSpeed,
                foresight.maxAchievableForwardVelocity.get(), "drive forward at full stick"));
        telemetry.addData("Top strafe speed", compareSpeed(peakStrafeSpeed,
                foresight.maxAchievableStrafeVelocity.get(), "strafe at full stick"));
        telemetry.addData("Natural decel", "fwd %.1f, strafe %.1f in/s^2",
                foresight.naturalForwardDeceleration.get(), foresight.naturalStrafeDeceleration.get());

        // Low battery makes the robot slower than when it was tuned; a slow loop makes paths jerky
        if (battery != null) telemetry.addData("Battery", "%.2f V", battery.getVoltage());
        telemetry.addData("Loop time", "%.0f ms", loopMs);
    }

    /*
     * Describes the fastest speed seen next to the speed Foresight was tuned with.
     */
    private static String compareSpeed(double measured, double tuned, String howToTest) {
        if (measured < 1) {
            return String.format("tuned %.1f in/s; %s to test", tuned, howToTest);
        }
        double percent = (measured - tuned) / tuned * 100;
        String verdict;
        if (percent > SPEED_MISMATCH_PERCENT) {
            verdict = "FASTER than tuned: re-run ForesightTuner";
        } else if (percent < -SPEED_MISMATCH_PERCENT) {
            verdict = "slower: try a longer run; if it stays low check battery or re-tune";
        } else {
            verdict = "OK";
        }
        return String.format("%.1f vs tuned %.1f in/s (%+.0f%%) %s", measured, tuned, percent, verdict);
    }

    private void resetPedroChecks() {
        peakForwardSpeed = 0;
        peakStrafeSpeed = 0;
        forwardCheck = "not tested: push left stick forward";
        strafeCheck = "not tested: push left stick right";
        turnCheck = "not tested: push right stick right";
    }

    void launch() {
        /*
         * Holding the left bumper runs everything backwards to clear a jam: the launcher spins in
         * reverse, the windmill pushes elements back out of the launcher, and the intake pushes
         * them out of the robot. This takes priority over the right bumper, so we return early
         * and skip the normal launch logic below.
         */
        if (gamepad1.left_bumper) {
            launcher.setVelocity(-LAUNCHER_REVERSE_VELOCITY);
            windmillServo.setPower(-1);
            intakePower = -1;
            return;
        }

        /*
         * Calling gamepad1.right_bumper returns a boolean which will be true if the bumper is
         * held down, and false if it is not. Notably, this will continue to be true for every
         * cycle of our code that the driver holds down that bumper.
         * The first step of our launch() function is checking to see if the user is currently
         * holding down the right gamepad. If they are, then we want to start spinning up the launcher.
         * Otherwise, we start spinning the launcher down.
         */
        if (gamepad1.right_bumper) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        /*
         * Here we ask if the driver is currently pressing the right bumper, AND the launcher is
         * spinning fast enough to make a successful shot. If it is, then we will turn on the
         * windmill servo to start feeding the elements into the launcher motor. We also
         * add some power to the intake power. This can sometimes help dislodge stuck elements from
         * inside the hopper.
         */
        if (gamepad1.right_bumper && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }
    }
}
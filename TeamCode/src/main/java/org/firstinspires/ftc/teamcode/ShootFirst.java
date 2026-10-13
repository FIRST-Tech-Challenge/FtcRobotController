package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "ShootFirst", group = "StarterBot")

public class ShootFirst extends OpMode {

    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double FEED_TIME = 0.20; //The feeder servos run this long when a shot is requested.
    final double TIME_BETWEEN_SHOTS = 2;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_LOW_VELOCITY = 1280;
    final double LAUNCHER_TARGET_MED_VELOCITY = 1280;
    final double LAUNCHER_TARGET_HIGH_VELOCITY = 1280;
    final double LAUNCHER_LOW_MIN_VELOCITY = 1280;
    final double LAUNCHER_MED_MIN_VELOCITY = 1280;
    final double LAUNCHER_HIGH_MIN_VELOCITY = 1280;
    private double targetMinVelocity;
    private double targetVelocity;
    int shotsToFire = 3; //The number of shots to fire in this auto.

    double robotRotationAngle = 45;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx flywheel1 = null;
    private CRServo leftIntake = null;
    private CRServo rightIntake = null;
    private DcMotor mainIntake = null;
    private CRServo spinner = null;

    private ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime driverTimer = new ElapsedTime();

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private int desiredAngle;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;
    /*
     * Here is our auto state machine enum. This captures each action we'd like to do in auto.
     */
    private enum AutonomousState {
        LAUNCH,
        DRIVING_OFF_LINE,
        ROTATE,
        FORWARD,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATE2,
        DRIVING_OFF_LINE2,
        COMPLETE;
    }

    private AutonomousState autonomousState;
    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE;
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel_1");
        leftIntake = hardwareMap.get(CRServo.class, "left_intake");
        rightIntake = hardwareMap.get(CRServo.class, "right_intake");
        mainIntake = hardwareMap.get(DcMotor.class, "main_intake");

        spinner = hardwareMap.get(CRServo.class, "spinner");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        flywheel1.setZeroPowerBehavior(FLOAT);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftIntake.setPower(STOP_SPEED);
        rightIntake.setPower(STOP_SPEED);

        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);

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
         * We also set the servo power to 0 here to make sure that the servo controller is booted
         * up and ready to go.
         */
        rightIntake.setPower(0);
        leftIntake.setPower(0);


        /*
         * Here we allow the driver to select which alliance we are on using the gamepad.
         */
        if (gamepad1.b) {
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        driverTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState){
            /*
             * Since the first state of our auto is LAUNCH, this is the first "case" we encounter.
             * This case is very simple. We call our .launch() function with "true" in the parameter.
             * This "true" value informs our launch function that we'd like to start the process of
             * firing a shot. We will call this function with a "false" in the next case. This
             * "false" condition means that we are continuing to call the function every loop,
             * allowing it to cycle through and continue the process of launching the first ball.
             */

            case LAUNCH:
                if (driverTimer.milliseconds() < 10000) {
                    launch(true);
                }
                else {
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                    driverTimer.reset();
                }
                break;
            case DRIVING_OFF_LINE:
                if (driverTimer.milliseconds() < 750) {
                    arcadeDrive(0.5 ,0,0);
                }
                else {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                break;

        }

        /*
         * Show the state and motor powers
         */
        telemetry.addData("Autonomous State", autonomousState);
        telemetry.addData("Motors", "front_left (%.2f), right_right (%.2f), back_left (%.2f), back_right (%.2f)", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        telemetry.addData("motorSpeed", flywheel1.getVelocity());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void arcadeDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe - rotate;
        rightBackPower = forward + strafe - rotate;

        // Normalize motor powers to ensure they are within the -1.0 to 1.0 range
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Drive Status", "Running");
        telemetry.addData("Front Left Power", leftFrontPower);
        telemetry.addData("Front Right Power", rightFrontPower);
        telemetry.addData("Back Left Power", leftBackPower);
        telemetry.addData("Back Right Power", rightBackPower);
        telemetry.update();
    }

    void launch(boolean shotRequested) {
        double currentDistance;
        if (desiredTag == null) {
            currentDistance = 110;
        } else {
            currentDistance = desiredTag.ftcPose.range;
        }
        if (currentDistance < 4) {
            targetVelocity = LAUNCHER_TARGET_LOW_VELOCITY;
            targetMinVelocity = LAUNCHER_LOW_MIN_VELOCITY;
        } else if (currentDistance < 100){
            targetVelocity = LAUNCHER_TARGET_MED_VELOCITY;
            targetMinVelocity = LAUNCHER_MED_MIN_VELOCITY;
            desiredTag = null;
        } else {
            targetVelocity = LAUNCHER_TARGET_HIGH_VELOCITY;
            targetMinVelocity = LAUNCHER_HIGH_MIN_VELOCITY;
            desiredTag = null;
        }
        flywheel1.setVelocity(targetVelocity);
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                flywheel1.setVelocity(targetVelocity);
                if (flywheel1.getVelocity() > targetMinVelocity) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (desiredTag == null || Math.abs(desiredTag.ftcPose.bearing - desiredAngle)<2) {
                    spinner.setPower(FULL_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                } else {
                    if (desiredTag != null) {
                        double turn = Range.clip(desiredTag.ftcPose.bearing * 0.3, -0.3, 0.3);
                        arcadeDrive(0.0, 0.0, turn);
                    }
                }
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    spinner.setPower(STOP_SPEED);
                }
                break;
        }
    }
}
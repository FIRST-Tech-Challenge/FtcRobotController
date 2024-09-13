// Hello! Welcome to our FTC 2024 Chocolate Main.

//FTC code package is recognized
package org.firstinspires.ftc.teamcode;

//All the things that we are using and borrowing
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="Chocolate", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    // Initializing all variables for the program below:
    // This chunk is everything we're doing to control our wheels
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;

    // This chunk is everything we're doing to control our arms
    private DcMotor viperSlide = null;
    double viperSlidePower = 0;
    private Servo leftArm = null;
    private Servo rightArm = null;
    private static final double ARM_DEFAULT = 0.5;
    private static final double ARM_MIN = 0.0;
    private static final double ARM_MAX = 1.0;

    // Collecting joystick position data
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    private final ElapsedTime runtime = new ElapsedTime();

    // Variables for turning
    private IMU imu = null;
    static final double TURN_SPEED_ADJUSTMENT = 0.015;     // Larger is more responsive, but also less stable
    static final double HEADING_ERROR_TOLERANCE = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double MAX_TURN_SPEED = 1.0;     // Max Turn speed to limit turn rate
    static final double MIN_TURN_SPEED = 0.15;     // Min Turn speed to limit turn rate
    private double turnSpeed = 0;
    private double degreesToTurn = 0;

    @Override
    //Op mode is running when the robot is running. It keeps the robot functioning throughout it's time of activity.
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArm = hardwareMap.get(Servo.class, "left_arm");
        rightArm = hardwareMap.get(Servo.class, "right_arm");
        double arm_position = ARM_DEFAULT; // NEHA
        leftArm.setPosition(arm_position);
        rightArm.setPosition(arm_position);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Initialize the IMU configuration // NEHA: should go earlier
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP; // NEHA: shouldn't be a seperate line of code
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD; // NEHA: shouldn't be a seperate line of code
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE); // NEHA: should go with the other wheel initialization
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max; // NEHA: should be up where we initialize all of our other variables

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate. // NEHA: we don't know what "POV Mode" means
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x; // NEHA: Michael thinks the robot will be easier to control if you use the triggers to strafe. Let's test it out!
            yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = (axial + lateral + yaw) / 2;
            rightFrontPower = (axial - lateral - yaw) / 2;
            leftBackPower = (axial - lateral + yaw) / 2;
            rightBackPower = (axial + lateral - yaw) / 2;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // gyro turns
            if (gamepad1.dpad_up)
                turnToHeading(0.0);
            if (gamepad1.dpad_down)
                turnToHeading(180.0);
            if (gamepad1.dpad_left)
                turnToHeading(90);
            if (gamepad1.dpad_right)
                turnToHeading(-135.0); // Turn to face the basket

            if (gamepad1.right_trigger > 0) {
                viperSlidePower = gamepad1.right_trigger;
            }
            if (gamepad1.left_trigger > 0) {
                viperSlidePower = gamepad1.left_trigger;
            }

            // Show the elapsed game time and wheel power.
            logScreenData();
        }
    }

    // Logging all relevant information about the robot in text on the controller hub screen so the driver can see what's going on.
    private void logScreenData() {
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Joystick Axial", "%4.2f", axial);
        telemetry.addData("Joystick Lateral", "%4.2f", lateral);
        telemetry.addData("Joystick Yaw", "%4.2f", yaw);
        telemetry.addData("Current Yaw", "%.0f", getHeading());
        telemetry.addData("Turn Speed", "%4.2f", turnSpeed);
        telemetry.addData("Degrees to turn", "%4.2f", degreesToTurn);

        telemetry.update();
    }

    // Turn to the desired heading.
    private void turnToHeading(double heading) {
        degreesToTurn = heading - getHeading();

        // Keep looping while we are still active, and not on heading.
        while (opModeIsActive()
                && (Math.abs(degreesToTurn) > HEADING_ERROR_TOLERANCE)
                && (gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0)) {

            degreesToTurn = heading - getHeading();
            if(degreesToTurn < -180) degreesToTurn += 360;
            if(degreesToTurn > 180) degreesToTurn -= 360;

            // Clip the speed to the maximum permitted value
            turnSpeed = Range.clip(degreesToTurn*TURN_SPEED_ADJUSTMENT, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            if(turnSpeed < MIN_TURN_SPEED && turnSpeed >= 0) turnSpeed = MIN_TURN_SPEED;
            if(turnSpeed > -MIN_TURN_SPEED && turnSpeed < 0) turnSpeed = -MIN_TURN_SPEED;

            leftFrontDrive.setPower(-turnSpeed);
            rightFrontDrive.setPower(turnSpeed);
            leftBackDrive.setPower(-turnSpeed);
            rightBackDrive.setPower(turnSpeed);

            logScreenData();
        }
    }

    // Used to read the Robot heading directly from the IMU (in degrees) // NEHA: We always want to be succinct. We can remove "Used to" and "directly"
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
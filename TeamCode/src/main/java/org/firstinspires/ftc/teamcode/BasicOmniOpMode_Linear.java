package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
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
    // Initialize all variables for the program below:
    // This chunk controls our wheels
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;

    // Collect joystick position data
    double axial = 0;
    double lateral = 0;
    double yaw = 0;

    // This chunk controls our vertical
    private DcMotor vertical = null;
    private static final double VERTICAL_POWER_DEFAULT = 0.6;
    double verticalPower = 0;
    private static final int VERTICAL_MAX = 700;
    private static final int VERTICAL_MIN = 30;
    private static final int VERTICAL_DEFAULT = 0;
    int verticalPosition = VERTICAL_DEFAULT;

    // This chunk controls our viper slide
    private DcMotor viperSlide = null;
    private static final double VIPER_POWER_DEFAULT = 0.6;
    double viperSlidePower = 0;
    private static final int VIPER_MAX = 3000;
    private static final int VIPER_MIN = 50;
    private static final int VIPER_DEFAULT = 0;
    int viperSlidePosition = VIPER_DEFAULT;

    // This chunk controls our claw
    private Servo claw = null;
    private static final double CLAW_DEFAULT = 0.3;
    private static final double CLAW_MIN = 0.26;
    private static final double CLAW_MAX = 0.5;
    double claw_position = CLAW_DEFAULT;

    private final ElapsedTime runtime = new ElapsedTime();

    // Variables for turns
    private IMU imu = null;
    static final double TURN_SPEED_ADJUSTMENT = 0.015;     // Larger is more responsive, but also less stable
    static final double HEADING_ERROR_TOLERANCE = 1.0;    // How close must the heading get to the target before moving to next step.
    static final double MAX_TURN_SPEED = 1.0;     // Max Turn speed to limit turn rate
    static final double MIN_TURN_SPEED = 0.15;     // Min Turn speed to limit turn rate
    private double turnSpeed = 0;
    private double degreesToTurn = 0;

    @Override
    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        // Initialize the hardware variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        vertical = hardwareMap.get(DcMotor.class, "vertical");
        vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viperSlide = hardwareMap.get(DcMotor.class, "viper_slide");
        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(claw_position);

        // Initialize the IMU configuration
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Get input from the joysticks
            axial = -gamepad1.left_stick_y;
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power.
            leftFrontPower = (axial + lateral + yaw) / 2;
            rightFrontPower = (axial - lateral - yaw) / 2;
            leftBackPower = (axial - lateral + yaw) / 2;
            rightBackPower = (axial + lateral - yaw) / 2;

            // Normalize the values so no wheel power exceeds 100%
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

            // Control the vertical
            verticalPosition = vertical.getCurrentPosition();
            if (gamepad1.dpad_up && vertical.getCurrentPosition() < VERTICAL_MAX) {
                verticalPower = -VERTICAL_POWER_DEFAULT;
            }
            else if (gamepad1.dpad_down && vertical.getCurrentPosition() > VERTICAL_MIN) {
                verticalPower = VERTICAL_POWER_DEFAULT;
            }
            else {
                verticalPower = 0;
            }
            vertical.setPower(verticalPower);

            // Control the viper slide
            viperSlidePosition = -viperSlide.getCurrentPosition();
            if (gamepad1.right_trigger > 0 && viperSlidePosition < VIPER_MAX) {
                viperSlidePower = -VIPER_POWER_DEFAULT;
            }
            else if (gamepad1.left_trigger > 0  && viperSlidePosition > VIPER_MIN) {
                viperSlidePower = VIPER_POWER_DEFAULT;
            }
            else {
                viperSlidePower = 0;
            }
            viperSlide.setPower(viperSlidePower);

            // Control the claw
            if (gamepad1.left_bumper && claw_position < CLAW_MAX) {
                claw_position += 0.01;
            }
            if (gamepad1.right_bumper && claw_position > CLAW_MIN) {
                claw_position -= 0.01;
            }
            claw.setPosition(claw_position);

            // Show the elapsed game time and wheel power.
            logScreenData();
        }
    }

    // Log all (relevant) info about the robot on the hub.
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
        telemetry.addData("Claw position", "%4.2f", claw_position);
        telemetry.addData("Viper Slide Power", "%4.2f", viperSlidePower);
        telemetry.addData("Viper Slide Position", "%d", viperSlidePosition);
        telemetry.addData("Viper Slide Vertical", "%d", vertical.getCurrentPosition());
        telemetry.update();
    }

    // Turn to desired heading.
    private void turnToHeading(double heading) {
        degreesToTurn = heading - getHeading();

        // Keep looping while we are still active.
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

    // Read the Robot heading in degrees directly from the IMU
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
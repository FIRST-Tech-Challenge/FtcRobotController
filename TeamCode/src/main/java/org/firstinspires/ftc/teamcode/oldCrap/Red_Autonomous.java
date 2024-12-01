package org.firstinspires.ftc.teamcode.oldCrap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Autonomous(name = "Red Autonomous Into The Deep")
public class Red_Autonomous extends LinearOpMode {
    // Motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor jointOne = null;
    private DcMotor jointTwo = null;

    // Servos
    private Servo claw = null;
    private Servo horizontalWrist = null;
    private Servo verticalWrist = null;

    // Sensors
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;

    // Timer
    private ElapsedTime runtime = new ElapsedTime();

    // Constants
    private static final double COUNTS_PER_MOTOR_REV = 28;
    private static final double WHEEL_DIAMETER_INCHES = 9.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * Math.PI) / WHEEL_DIAMETER_INCHES;
    private static final double robotLength = 16.0;
    private static final double robotWidth = 16.0;
    private static final double robotHeight = 16.0;
    private static final double halfRL = robotLength / 2;
    private static final double halfRW = robotWidth / 2;
    private static final double halfRH = robotHeight / 2;

    @Override
    public void runOpMode() {
        initialize();
        setBrake();
        stopAndResetEncoders();
        setDirection();

        waitForStart();

        while (opModeIsActive()) {
            setTelemetry();

            double red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Determine alliance and enemy colors
            double allianceColor = red;
            int enemyColor = blue;
            if(allianceColor < 200){
                telemetry.addData("Red winning", "False");
            }else{
                telemetry.addData("Red winning", "True");
            }
            telemetry.update();
            double threshold = 200;
            boolean yellow = (red > threshold && green > threshold && blue < threshold);

            double distance = distanceSensor.getDistance(DistanceUnit.INCH);



            turnLeft(90, 0.25);

            // Move forward while alliance color is weaker than enemy color

        }
    }

    // Initializes motors, servos, and sensors
    private void initialize() {
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        jointOne = hardwareMap.get(DcMotor.class, "jointOne");
        jointTwo = hardwareMap.get(DcMotor.class, "jointTwo");

        claw = hardwareMap.get(Servo.class, "claw");
        horizontalWrist = hardwareMap.get(Servo.class, "horizontalWrist");
        verticalWrist = hardwareMap.get(Servo.class, "verticalWrist");
    }

    // Sets the motors direction when powered with a positive number
    private void setDirection() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        jointOne.setDirection(DcMotor.Direction.REVERSE);
        jointTwo.setDirection(DcMotor.Direction.REVERSE);
    }

    // Resets all motors' encoder counts
    private void stopAndResetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // Sets motors' target position
    private void setPosition(int targetEncoderCount) {
        frontLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
    }

    // Runs motors to their target position
    private void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Sets motors to brake when not in use
    private void setBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Updates telemetry data
    private void setTelemetry() {
        int threshold = 200;
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        boolean yellow = (red > threshold && green > threshold && blue < threshold);

        telemetry.addData("Front left current Position", frontLeft.getCurrentPosition());
        telemetry.addData("Front right current Position", frontRight.getCurrentPosition());
        telemetry.addData("Back left current Position", backLeft.getCurrentPosition());
        telemetry.addData("Back right current Position", backRight.getCurrentPosition());
        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);
        telemetry.addData("Green", green);
        telemetry.addData("Yellow", yellow ? "True" : "False");
        telemetry.update();
        telemetry.update();
    }

    // Moves the robot forward
    private void forward(double inches, double speed) {
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH);
        setPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

//        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
//            setTelemetry();
//        }
        stopAndResetEncoders();
    }

    // Moves the robot backward
    private void backward(double inches, double speed) {
        int targetEncoderCount = -((int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH));
        setPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }

    // Strafe left movement
    private void strafeLeft(double inches, double speed) {
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(-targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(-targetEncoderCount);
        runToPosition();
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }

    // Strafe right movement
    private void strafeRight(double inches, double speed) {
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int) (robotLength * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(-targetEncoderCount);
        backLeft.setTargetPosition(-targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }

    // Turn right by specified degrees
    private void turnRight(double degrees, double speed) {
        double turningRadius = 13.0; // TODO: Find distance from center to wheel
        double turnCircumference = 2 * Math.PI * turningRadius;
        double distancePerWheel = (degrees / 360) * turnCircumference;
        int targetEncoderCount = (int) (distancePerWheel * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(-targetEncoderCount);
        backRight.setTargetPosition(-targetEncoderCount);
        runToPosition();
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        frontRight.setPower(-speed);
        backRight.setPower(-speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }

    // Turn left by specified degrees
    private void turnLeft(double degrees, double speed) {
        double turningRadius = 13.0; // TODO: Find distance from center to wheel
        double turnCircumference = 2 * Math.PI * turningRadius;
        double distancePerWheel = (degrees / 360) * turnCircumference;
        int targetEncoderCount = (int) (distancePerWheel * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(-targetEncoderCount);
        backLeft.setTargetPosition(-targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
        runToPosition();
        frontLeft.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() && (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }

    // Opens the claw to a specific position
    private void openClaw(double openPosition) {
        claw.setPosition(openPosition);
    }

    // Closes the claw to a specific position
    private void closeClaw(double closePosition) {
        claw.setPosition(closePosition);
    }

    // Stops the robot by setting power to 0
    private void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}

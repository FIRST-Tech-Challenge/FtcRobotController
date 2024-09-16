package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Ali's Distance Based Autonomous")
public class Blue_Autonomous extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private DcMotor arm = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;
    private ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;
    private ElapsedTime runtime = new ElapsedTime();

    private static final double COUNTS_PER_MOTOR_REV = 28; // Replace with your encoder counts
    private static final double WHEEL_DIAMETER_INCHES = 9.0; // Replace with your wheel diameter
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * Math.PI) / WHEEL_DIAMETER_INCHES;

    private static final double robotLength = 16.0;
    private static final double robotWidth = 16.0;
    private static final double robotHeight = 16.0;
    private static final double halfRL = robotLength/2;
    private static final double halfRW = robotWidth/2;
    private static final double halfRH = robotHeight/2;


    int red = colorSensor.red();
    int green = colorSensor.green();
    int blue = colorSensor.blue();
    double distance = distanceSensor.getDistance(DistanceUnit.INCH);

    int allienceColor = blue;
    int enemyColor = red;


    @Override
    public void runOpMode() {
        initialize();
        setBrake();
        stopAndResetEncoders();
        setDirection();

        waitForStart();

        while(allienceColor < enemyColor){
            forward(1,0.3);
        }
        backward(1,0.3);


    }
    private void initialize() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        arm = hardwareMap.get(DcMotor.class, "arm");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }
    private void setDirection() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    private void stopAndResetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setPosition(int targetEncoderCount) {
        frontLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
    }
    private void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void setTelemetry(){
        telemetry.addData("Front left current Position", frontLeft.getCurrentPosition());
        telemetry.addData("Front right current Position", frontRight.getCurrentPosition());
        telemetry.addData("Back left current Position", backLeft.getCurrentPosition());
        telemetry.addData("Back right current Position", backRight.getCurrentPosition());

        telemetry.addData("Amount of red",red);
        telemetry.addData("Amount of blue",blue);
        telemetry.addData("Amount of green",green);

        telemetry.update();
    }

    private void forward(double inches, double speed){
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int)(robotLength * COUNTS_PER_INCH);
        setPosition(targetEncoderCount);
        runToPosition();

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&  (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }
    private void backward(double inches, double speed){
        int targetEncoderCount = -((int) (inches * COUNTS_PER_INCH) + (int)(robotLength * COUNTS_PER_INCH));
        setPosition(targetEncoderCount);
        runToPosition();

        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        while (opModeIsActive() &&  (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }

        stopAndResetEncoders();
    }

    private void strafeLeft(double inches, double speed){
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int)(robotLength * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(-targetEncoderCount);
        frontRight.setTargetPosition(targetEncoderCount);
        backLeft.setTargetPosition(targetEncoderCount);
        backRight.setTargetPosition(-targetEncoderCount);
        runToPosition();

        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        while (opModeIsActive() &&  (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }
    private void strafeRight(double inches, double speed){
        int targetEncoderCount = (int) (inches * COUNTS_PER_INCH) + (int)(robotLength * COUNTS_PER_INCH);
        frontLeft.setTargetPosition(targetEncoderCount);
        frontRight.setTargetPosition(-targetEncoderCount);
        backLeft.setTargetPosition(-targetEncoderCount);
        backRight.setTargetPosition(targetEncoderCount);
        runToPosition();

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        while (opModeIsActive() &&  (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            setTelemetry();
        }
        stopAndResetEncoders();
    }

    private void turnRight(double degrees, double speed) {
            double turningRadius = 13.0; //TODO: find distance from center to wheel

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
    private void turnLeft(double degrees, double speed){

            double turningRadius = 13.0; //TODO: find distance from center to wheel

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

    private void openClaw(double leftOpenPosition, double rightOpenPosition){
        leftClaw.setPosition(leftOpenPosition);
        rightClaw.setPosition(rightOpenPosition);
    }
    private void closeClaw(double leftClosedPosition, double rightClosedPosition){
        leftClaw.setPosition(leftClosedPosition);
        rightClaw.setPosition(rightClosedPosition);
    }

    public void rest(double time){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Arm Position: ", frontLeft.getCurrentPosition());
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    private void setBrake(){
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
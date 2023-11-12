package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "blue-far-auto")
public class BlueDistanceTest extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    //private DcMotor linearSlide = null;
    //private CRServo intake = null;
    double integralSum = 0;
    double Kp = 0.1;
    double Ki = 0;
    double Kd = 0;
    // double Kp = 0.05;
    // double Ki = 0.0150;
    // double Kd = 0.000001;
    ElapsedTime timer = new ElapsedTime();

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */

    /**
     * The variable to store our instance of the vision portal.
     */

    @Autonomous(name="blue-far-auto")
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        //linearSlide = hardwareMap.get(DcMotor.class, "outputslide");
        //intake = hardwareMap.crservo.get("outputservo");

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                moveForwardToThree();
                resetPower();
                strafeLeft();
                telemetry.addData("leftfront", leftFrontDrive.getCurrentPosition());
                telemetry.addData("rightfront", rightFrontDrive.getCurrentPosition());
                telemetry.addData("leftback", leftBackDrive.getCurrentPosition());
                telemetry.addData("rightback", rightBackDrive.getCurrentPosition());
                //telemetry.addData("linearslide", linearSlide.getCurrentPosition());
                telemetry.update();
            }}}
    public void moveToPosition(double reference, DcMotor motor) {
        while (motor.getCurrentPosition() != reference) {
            double power = PIDControl(reference, reference, motor);
            motor.setPower(power);
        }
    }
    public double PIDControl(double reference, double lastError, DcMotor motor) {
        double state = motor.getCurrentPosition();
        double error = reference - state;
        if(error < 100 && error > -100) {
            error = 0;
        }
        integralSum += error * timer.seconds();
        double derivative = (error-lastError) / timer.seconds();

        lastError = error;

        timer.reset();

        double out = (error*Kp) + (derivative * Kd) + (integralSum * Ki);
        telemetry.addData("out", out);
        telemetry.addData("position", state);
        telemetry.update();
        return out;
    }
    public void moveForwardToThree() {
        int variance = 100;
        int desiredLoc = -1700;
        while(leftFrontDrive.getCurrentPosition() > desiredLoc + variance
                || leftFrontDrive.getCurrentPosition() < desiredLoc - variance) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power * 0.5);
            rightFrontDrive.setPower(-power * 0.5);
            leftBackDrive.setPower(power * 0.5);
            rightBackDrive.setPower(-power * 0.5);
        }
    }

    public void resetPower() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void strafeLeft() {
        int desiredLoc = 2000;
        int variance = -25;
        while(leftFrontDrive.getCurrentPosition() > desiredLoc - variance
                || leftFrontDrive.getCurrentPosition() < desiredLoc + variance
        ) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
        }
    }

    public void turnLeft() {
        int variance = 25;
        int desiredLoc = -1350;
        while(leftFrontDrive.getCurrentPosition() < desiredLoc - variance
                || leftFrontDrive.getCurrentPosition() > desiredLoc + variance) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        }
    }
}
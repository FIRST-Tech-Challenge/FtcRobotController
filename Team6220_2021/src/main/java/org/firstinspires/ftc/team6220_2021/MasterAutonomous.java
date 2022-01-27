package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

public abstract class MasterAutonomous extends MasterOpMode {
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    // This method gets the maximum value of 3 doubles
    public double max(double a, double b, double c) {
        return Math.max(a, Math.max(b, c));
    }

    // This method drives tank when given an angle drive power and turning power
    public void driveTank(double leftSidePower, double rightSidePower) {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setPower(leftSidePower);
        motorBL.setPower(leftSidePower);
        motorFR.setPower(rightSidePower);
        motorBR.setPower(rightSidePower);
    }

    // This method drives a specified number of inches in a straight line when given a target distance and max speed
    // Set direction to false when going forward and true when going backwards
    public void driveInches(double targetDistance, double minSpeed, boolean forwards) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean distanceReached = false;

        double position = 0.0;
        double distanceLeft;

        while (!distanceReached && opModeIsActive()) {
            if (forwards) {
                distanceLeft = targetDistance + position;
            } else {
                distanceLeft = targetDistance - position;
            }

            if (forwards) {
                driveTank(max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR) * -1,
                        max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR) * -1);
            } else {
                driveTank(max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR),
                        max(distanceLeft / 48, minSpeed, Constants.MIN_DRIVE_PWR));
            }

            // Update positions using last distance measured by encoders
            position = Constants.IN_PER_AM_TICK * (motorFL.getCurrentPosition() + motorBL.getCurrentPosition() +
                    motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / 4.0;

            if (Math.abs(distanceLeft) <= 0.5) {
                driveTank(0.0, 0.0);
                distanceReached = true;
            }
        }
    }

    // This method turns a specified number of degrees when given a target angle to turn
    public void turnToAngle(double targetAngle) {
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double angleLeft;
        boolean angleReached = false;

        while (!angleReached && opModeIsActive()) {
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            // This gets the angle change
            angleLeft = targetAngle - currentAngle;

            if (Math.abs(angleLeft) > 180 && angleLeft > 180) {
                angleLeft = angleLeft - 360;
            } else if (Math.abs(angleLeft) > 180 && angleLeft < 180) {
                angleLeft = angleLeft + 360;
            }

            if (angleLeft > 0) {
                driveTank(Math.max(angleLeft / 90, Constants.MIN_TURN_PWR),
                        Math.max(angleLeft / 90, Constants.MIN_TURN_PWR) * -1);
            } else if (angleLeft < 0) {
                driveTank(Math.min(angleLeft / 90, Constants.MIN_TURN_PWR * -1),
                        Math.max(angleLeft / 90, Constants.MIN_TURN_PWR));
            }

            if (Math.abs(angleLeft) <= 5) {
                driveTank(0.0, 0.0);
                angleReached = true;
            }
        }
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.TENSORFLOW_MODEL_ASSET, Constants.TENSORFLOW_LABELS);
    }
}
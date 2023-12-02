package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "Red auto")
public class AutonomousRed extends LinearOpMode {

    protected DcMotorEx left_front;
    protected DcMotorEx right_front;
    protected DcMotorEx left_back;
    protected DcMotorEx right_back;
    ArrayList<DcMotorEx> driveMotors = new ArrayList<>();
    protected DcMotorEx lift;
    protected Servo servo;
    private DcMotor intake = null;
    OpenCvCamera camera;
    BasicPipeline pipeline = new BasicPipeline();
    // private VisionPortal visionPortal;               // Used to manage the video source.
    // private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.get(Servo.class, "servo");

        driveMotors.add(left_front);
        driveMotors.add(left_back);
        driveMotors.add(right_front);
        driveMotors.add(right_back);

        for(DcMotorEx driveMotor: driveMotors) {
            driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        left_front.setDirection(Constants.motorDirections.get("left_front"));
        left_back.setDirection(Constants.motorDirections.get("left_back"));
        right_front.setDirection(Constants.motorDirections.get("right_front"));
        right_back.setDirection(Constants.motorDirections.get("right_back"));
        lift.setDirection(Constants.motorDirections.get("lift"));

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setVelocity(1);
        servo.setPosition(0.1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Ensure pixel is in right side of box!!");
            telemetry.addData("Prop x value: ", pipeline.getJunctionPoint().x);
            telemetry.addData("Prop area: ", pipeline.getPropAreaAttr());
            telemetry.update();
        }


        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setVelocity(1);
        double propX = pipeline.getJunctionPoint().x;
        double propArea = pipeline.getPropAreaAttr();

        //initAprilTag();
        //setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        //goToAprilTagRelative(1000, 1000, 0, 20, 1);

        forward(0.25, 1200);
        if (propArea < 10000) { // none detected we assume left spike mark
            telemetry.addLine("Left spike mark");
            telemetry.update();
            turn(-0.25, -1000);
            forward(0.5, -100);
            // eject pixel
            intake.setPower(-0.2);
            sleep(1000);
            intake.setPower(0);
            // nudge the pixel incase it falls vertically
            forward(0.5, 50);
            forward(0.5, -50);
            // go to backboard
            forward(0.25, -2500);

        } else if (propX > 600) { // right spike mark
            telemetry.addLine("Right spike mark");
            telemetry.update();
            // line up with mark
            turn(0.25, 1000);
            forward(0.25, 100);
            // eject pixel
            intake.setPower(-0.25);
            sleep(1000);
            intake.setPower(0);
            // nudge the pixel in case it falls vertically
            forward(0.5, 50);
            forward(0.5, -100);

            strafe(0.25, 1000);
            // do a 180
            turn(0.25, 2000);
            forward(0.25, -1000);
            strafe(0.25, 1000);
            forward(0.25, -1000);

        } else { // middle spike mark
            telemetry.addLine("Middle spike mark");
            telemetry.update();
            // push prop out of the way
            forward(0.25, 200);
            forward(0.25, -200);
            // eject pixel- we're already there
            intake.setPower(-0.25);
            sleep(1000);
            intake.setPower(0);
            // nudge the pixel over in case it falls vertically
            forward(0.5, 50);
            forward(0.5, -100);
            // go to backboard
            turn(0.25, -1000);
            forward(0.25, -2500);
        }

        sleep(400);
        lift.setTargetPosition(1500);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setVelocity(1000);
        while (lift.isBusy()) telemetry.addData("lift position: ", lift.getCurrentPosition());
        // wiggle the pixel off
        for (int i = 0; i < 20; i++) {
            servo.setPosition(0.19);
            sleep(100);
            servo.setPosition(0.1);
        }



        sleep(100000);
    }

    public void forward(double power, int setpoint) {
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            driveMotor.setTargetPosition(setpoint);
            driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            driveMotor.setPower(power);
        }
        while (left_front.isBusy()) {
            telemetry.addData("lf position: ", left_front.getCurrentPosition());
            telemetry.addData("lb position: ", left_back.getCurrentPosition());
            telemetry.addData("rf position: ", right_front.getCurrentPosition());
            telemetry.addData("rb position: ", right_back.getCurrentPosition());
            telemetry.update();
        }
    }

    public void strafe(double power, int setpoint){
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        left_front.setTargetPosition(setpoint);
        left_back.setTargetPosition(-setpoint);
        right_front.setTargetPosition(-setpoint);
        right_back.setTargetPosition(setpoint);
        for (DcMotorEx driveMotor : driveMotors) driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setPower(power);
        left_front.setPower(-power);
        right_back.setPower(-power);
        left_back.setPower(power);

    }

    public void turn(double power, int setpoint){
        for (DcMotorEx driveMotor : driveMotors) {
            driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        left_front.setTargetPosition(setpoint);
        left_back.setTargetPosition(setpoint);
        right_front.setTargetPosition(-setpoint);
        right_back.setTargetPosition(-setpoint);
        for (DcMotorEx driveMotor : driveMotors) driveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front.setPower(-power);
        left_front.setPower(power);
        right_back.setPower(-power);
        left_back.setPower(power);

        while (left_front.isBusy()) {
            telemetry.addData("lf position: ", left_front.getCurrentPosition());
            telemetry.addData("lb position: ", left_back.getCurrentPosition());
            telemetry.addData("rf position: ", right_front.getCurrentPosition());
            telemetry.addData("rb position: ", right_back.getCurrentPosition());
        }
    }


    /*
    public void goToAprilTagRelative(double fwd, double strafe, double rot, double allowedError, int tagID) {
        double rangeError = 0;
        double headingError = 0;
        double bearingError = 0;
        AprilTagDetection desiredTag;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // first time around before we start checking if we're within the allowed error
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((tagID < 0) || (detection.id == tagID))  ){
                desiredTag = detection;
                rangeError = desiredTag.ftcPose.range - fwd;
                bearingError = desiredTag.ftcPose.yaw - strafe;
                headingError = desiredTag.ftcPose.bearing - rot;
                telemetry.addData("Good tag found! ", "range: %.2f, yaw: %.2f, bearing/heading: %.2f",
                        desiredTag.ftcPose.range, desiredTag.ftcPose.yaw, desiredTag.ftcPose.bearing);
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        while ((rangeError*rangeError) + (bearingError*bearingError) >= allowedError*allowedError) {
            currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((tagID < 0) || (detection.id == tagID))  ){
                    desiredTag = detection;
                    rangeError = desiredTag.ftcPose.range - fwd;
                    bearingError = desiredTag.ftcPose.yaw - strafe;
                    headingError = desiredTag.ftcPose.bearing - rot;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }
            left_front.setPower((rangeError + bearingError + headingError) * 0.01);
            left_back.setPower((rangeError - bearingError + headingError) * 0.01);
            right_front.setPower((rangeError - bearingError - headingError) * 0.01);
            right_back.setPower((rangeError + bearingError - headingError) * 0.01);
        }

        for (DcMotorEx motor : driveMotors) motor.setPower(0);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(aprilTag)
                .build();
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

     */

}

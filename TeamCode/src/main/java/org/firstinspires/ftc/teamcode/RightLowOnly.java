package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

@Autonomous(name = "RightLowOnly", group = "")
public class RightLowOnly extends LinearOpMode {
    private static final int NUMLOOPS = 3 ;
    //test1

    private SampleMecanumDrive drive;
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    private DcMotor arm = null;

    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public double desiredHeading;

    Orientation angles;
    Acceleration gravity;
    private OpenCvCamera webCam;
    private boolean isCameraStreaming = false;
    Pipeline modifyPipeline = new Pipeline();

    private int resultROI = 2;

    private boolean done = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        Pose2d startPose = new Pose2d(-63, -31, 0);
        drive.setPoseEstimate(startPose);
        //TrajectorySequence aSeq = autoSeq(startPose);


        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);

        desiredHeading = getHeading();

        actuatorUtils.initializeActuator(arm, gripper);

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        initOpenCV();

        actuatorUtils.gripperClose(false);

        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        if (resultROI == 2) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            done = false;
            while (!done && opModeIsActive()) {
                //if (currTime - startTime < 500) {
                //    telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
                //} else {
                telemetry.addData("Time Delta: ", (currTime - startTime));
                resultROI = modifyPipeline.getResultROI();
                if (resultROI == 1) {
                    telemetry.addData("Resulting ROI: ", "Red");
                    done = true;
                } else if (resultROI == 2) {
                    telemetry.addData("Resulting ROI: ", "Green");
                    done = true;
                } else if (resultROI == 3) {
                    telemetry.addData("Resulting ROI: ", "Blue");
                    done = true;
                } else {
                    telemetry.addData("Resulting ROI: ", "Something went wrong.");
                }
                //}
                telemetry.update();
                currTime = System.currentTimeMillis();

            }

        }
        telemetry.update();
        done = false;

        //lift arm up
        while (((currTime - startTime) < 30000) && !done && opModeIsActive()) {
            autoSeq();
            telemetry.addData("IM at ", getHeading());
            telemetry.update();
            if (!isStopRequested())
                actuatorUtils.armPole(actuatorUtils.ArmLevel.ZERO,false);
            parkSeq(resultROI);
            //set arm to lowest position
       /*     while (gripperSensor.getDistance(DistanceUnit.INCH)>2 && !isStopRequested())
            {
                telemetry.addData("ARM Position = ", arm.getCurrentPosition());
                telemetry.update();
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-0.7);
            }
            //disabling and resetting arm
            arm.setPower(0);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
*/
            currTime = System.currentTimeMillis();
            done = true;
        }
    }

    private actuatorUtils.ArmLevel getNextLevel (int currentLevel)
    {
        if (currentLevel == 0)
            return actuatorUtils.ArmLevel.CONE4;
        else if (currentLevel == 1)
            return actuatorUtils.ArmLevel.CONE3;
        else if (currentLevel == 2)
            return actuatorUtils.ArmLevel.CONE2;
        else
            return actuatorUtils.ArmLevel.CONE1;
    }

    private void autoSeq() {
        //raising arm to high pole height
        if (isStopRequested())
            return;
        try {
            actuatorUtils.armPole(actuatorUtils.ArmLevel.LOW_POLE,false);
        } catch (InterruptedException e) {
            telemetry.addData(e.getMessage(), "");
            telemetry.update();
        }

        //Sequence to drive to drive towards low pole
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(0))) //move to middle of the corner square
                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(45))) //goes to middle of 2nd square towards cone stack (and turns 45 toward low junction)
                .lineToLinearHeading(new Pose2d(-29,-52,Math.toRadians(45))) //drives to low junction
                .build();

        //drive to low pole
        if (isStopRequested())
            return;
        drive.followTrajectorySequence(seq1);

        //drop cone
        if (isStopRequested())
            return;
        try {
            actuatorUtils.gripperOpen(false,false);
            actuatorUtils.armPole(actuatorUtils.ArmLevel.CONE5,false);
        } catch (InterruptedException e) {
            telemetry.addData(e.getMessage(), "");
            telemetry.update();
        }

        //sequence to align to the cone stack
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-36, -60, Math.toRadians(45))) //back up to middle of second square by wall
                .lineToLinearHeading(new Pose2d(-11, -58, Math.toRadians(-90))) //turns and lines up to cone stack
                .build();

        //drive to cone stack
        if (isStopRequested())
            return;
        drive.followTrajectorySequence(seq2);

        //Sequence to drive cone stack
        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-11, -62, Math.toRadians(-90))) //moves more forward to cone stack
                .build();

        //drive to cone stack
        if (isStopRequested())
            return;
        drive.followTrajectorySequence(seq3);

        //cycle loop
        for (int i = 0; i < NUMLOOPS ; i++)
        {
            //grab cone and raise arm to low pole
            if (isStopRequested())
                return;
            try {
                actuatorUtils.gripperClose(true);
                actuatorUtils.armPole(actuatorUtils.ArmLevel.LOW_POLE, false);
            } catch (InterruptedException e) {
                telemetry.addData(e.getMessage(), "");
                telemetry.update();
            }

            //Sequence to drive to low pole
            TrajectorySequence seq4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-11, -52, Math.toRadians(-90))) //backing up to make turn
                    .lineToLinearHeading(new Pose2d(-11, -46.5, Math.toRadians(180))) //turn to going toward pole
                    .lineToLinearHeading(new Pose2d(-17, -46.5, Math.toRadians(180))) //driving to pole
                    .build();

            //drive to low pole
            if (isStopRequested())
                return;
            drive.followTrajectorySequence(seq4);

            //drop cone and lower arm to next cone height
            if (isStopRequested())
                return;
            try {
                actuatorUtils.gripperOpen(false);
                actuatorUtils.armPole(getNextLevel(i),false);
            }
            catch (InterruptedException e)
            {
                telemetry.addData(e.getMessage(), "");
                telemetry.update();
            }

            //Sequence to drive back to cone stack
            TrajectorySequence seq5 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-10, -46.5, Math.toRadians(180))) //back up
                    .build();

            //drive to cone stack
            if (isStopRequested())
                return;
            drive.followTrajectorySequence(seq5);

            if (i+1<NUMLOOPS) {
                TrajectorySequence seq6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-11, -62, Math.toRadians(-90))) //go to stack again
                        .build();
                //drive to cone stack
                if (isStopRequested())
                    return;
                drive.followTrajectorySequence(seq6);
            }
        }
    }
    private void parkSeq(int park) {
        Pose2d pose = drive.getPoseEstimate();
        if (park == 3) {
            pose = new Pose2d(-12, -60, Math.toRadians(-90));
        } else if (park == 2) {
            pose = new Pose2d(-12, -36, Math.toRadians(-90));
        } else {
            pose = new Pose2d(-12, -10, Math.toRadians(-90));
        }
        TrajectorySequence seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build();
        drive.followTrajectorySequence(seq);
    }
    private void initOpenCV() {
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        // For a webcam (uncomment below)
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId2);
        // For a phone camera (uncomment below)
        // webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Pipeline: ", "Initialized");
                telemetry.update();
                isCameraStreaming = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getHeading() {
        double angle = drive.getRawExternalHeading();
        return angle;
    }
    private float convertRad(int input) {
        float x;
        x=input/180f;
        x*=Math.PI;
        x*=(1);
        return x;
    }
}



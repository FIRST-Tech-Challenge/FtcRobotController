package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.teamcode.Robot.*;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name = "Auto Red Top RR", preselectTeleOp = "teleop")
public class RedTopRR extends LinearOpMode {

    OpenCvCamera webcam;
    final int ARM_DOWN = 388;
    final int ARM_UP = 0;
    final double ARM_POWER = 0.4;
    private Trajectory moveToZoneAgain = null;

    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        MainMecanumDrive drive = new MainMecanumDrive(hardwareMap);
        Robot.initAccessories(this);
        Pose2d startPose = new Pose2d(57, 20, Math.toRadians(0)); //init starting position
        drive.setPoseEstimate(startPose);
        //////Start Camera Streaming//////
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        RingVisionPipeline pipeline = new RingVisionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

////////Program start////////////////////////////////////////////////////////////////////////
        //init servos
        blocker.setPosition(BLOCKER_OPEN);
        //wobbleArmMotor.setTargetPosition(ARM_UP);
        //wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //wobbleArmMotor.setPower(.5);
        wobbleClaw.setPosition(0);

        waitForStart();

        telemetry.addData("location: ", pipeline.getLocation());
        telemetry.update();

        //store the value of the ring stack
        int stackPos;
        switch (pipeline.getLocation()) {
            case C_FULL_STACK:
                stackPos = 3;
                break;
            case B_HALF_STACK:
                stackPos = 2;
                break;
            case A_NO_STACK:
                stackPos = 1;
                break;
            default: stackPos = 1;
        }

        //move to first pos
        Trajectory initForward = drive.trajectoryBuilder(startPose,true)
                .splineTo(new Vector2d(-4,20), Math.toRadians(180))
                .build();
        drive.followTrajectory(initForward);

        //Ramp up launcher
        launcher2.setPower(0.85);
        //move to first shot
        Trajectory shot1 = drive.trajectoryBuilder(initForward.end())
                .lineToLinearHeading(new Pose2d(-7.5, 43, Math.toRadians(0)))
                .build();
        drive.followTrajectory(shot1);

        //launch ring1
        sleep(1500);
        launcherbelt.setTargetPosition(900);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(1); //0.6
        while(launcherbelt.isBusy()) {}
        //launch ring2
        sleep(300);
        launcherbelt.setTargetPosition(3000);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(1); //.6
        while(launcherbelt.isBusy()) {}
        //launch ring3
        sleep(300);
        launcherbelt.setTargetPosition(7500);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(1); //.6
        while(launcherbelt.isBusy()) {}
        launcher1.setPower(0);
        launcher2.setPower(0);

        ///////////three different paths depending on drop zone of wobble goal////////////


        switch (stackPos) {
            case 3:
                //FULL Stack rings
                zoneC(drive, shot1);
                break;
            case 2:
                //HALF stack rings
                zoneB(drive, shot1);
                break;
            case 1:
                //NO rings
                zoneA(drive, shot1);
                break;
            default:
                moveToZoneAgain = drive.trajectoryBuilder(shot1.end(),true)
                        .splineTo(new Vector2d(-5,40), Math.toRadians(-90))
                        .build();
                drive.followTrajectory(moveToZoneAgain);
        }


        ejectWobbleGoal();

        Trajectory moveToLine = drive.trajectoryBuilder(moveToZoneAgain.end(),true)
                .splineTo(new Vector2d(-19,2), Math.toRadians(0))
                .build();
        drive.followTrajectory(moveToLine);

        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);


        if (isStopRequested()) return;
        sleep(2000);
    }


    private void zoneA(MainMecanumDrive drive, Trajectory shot1){
        Trajectory moveToZone;
        Trajectory moveToWobble2;
        Trajectory approachWobble2;
        moveToZone = drive.trajectoryBuilder(shot1.end(),true)
                .lineToLinearHeading(new Pose2d(-10, 54, Math.toRadians(115)))
                .build();
        drive.followTrajectory(moveToZone);
        ejectWobbleGoal();
        moveToWobble2 = drive.trajectoryBuilder(moveToZone.end(), true)
                //.splineTo(new Vector2d(7,43), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(10, 49, Math.toRadians(0)))
                .build();
        drive.followTrajectory(moveToWobble2);

        //move arm down
        moveWobbleArmDown();

        approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                .forward(13)
                .build();
        drive.followTrajectory(approachWobble2);

        pickUpWobble();

        moveToZoneAgain = drive.trajectoryBuilder(moveToWobble2.end())
                .lineToLinearHeading(new Pose2d(-5, 51, Math.toRadians(143)))
                .build();
        drive.followTrajectory(moveToZoneAgain);
    }

    private void zoneB(MainMecanumDrive drive, Trajectory shot1){
        Trajectory moveToZone;
        Trajectory moveToWobble2;
        Trajectory approachWobble2;
        moveToZone = drive.trajectoryBuilder(shot1.end(),true)
                .splineTo(new Vector2d(-29,32), Math.toRadians(-45))
                .build();
        drive.followTrajectory(moveToZone);
        ejectWobbleGoal();
        moveToWobble2 = drive.trajectoryBuilder(moveToZone.end(),true)
                .splineTo(new Vector2d(8.5,44.8), Math.toRadians(180))
                .build();
        drive.followTrajectory(moveToWobble2);

        //move arm down
        moveWobbleArmDown();

        approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                .forward(13.8)
                .build();
        drive.followTrajectory(approachWobble2);
        pickUpWobble();

        moveToZoneAgain = drive.trajectoryBuilder(approachWobble2.end(),true)
                .splineTo(new Vector2d(-20,27), Math.toRadians(-45))
                .build();
        drive.followTrajectory(moveToZoneAgain);
    }

    private void zoneC(MainMecanumDrive drive, Trajectory shot1){
        Trajectory moveToZone;
        Trajectory moveToWobble2;
        Trajectory approachWobble2;
        Trajectory backUp;
        moveToZone = drive.trajectoryBuilder(shot1.end(),true)
                //.splineTo(new Vector2d(-48,50), Math.toRadians(-45))
                .lineToLinearHeading(new Pose2d(-49, 59, Math.toRadians(145)))
                .build();
        drive.followTrajectory(moveToZone);
        ejectWobbleGoal();
        backUp = drive.trajectoryBuilder(moveToZone.end())
                .lineToLinearHeading(new Pose2d(-30, 40, Math.toRadians(180)))
                .build();
        drive.followTrajectory(backUp);

        moveToWobble2 = drive.trajectoryBuilder(backUp.end())
                //.splineTo(new Vector2d(13.7,47), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(10, 47, Math.toRadians(0)))
                .build();
        drive.followTrajectory(moveToWobble2);

        //move arm down
        moveWobbleArmDown();

        approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                .forward(13)
                .build();
        drive.followTrajectory(approachWobble2);
        pickUpWobble();

        moveToZoneAgain = drive.trajectoryBuilder(approachWobble2.end(),true)
                //.splineTo(new Vector2d(-46,48), Math.toRadians(-45))
                .lineToLinearHeading(new Pose2d(-49, 52, Math.toRadians(115)))
                .build();
        drive.followTrajectory(moveToZoneAgain);
    }





    private void pickUpWobble(){
        wobbleClaw.setPosition(0); //closed
        sleep(1000);
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() > ARM_UP + 30){}

    }

    private void ejectWobbleGoal() {
        wobbleArmMotor.setTargetPosition(ARM_DOWN);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 30){}
        wobbleClaw.setPosition(1);
        sleep(1000);
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() > ARM_UP + 30){}
        wobbleClaw.setPosition(0);
    }

    private void moveWobbleArmDown(){
        //move arm down
        wobbleClaw.setPosition(1);
        wobbleArmMotor.setTargetPosition(ARM_DOWN);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 10){}
    }
}

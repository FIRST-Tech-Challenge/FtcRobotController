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
@Autonomous(name = "Auto Red Top RR")
public class RedTopRR extends LinearOpMode {

    OpenCvCamera webcam;
    final int ARM_DOWN = 388;
    final int ARM_UP = 0;
    final double ARM_POWER = 0.4;

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
        wobbleArm.setPosition(1);
        blocker.setPosition(.2);
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
        Trajectory shot = drive.trajectoryBuilder(startPose,true)
                .splineTo(new Vector2d(-4,20), Math.toRadians(180))
                .build();
        drive.followTrajectory(shot);

        //move to first shot
        Trajectory shot1 = drive.trajectoryBuilder(shot.end())
                .lineToLinearHeading(new Pose2d(-6, 58, Math.toRadians(0)))
                .build();
        drive.followTrajectory(shot1);

        //launch ring1
        launcher2.setPower(0.83);
        sleep(1500);
        launcherbelt.setTargetPosition(900);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(1); //0.6
        while(launcherbelt.isBusy()) {}
        //launch ring2
        sleep(300);
        launcherbelt.setTargetPosition(1800);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(1); //.6
        while(launcherbelt.isBusy()) {}
        //launch ring3
        sleep(300);
        launcherbelt.setTargetPosition(3400);
        launcherbelt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcherbelt.setPower(1); //.6
        while(launcherbelt.isBusy()) {}
        launcher1.setPower(0);
        launcher2.setPower(0);

        //move to mid field
        Trajectory shot3 = drive.trajectoryBuilder(startPose,true)
                .splineTo(new Vector2d(-2,21.5), Math.toRadians(180))
                .build();
        //drive.followTrajectory(shot3);

        ///////////three different paths depending on drop zone of wobble goal////////////
        Trajectory moveToZone;
        Trajectory moveToWobble2;
        Trajectory approachWobble2;
        Trajectory moveToZoneAgain;
        switch (stackPos) {
            case 3:
                //FULL Stack rings
                Trajectory backUp;
                moveToZone = drive.trajectoryBuilder(shot1.end(),true)
                        //.splineTo(new Vector2d(-48,50), Math.toRadians(-45))
                        .lineToLinearHeading(new Pose2d(-53, 59, Math.toRadians(145)))
                        .build();
                drive.followTrajectory(moveToZone);
                ejectWobbleGoal();
                backUp = drive.trajectoryBuilder(moveToZone.end())
                        .lineToLinearHeading(new Pose2d(-30, 40, Math.toRadians(180)))
                        .build();
                drive.followTrajectory(backUp);

                /*

                moveToZone = drive.trajectoryBuilder(shot3.end(),true)
                        .splineTo(new Vector2d(-48,50), Math.toRadians(-45))
                        .build();
                drive.followTrajectory(moveToZone);
                ejectWobbleGoal();

                backUp = drive.trajectoryBuilder(moveToZone.end())
                        .back(20)
                        .build();
                drive.followTrajectory(backUp);
                 */

                moveToWobble2 = drive.trajectoryBuilder(backUp.end())
                        .splineTo(new Vector2d(13.7,47), Math.toRadians(0))
                        .build();
                drive.followTrajectory(moveToWobble2);

                //move arm down
                wobbleClaw.setPosition(1);
                wobbleArm.setPosition(.5); //down
                wobbleArmMotor.setTargetPosition(ARM_DOWN);
                wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArmMotor.setPower(ARM_POWER);
                while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 10){}

                approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                        .forward(13)
                        .build();
                drive.followTrajectory(approachWobble2);
                pickUpWobble();

                moveToZoneAgain = drive.trajectoryBuilder(approachWobble2.end(),true)
                        .splineTo(new Vector2d(-46,48), Math.toRadians(-45))
                        .build();
                drive.followTrajectory(moveToZoneAgain);
                break;


            case 2:
                //HALF stack rings
                drive.followTrajectory(shot3);
                moveToZone = drive.trajectoryBuilder(shot3.end(),true)
                        .splineTo(new Vector2d(-34.5,32), Math.toRadians(-45))
                        .build();
                drive.followTrajectory(moveToZone);
                ejectWobbleGoal();
                moveToWobble2 = drive.trajectoryBuilder(moveToZone.end(),true)
                        .splineTo(new Vector2d(1,43.8), Math.toRadians(180))
                        .build();
                drive.followTrajectory(moveToWobble2);

                //move arm down
                wobbleClaw.setPosition(1);
                wobbleArm.setPosition(.5); //down
                wobbleArmMotor.setTargetPosition(ARM_DOWN);
                wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArmMotor.setPower(ARM_POWER);
                while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 30){}

                approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                        .forward(13.8)
                        .build();
                drive.followTrajectory(approachWobble2);
                pickUpWobble();

                moveToZoneAgain = drive.trajectoryBuilder(approachWobble2.end(),true)
                        .splineTo(new Vector2d(-28,22), Math.toRadians(-45))
                        .build();
                drive.followTrajectory(moveToZoneAgain);


                break;


            case 1:
                //NO rings
                drive.followTrajectory(shot3);
                moveToZone = drive.trajectoryBuilder(shot3.end(),true)
                        .splineTo(new Vector2d(-17,54), Math.toRadians(-60))
                        .build();
                drive.followTrajectory(moveToZone);
                ejectWobbleGoal();
                moveToWobble2 = drive.trajectoryBuilder(moveToZone.end(),true)
                        .splineTo(new Vector2d(6,43), Math.toRadians(180))
                        .build();
                drive.followTrajectory(moveToWobble2);

                //move arm down
                wobbleClaw.setPosition(1);
                wobbleArm.setPosition(.5); //down
                wobbleArmMotor.setTargetPosition(ARM_DOWN);
                wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArmMotor.setPower(ARM_POWER);
                while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 30){}

                approachWobble2 = drive.trajectoryBuilder(moveToWobble2.end(),true)
                        .forward(13)
                        .build();
                drive.followTrajectory(approachWobble2);
                pickUpWobble();

                Trajectory moveRight = drive.trajectoryBuilder(moveToWobble2.end(),true)
                        .strafeRight(13)
                        .build();
                drive.followTrajectory(moveRight);

                moveToZoneAgain = drive.trajectoryBuilder(shot.end())
                        .lineToLinearHeading(new Pose2d(-14, 42, Math.toRadians(145)))
                        .build();
                drive.followTrajectory(moveToZoneAgain);
                /*
                moveToZoneAgain = drive.trajectoryBuilder(moveRight.end(),true)
                        .splineTo(new Vector2d(-14,42), Math.toRadians(-30))
                        .build();
                drive.followTrajectory(moveToZoneAgain);

                 */
                break;
            default:
                moveToZoneAgain = drive.trajectoryBuilder(shot3.end(),true)
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

    private void pickUpWobble(){
        wobbleClaw.setPosition(0); //closed
        sleep(1000);
        wobbleArm.setPosition(1); //up
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() > ARM_UP + 30){}

    }

    private void ejectWobbleGoal() {
        wobbleArm.setPosition(.5); //down
        wobbleArmMotor.setTargetPosition(ARM_DOWN);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() < ARM_DOWN - 30){}
        wobbleClaw.setPosition(1);
        sleep(1000);
        wobbleArm.setPosition(1); //up
        wobbleArmMotor.setTargetPosition(ARM_UP);
        wobbleArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArmMotor.setPower(ARM_POWER);
        while (wobbleArmMotor.getCurrentPosition() > ARM_UP + 30){}
        wobbleClaw.setPosition(0);
    }
}

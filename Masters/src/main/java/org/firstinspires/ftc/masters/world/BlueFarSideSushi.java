package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.BlueFarSidePath;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Date;
import java.util.List;

@Autonomous(name="Far Side Blue under Truss", group="competition")
public class BlueFarSideSushi extends FarSideOpMode {

    protected ElapsedTime dropTime = null;

    @Override
    protected void initializeProp(){
        drive.initializePropFindRightProcessing();
    }

    @Override
    protected void initAuto(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.initializeAprilTagProcessing();
        initializeProp();
        drive.initializeVisionPortal();

        long startTime = new Date().getTime();
        long time = 0;

        VisionPortal visionPortal = drive.getMyVisionPortal();
        while(time<2000 && opModeIsActive()){
//        while(visionPortal.getCameraState()!= VisionPortal.CameraState.CAMERA_DEVICE_READY
//                 && opModeIsActive()){
            time = new Date().getTime() - startTime;
            telemetry.addData("camera state", visionPortal.getCameraState());
            telemetry.update();
        }

//        drive.activateFrontCamera();
        drive.enablePropProcessor();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.raiseIntake();
        drive.closeBackFingers();
        drive.initPlane();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initAuto();

        Pose2d startPose = new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);
        drive.setWristServoPosition(outtakeWristPosition);

        //PURPLE PIXEL

        rightPurple = BlueFarSidePath.getRightPurpleSushi(drive, startPose);

        leftPurple = BlueFarSidePath.getLeftPurpleSushi(drive, startPose);

        middlePurple = BlueFarSidePath.getMidPurpleSushi(drive, startPose);


        stackToRightYellow = BlueFarSidePath.rightPurpleToYellow(drive, rightPurple.end());
        stackToMidYellow = BlueFarSidePath.midPurpleToYellow(drive, middlePurple.end());
        stackToLeftYellow = BlueFarSidePath.leftPurpleToYellow(drive, leftPurple.end());

        parkFromMid = BlueFarSidePath.parkOutside(drive, stackToMidYellow.end());
        parkFromLeft = BlueFarSidePath.parkOutside(drive, stackToLeftYellow.end());
        parkFromRight = BlueFarSidePath.parkOutside(drive, stackToRightYellow.end());

        propPos = drive.getPropFindProcessor().position;

        waitForStart();
        drive.dropIntake();
        currentState = State.PURPLE_DEPOSIT_PATH;


        retrievePropPos();

        TrajectorySequence nextPath=null;


        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(outtakeTarget);
            telemetry.update();

            switch (currentState){
                case PURPLE_DEPOSIT_PATH:
                    purpleDepositPath();
                    break;
                case PURPLE_DEPOSIT:
                    purpleDeposit();
                    break;

                case BACKDROP_DEPOSIT_PATH:
                    switch (propPos){
                        case LEFT:
                            backdropDepositPath(State.PARK, parkFromLeft);
                            break;
                        case RIGHT:
                            backdropDepositPath(State.PARK, parkFromRight);
                            break;
                        case MID:
                            backdropDepositPath(State.PARK, parkFromMid);
                            break;
                    }

                    break;
                case PARK:

                    park();
                    break;
            }
        }
    }

    protected void purpleDepositPath(){
        if (propPos == PropFindRightProcessor.pos.LEFT) {
            drive.followTrajectorySequenceAsync(leftPurple);

        } else if (propPos == PropFindRightProcessor.pos.RIGHT) {
            drive.followTrajectorySequenceAsync(rightPurple);

        } else {
            drive.followTrajectorySequenceAsync(middlePurple);
        }

        currentState = FarSideOpMode.State.PURPLE_DEPOSIT;
    }

    @Override
    protected void purpleDeposit(){

        if (!drive.isBusy()){
            if (purpleDepositTime ==null){
                drive.raiseIntake();
                outtakeWristPosition = getOuttakeWristPosition();

                purpleDepositTime = new ElapsedTime();
            } else if (purpleDepositTime.milliseconds()>100) {

                switch(propPos){
                    case RIGHT:
                        drive.followTrajectorySequenceAsync(stackToRightYellow);
                        break;
                    case LEFT:
                        drive.followTrajectorySequenceAsync(stackToLeftYellow);
                        break;
                    case MID:
                        drive.followTrajectorySequenceAsync(stackToMidYellow);
                        break;
                }
//                outtakeTarget = CSCons.OuttakePosition.AUTO.getTarget();
                currentState= State.BACKDROP_DEPOSIT_PATH;
            }
        }
    }

    protected void backdropDepositPath(FarSideOpMode.State nextState, TrajectorySequence nextPath){

        if (drive.getPoseEstimate().getX()>15){
           outtakeTarget = CSCons.OuttakePosition.AUTO.getTarget();
            drive.outtakeToBackdrop();
        }
        if (drive.getPoseEstimate().getX()>25){
            drive.setWristServoPosition(getOuttakeWristPosition());
        }
//
//        if (drive.getBackSlides().getCurrentPosition() > outtakeTarget - 150) {
//                drive.setWristServoPosition(getOuttakeWristPosition());
//
//        }

        if (!drive.isBusy()){

            drive.openFingers();
            if (depositTime==null){
                depositTime= new ElapsedTime();
            } else if (depositTime.milliseconds()>100){
                dropTime = new ElapsedTime();
                if (nextState == FarSideOpMode.State.PARK){
                    drive.followTrajectorySequenceAsync(nextPath);
                } else if (nextState == FarSideOpMode.State.TO_STACK){
                    drive.intakeOverStack();
                    drive.followTrajectorySequenceAsync(nextPath);
                }
                cycleCount++;
                currentState= nextState;
            }
        }
    }
}

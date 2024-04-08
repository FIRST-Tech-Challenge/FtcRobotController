package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Date;
import java.util.List;

public abstract class BackDropOpMode extends LinearOpMode {

    protected OpenCvCamera webcam;

    protected static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    protected static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    protected TelemetryPacket packet = new TelemetryPacket();

    public enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,

        TO_STACK,
        TO_BACKBOARD,

        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT,

        TO_PARK,

        PARK,
        LOWER,
        END
    }

    protected ElapsedTime purpleDepositTime = null;
    protected ElapsedTime depositTime = null;
    protected ElapsedTime dropTime = null;
    protected ElapsedTime pickupElapsedTime= null;

    protected CSCons.OuttakeWrist outtakeWristPosition = CSCons.OuttakeWrist.vertical;

    protected SampleMecanumDrive drive;

    protected PropFindRightProcessor.pos propPos = null;

    protected TrajectorySequence rightPurple, leftPurple, middlePurple;
    protected TrajectorySequence rightYellow, leftYellow, midYellow;
    protected TrajectorySequence toStackFromRight, toStackFromLeft, toStackFromMid;
    protected TrajectorySequence toBackBoard;
    protected TrajectorySequence park;

    protected State currentState;
    protected int outtakeTarget = 0;

    protected void initAuto(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.initializeAprilTagProcessing();
        initializeProp();
        drive.initializeVisionPortal(drive.getPropFindProcessor());


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        drive.raiseIntake();
        drive.closeFingers();


    }

    protected void initializeProp(){

    }

    protected void retrievePropPos(){
        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = drive.getPropFindProcessor().position;
            telemetry.addData("Position", propPos);
        }
    }

    protected void purpleDepositPathState(){
        if (propPos == PropFindRightProcessor.pos.LEFT) {
            drive.followTrajectorySequenceAsync(leftPurple);

        } else if (propPos == PropFindRightProcessor.pos.RIGHT) {
            drive.followTrajectorySequenceAsync(rightPurple);

        } else {
            drive.followTrajectorySequenceAsync(middlePurple);
        }

        currentState = State.PURPLE_DEPOSIT;
    }

    protected void purpleDepositState(){
        if (!drive.isBusy()){
            if (purpleDepositTime ==null){
                drive.raiseIntake();
                outtakeWristPosition = CSCons.OuttakeWrist.flatRight;
                purpleDepositTime = new ElapsedTime();
            } else if (purpleDepositTime.milliseconds()>100) {
                outtakeTarget = CSCons.OuttakePosition.AUTO.getTarget();
                switch(propPos){
                    case RIGHT:
                        drive.followTrajectorySequenceAsync(rightYellow);
                        break;
                    case LEFT:
                        drive.followTrajectorySequenceAsync(leftYellow);
                        break;
                    case MID:
                        drive.followTrajectorySequenceAsync(midYellow);
                        break;
                }
                currentState=State.YELLOW_DEPOSIT_PATH;
            }
        }
    }

    protected void yellowDepositPathState(State nextState){
        if (drive.getBackSlides().getCurrentPosition()>outtakeTarget- 200){
            drive.outtakeToBackdrop();
            drive.setWristServoPosition(CSCons.OuttakeWrist.flatRight);
        } else if (drive.getBackSlides().getCurrentPosition()>10){
            drive.outtakeToBackdrop();
        }

        if (!drive.isBusy()){
            drive.openFingers();
            if (depositTime==null){
                depositTime= new ElapsedTime();
            } else if (depositTime.milliseconds()>100){

                if (nextState == State.PARK){
                    drive.followTrajectorySequenceAsync(park);
                } else if (nextState == State.TO_STACK){
                    drive.intakeOverStack();
                    switch(propPos){
                        case RIGHT:
                            drive.followTrajectorySequenceAsync(toStackFromRight);
                            break;
                        case LEFT:
                            drive.followTrajectorySequenceAsync(toStackFromLeft);
                            break;
                        case MID:
                            drive.followTrajectorySequenceAsync(toStackFromMid);
                            break;
                    }
                }
                currentState= nextState;
            }
        }
    }

    protected void toStack(){
        if (drive.getPoseEstimate().getX()<-30){
            drive.startIntake();
        }
        if (!drive.isBusy()){
            if(dropTime==null) {
                drive.intakeToTopStack();
                dropTime = new ElapsedTime();
            } else if (dropTime.milliseconds()>500){
                drive.stopIntake();
                drive.outtakeToPickup();
                pickupElapsedTime = new ElapsedTime();
                currentState = State.TO_BACKBOARD;

                drive.followTrajectorySequenceAsync(toBackBoard);

            }
        }
    }

    protected void followPath(){
        switch (currentState) {
            case PURPLE_DEPOSIT_PATH:



                break;

            case PURPLE_DEPOSIT:

                break;

            case YELLOW_DEPOSIT_PATH:




                break;

//                case YELLOW_DEPOSIT:
//                    if(!drive.isBusy()) {
//                        if (preloadInt == 0) {
//                            preloadTime.reset();
//                            drive.dropPixel();
//                            preloadInt++;
//                        }
//                        if(preloadInt == 1){
//                            if(preloadTime.milliseconds() > 1000) {
//                                outtakeTarget = 1700;
//                                currentState = State.TOPARK;
//                            }
//
//                        }
//                    }
//                    break;
//
//                case TOPARK:
//                    drive.followTrajectorySequenceAsync(backAway);
//                    currentState = State.PARK;
//                    break;
//
//                case PARK:
//                    if(!drive.isBusy()){
//
//                        drive.followTrajectorySequenceAsync(Park);
//
//                        drive.outtakeToTransfer();
//
//                        outtakeTarget = 0;
//                        currentState = State.END;
//
//
//                    }
//
//
//
//                    break;
//
            case END:

                break;
        }
    }

}

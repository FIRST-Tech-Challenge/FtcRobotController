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

public abstract class FarSideOpMode extends LinearOpMode {

    protected OpenCvCamera webcam;

    protected static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    protected static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    protected TelemetryPacket packet = new TelemetryPacket();

    int cycleCount=0;
    public enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,

        TO_STACK,

        BACKDROP_DEPOSIT_PATH,
        YELLOW_DEPOSIT,

        TOPARK,

        TO_STACK_CYCLE,
        TO_BACKBOARD_CYCLE,

        PARK,
        LOWER,
        END
    }

    protected ElapsedTime purpleDepositTime = null;
    protected ElapsedTime depositTime = null;

    protected ElapsedTime liftTime = null;
    protected ElapsedTime pickupElapsedTime= null;
    protected CSCons.OuttakeWrist outtakeWristPosition = CSCons.OuttakeWrist.vertical;

    protected SampleMecanumDrive drive;

    protected PropFindRightProcessor.pos propPos = null;

    protected TrajectorySequence rightPurple, leftPurple, middlePurple;
    protected TrajectorySequence rightPurpleToStack, leftPurpleToStack, midPurpleToStack;
    protected TrajectorySequence stackToRightYellow, stackToLeftYellow,stackToMidYellow;
    protected TrajectorySequence toStackCycleGateLeft, toStackCycleGateRight, toStackCycleGateMid, toBackboardCycleGate, toStackCycleTruss, goBackboardCycleGate;
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

    protected void purpleDepositPath(){
        if (propPos == PropFindRightProcessor.pos.LEFT) {
            drive.followTrajectorySequenceAsync(leftPurple);

        } else if (propPos == PropFindRightProcessor.pos.RIGHT) {
            drive.followTrajectorySequenceAsync(rightPurple);

        } else {
            drive.followTrajectorySequenceAsync(middlePurple);
        }

        currentState = State.PURPLE_DEPOSIT;
        purpleDepositTime = new ElapsedTime();
    }

    protected void purpleDeposit(){
        if (purpleDepositTime.milliseconds()>500){
            outtakeTarget= CSCons.OuttakePosition.LOW_AUTO.getTarget();
        }
        if (drive.getBackSlides().getCurrentPosition()>50){
            drive.setOuttakeToGround();
        }
        if (!drive.isBusy()){
            if (liftTime==null) {
                drive.openFrontFinger();
                liftTime = new ElapsedTime();
            } else {
                liftTime=null;
                outtakeTarget = 0;
                drive.setOuttakeToTransfer();
                drive.intakeOverStack();
                drive.startIntake();
                currentState= State.TO_STACK;
                switch (propPos){
                    case LEFT:
                        drive.followTrajectorySequenceAsync(leftPurpleToStack);
                        break;
                    case RIGHT:
                        drive.followTrajectorySequenceAsync(rightPurpleToStack);
                        break;
                    case MID:
                        drive.followTrajectorySequenceAsync(midPurpleToStack);
                        break;
                }

            }


        }

    }

    protected void toStack(TrajectorySequence nextPath){
        if (drive.isBusy()) {
            drive.setOuttakeToTransfer();
        }
        if (!drive.isBusy()){
            if(pickupElapsedTime==null) {
                drive.intakeToTopStack();
                pickupElapsedTime = new ElapsedTime();
            } else if (pickupElapsedTime.milliseconds()>1000){
                drive.stopIntake();
                drive.raiseIntake();
                drive.outtakeToPickup();
                pickupElapsedTime = new ElapsedTime();
                currentState = State.BACKDROP_DEPOSIT_PATH;
                drive.followTrajectorySequenceAsync(nextPath);
//                switch (propPos){
//                    case LEFT:
//                        drive.followTrajectorySequenceAsync(stackToLeftYellow);
//                        break;
//                    case RIGHT:
//                        drive.followTrajectorySequenceAsync(stackToRightYellow);
//                        break;
//                    case MID:
//                        drive.followTrajectorySequenceAsync(stackToMidYellow);
//                        break;
//                }
            }
        }
    }

    protected void backdropDepositPath(State nextState, TrajectorySequence nextPath){
        if (pickupElapsedTime!=null &&  pickupElapsedTime.milliseconds()>250){
            drive.closeFingers();
            drive.revertIntake();
            pickupElapsedTime =null;
        }
        if (drive.getPoseEstimate().getX()>25){
            outtakeTarget = CSCons.OuttakePosition.AUTO.getTarget();
            drive.closeFingers();
            if (drive.getBackSlides().getCurrentPosition()>outtakeTarget- 200){
                drive.outtakeToBackdrop();
                drive.setWristServoPosition(CSCons.OuttakeWrist.flatLeft);
            } else if (drive.getBackSlides().getCurrentPosition()>10){
                drive.outtakeToBackdrop();
            }
            drive.stopIntake();
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
                    outtakeTarget=0;
                    drive.outtakeToTransfer();
                    drive.followTrajectorySequenceAsync(nextPath);
                }
                cycleCount++;
                currentState= nextState;
            }
        }

    }

    protected void toStackCycleGate(){
        if (drive.isBusy()) {
            drive.setOuttakeToTransfer();
        }
        if (!drive.isBusy()){
            if(pickupElapsedTime==null) {
                drive.intakeToTopStack();
                pickupElapsedTime = new ElapsedTime();
            } else if (pickupElapsedTime.milliseconds()>1000){
                drive.stopIntake();
                drive.raiseIntake();
                drive.outtakeToPickup();
                pickupElapsedTime = new ElapsedTime();
                currentState = State.TO_BACKBOARD_CYCLE;
               drive.followTrajectorySequenceAsync(toBackboardCycleGate);
            }
        }
    }

    protected  void park(){
        if (!drive.isBusy()){
            outtakeTarget = 0;
            drive.outtakeToTransfer();
        }
    }

}

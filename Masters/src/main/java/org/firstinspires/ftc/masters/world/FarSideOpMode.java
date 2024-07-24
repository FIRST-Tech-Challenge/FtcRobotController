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
import org.firstinspires.ftc.vision.VisionPortal;
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

        TO_GATE,
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
    protected ElapsedTime parkTime = null;

    protected ElapsedTime liftTime = null;
    protected ElapsedTime pickupElapsedTime= null;
    protected ElapsedTime retractElapsed= null;
    protected ElapsedTime shooterTime = null;
    protected CSCons.OuttakeWrist outtakeWristPosition = CSCons.OuttakeWrist.vertical;

    protected SampleMecanumDrive drive;

    protected PropFindRightProcessor.pos propPos = null;

    protected TrajectorySequence rightPurple, leftPurple, middlePurple;
    protected TrajectorySequence rightPurpleToStack, leftPurpleToStack, midPurpleToStack;
    protected TrajectorySequence stackToRightYellow, stackToLeftYellow,stackToMidYellow;
    protected TrajectorySequence toGateCycleLeft, toGateCycleMid, toGateCycleRight;
    protected TrajectorySequence toStackCycleGateLeft, toStackCycleGateRight, toStackCycleGateMid, toBackboardCycleGate, toStackCycleTruss, goBackboardCycleGate;
    protected TrajectorySequence toStackFromCenterGate;
    protected TrajectorySequence park, parkFromLeft, parkFromMid, parkFromRight;
    protected TrajectorySequence toStackFromPark;

    protected State currentState;
    protected int outtakeTarget = 0;
    protected boolean has2pixels= false;

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
        drive.closeFrontFingers();
        drive.initPlane();

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
        drive.enableAprilTag();
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

        if (!drive.isBusy()){
            if (depositTime==null){
                depositTime= new ElapsedTime();
                drive.setOuttakeToGround();
            }

            if (liftTime==null && depositTime!=null && depositTime.milliseconds()>1000) {

                drive.openFrontFinger();
                liftTime = new ElapsedTime();

            } else if (liftTime!=null && liftTime.milliseconds()>200){
                liftTime=null;
                outtakeTarget = 0;
                drive.setOuttakeToTransfer();

                drive.intakeToPosition4();
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

        } else {
            drive.liftOuttake();
        }

    }

    protected void toGate(){
        if (cycleCount==0){
            currentState=State.TO_STACK;
        } else {
            if (!drive.isBusy()) {
                outtakeTarget = 0;
                drive.outtakeToTransfer();

                currentState = State.TO_STACK;
                drive.followTrajectorySequenceAsync(toStackFromCenterGate);
            }
        }
    }

    protected void toStack(TrajectorySequence nextPath){
        if (drive.isBusy() && cycleCount==1) {
          drive.intakeToPosition1();
          drive.startIntake();
        }
        if (!drive.isBusy()){
            telemetry.addData("pickup elapsed", pickupElapsedTime);
            if(pickupElapsedTime==null) {
                telemetry.addData("move intake", "true");
                has2pixels = false;
                drive.outtakeToTransfer();
                if (cycleCount==0) {
                    drive.intakeToTopStack();
                } else{

                    drive.intakeToPosition2();
                }
                pickupElapsedTime = new ElapsedTime();
            }
            if (has2Pixels() && ! has2pixels ){
                has2pixels = true;
                telemetry.addData("has 2 pixels", "true");
                pickupElapsedTime = new ElapsedTime();
            } else {
                telemetry.addData("has 2 pixels", "false");
            }

            if (!has2pixels &&  cycleCount==1){
//                if (pickupElapsedTime.milliseconds()>1500){
//                    drive.intakeToPosition1();
//                } else
               if (pickupElapsedTime.milliseconds()>700){
                    drive.intakeToPosition1();
                }
            }

            if (!has2pixels && cycleCount==0){
                if (pickupElapsedTime.milliseconds()>500){
                    drive.intakeToPosition3();
                }
            }
            int wait =1000;
            if (cycleCount==1){
                wait = 3000;
            }

           if (pickupElapsedTime!=null && (pickupElapsedTime.milliseconds()>wait || (has2pixels && pickupElapsedTime.milliseconds()>300))  ){
               telemetry.addData("move intake", "done");
               drive.pushPixels();

              // drive.stopIntake();
                drive.raiseIntake();
              //  drive.outtakeToPickup();
                pickupElapsedTime = new ElapsedTime();
                currentState = State.BACKDROP_DEPOSIT_PATH;
               telemetry.addData("follow path", cycleCount);
                drive.followTrajectorySequenceAsync(nextPath);

            }
        }
    }

    protected void backdropDepositPath(State nextState, TrajectorySequence nextPath){
        if (pickupElapsedTime!=null && pickupElapsedTime.milliseconds()>100){
            drive.outtakeToPickup();
            drive.revertIntake();

        }

        if (pickupElapsedTime!=null &&  pickupElapsedTime.milliseconds()>250){
            drive.closeFingers();
            drive.revertIntake();
            drive.raiseTransferArm();

        }
        if (pickupElapsedTime!=null && pickupElapsedTime.milliseconds()>1000){
            drive.stopIntake();
            pickupElapsedTime =null;
        }
        if (drive.getPoseEstimate().getX()>25){
            outtakeTarget = CSCons.OuttakePosition.AUTO.getTarget();
            if (cycleCount==1){
                outtakeTarget= CSCons.OuttakePosition.AUTO.getTarget()+400;
            }
            drive.closeFingers();
            if (drive.getBackSlides().getCurrentPosition()>outtakeTarget- 200){
                drive.outtakeToBackdrop();
                drive.setWristServoPosition(getOuttakeWristPosition(propPos));
            } else if (drive.getBackSlides().getCurrentPosition()>10){
                drive.outtakeToBackdrop();
            }

        }

        if (!drive.isBusy()){
            drive.openFingers();
            //drive.shootPlane();
            if (depositTime==null){
                depositTime= new ElapsedTime();
            } else if (depositTime.milliseconds()>500){

                if (nextState == State.PARK){
                    drive.followTrajectorySequenceAsync(nextPath);
                    retractElapsed = new ElapsedTime();
                } else if (nextState == State.TO_STACK){
                    drive.intakeOverStack();

                    retractElapsed = new ElapsedTime();
//                    drive.outtakeToTransfer();
                    drive.followTrajectorySequenceAsync(nextPath);
                }
                cycleCount++;
                depositTime= null;
                currentState= nextState;
               // drive.initPlane();
            }
        }

    }

//    protected void toStackCycleGate(){
//        if (drive.isBusy()) {
//            drive.setOuttakeToTransfer();
//        }
//        if (!drive.isBusy()){
//            if(pickupElapsedTime==null) {
//                drive.intakeToTopStack();
//                pickupElapsedTime = new ElapsedTime();
//            } else if (pickupElapsedTime.milliseconds()>1000){
//                drive.stopIntake();
//                drive.raiseIntake();
//                drive.outtakeToPickup();
//                pickupElapsedTime = new ElapsedTime();
//                currentState = State.TO_BACKBOARD_CYCLE;
//               drive.followTrajectorySequenceAsync(toBackboardCycleGate);
//            }
//        }
//    }

    protected  void park(){
        if (!drive.isBusy()){
            outtakeTarget = 0;
            drive.outtakeToTransfer();
            drive.shootPlane();
        }
    }

    protected  void park(TrajectorySequence nextPath){
        if (!drive.isBusy()){
            outtakeTarget = 0;
            drive.outtakeToTransfer();
            if (parkTime==null) {
                parkTime = new ElapsedTime();
            }
            if (parkTime!=null && parkTime.milliseconds()>1000) {

                if (cycleCount <= 1) {
                    drive.startIntake();
                    drive.intakeToPosition2();
                    drive.followTrajectorySequenceAsync(nextPath);
                    currentState = State.TO_STACK;
                } else {
                    if (shooterTime==null) {
                        drive.shootPlane();
                        shooterTime = new ElapsedTime();
                    }
                }
            }

            if (shooterTime!=null && shooterTime.milliseconds()>700){
                drive.initPlane();
            }
        }
    }

    protected boolean has2Pixels(){
        return !drive.frontBreakBeam.getState() && !drive.backBreakBeam.getState();
    }

    public CSCons.OuttakeWrist getOuttakeWristPosition(PropFindRightProcessor.pos propPos){
        if (propPos== PropFindRightProcessor.pos.RIGHT){
            return CSCons.OuttakeWrist.flatLeft;
        } else {
            return CSCons.OuttakeWrist.flatRight;
        }
    }

    public CSCons.OuttakeWrist getOuttakeWristPosition(){
        return CSCons.OuttakeWrist.flatRight;
    }



}

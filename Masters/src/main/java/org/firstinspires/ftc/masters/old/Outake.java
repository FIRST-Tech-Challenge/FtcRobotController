package org.firstinspires.ftc.masters.old;

import static org.firstinspires.ftc.masters.old.CSCons.servo1Down;
import static org.firstinspires.ftc.masters.old.CSCons.servo1Up;
import static org.firstinspires.ftc.masters.old.CSCons.servo2Down;
import static org.firstinspires.ftc.masters.old.CSCons.servo2Up;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.components.Component;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class Outake implements Component {

    boolean hanging = false;

    CSCons.DriveMode driveMode;
    CSCons.OuttakePosition backSlidePos;

    DcMotor backSlides = null;
    DcMotor otherBackSlides = null;
    Servo wristServo;
    Servo outtakeServo1, outtakeServo2;
    Servo outtakeRotation;
    Servo outtakeMovement;
    Servo outtakeMovementRight;
    private CSCons.OuttakeWrist outtakeWristPosition;

    private CSCons.OuttakeState outtakeState;
    CSCons.TransferStatus transferStatus;

    //PID
    PIDController outtakeController;

    private long closeFingerWaitTime= 100;
    private long openFingerWaitTime = 100;
    private long wristWaitTime = 100;



    ElapsedTime fingersElapsedTime = null;
    ElapsedTime wristElapsedTime = null;
    ElapsedTime retractTime = null;

    private final double ticks_in_degrees = 384.5 / 180;
    public static double p = 0.01, i = 0, d = 0.0001;
    public static double f = 0.05;

    private int stackPosition =0;
    private int target;


    private HardwareMap hardwareMap = null;
    private Transfer transfer = null;
    private Telemetry telemetry;
    public Outake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap=hardwareMap;
        this.transfer = Transfer.getInstance(hardwareMap, telemetry);
        this.telemetry= telemetry;
        initializeHardware();
    }
    public void initializeHardware(){

        backSlides = hardwareMap.dcMotor.get("backSlides");
        otherBackSlides = hardwareMap.dcMotor.get("otherBackSlides");

        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovement = hardwareMap.servo.get("backSlideServo");
        outtakeMovementRight = hardwareMap.servo.get("backSlideServoRight");
        wristServo = hardwareMap.servo.get("wrist");

        outtakeServo1 = hardwareMap.servo.get("outtakeHook");
        outtakeServo2 = hardwareMap.servo.get("microHook");

        otherBackSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        otherBackSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        otherBackSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        driveMode= CSCons.DriveMode.NORMAL;
        outtakeState = CSCons.OuttakeState.ReadyToTransfer;

        backSlidePos= CSCons.OuttakePosition.BOTTOM;
        target = backSlidePos.getTarget();

        outtakeController = new PIDController(p, i, d);
        outtakeController.setPID(p, i, d);

        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);


        outtakeServo1.setPosition(CSCons.servo1Up);
        outtakeServo2.setPosition(CSCons.servo2Up);

        outtakeWristPosition = CSCons.OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
    }

    public void update(Gamepad gamepad1, CSCons.DriveMode driveMode){

            telemetry.addData("outtake state", outtakeState);

        if (driveMode== CSCons.DriveMode.HANG && this.driveMode!=driveMode){
            //setup hang
            setupHang();
            this.driveMode= driveMode;
        }

        switch (this.driveMode){

            case HANG:
                hang(gamepad1);
                break;
            case NORMAL:
                if (this.transfer.getCurrentTransferStatus()== CSCons.TransferStatus.MOVE_OUTTAKE){
                    setOuttakeToPickup();
                }
                if (this.transfer.getCurrentTransferStatus() == CSCons.TransferStatus.CLOSE_FINGERS){
                    closeFingers();
                }

                if (this.transfer.getCurrentTransferStatus()== CSCons.TransferStatus.DONE || this.transfer.getCurrentTransferStatus()== CSCons.TransferStatus.WAITING_FOR_PIXELS){

                    switch (outtakeState){
                        case ReadyToTransfer:
                            //grab pixels even if 2 pixels are not detected
                            if (gamepad1.right_stick_button){
                                //telemetry.addLine("pickup pixels");
                                transfer.setPickupOverride(true);
                            }

                            //realese pixels in tansfer
                            if (gamepad1.left_stick_button) { //set to transfer position
                                liftOuttakeFingers();
                                setOuttakeToTransfer();
                                transfer.setPickupOverride(false);
                                transfer.setCurrentTransferStatus(CSCons.TransferStatus.WAITING_FOR_PIXELS);
                            }

                            if (gamepad1.options) { //set to transfer position
                                setOuttakeToHub();
                            }

                            if (gamepad1.right_stick_x < -0.5 && backSlidePos == CSCons.OuttakePosition.AUTO){
                                liftOuttakeFingers();
                                try {
                                    Thread.sleep(500);
                                } catch (InterruptedException e) {
                                    throw new RuntimeException(e);
                                }
                                outtakeState = CSCons.OuttakeState.Retract;
                            }

                            setWristAndSlidesPosition(gamepad1);


                            break;
                        case MoveToTransfer:
                            if (backSlides.getCurrentPosition() < 50) {
                                liftOuttakeFingers();
                                outtakeState = CSCons.OuttakeState.ReadyToTransfer;
//                                backSlidePos= CSCons.OuttakePosition.BOTTOM;
//                                target= backSlidePos.getTarget();

                            }
                            break;
                        case Retract:
                            if (retractTime == null) {
                                liftOuttakeFingers();
                                setOuttakeToTransfer();
                                if (backSlides.getCurrentPosition() > CSCons.OuttakePosition.LOW.getTarget() + 400) {
                                    backSlidePos = CSCons.OuttakePosition.BOTTOM;
                                    target = backSlidePos.getTarget();
                                    outtakeState = CSCons.OuttakeState.MoveToTransfer;
                                } else {
                                    retractTime = new ElapsedTime();
                                }
                            } else  if (retractTime.milliseconds() > 500) {
                                backSlidePos = CSCons.OuttakePosition.BOTTOM;
                                target = backSlidePos.getTarget();
                                outtakeState = CSCons.OuttakeState.MoveToTransfer;
                                retractTime =  null;

                            }
                            wristServo.setPosition(outtakeWristPosition.getPosition());

                            break;
//                        case GrabPixels:
//                            if (transfer.getCurrentTransferStatus()== CSCons.TransferStatus.DONE){
//                                outtakeState= CSCons.OuttakeState.MoveToDrop;
//                                target= backSlidePos.getTarget();
//                            }
//                            break;
                        case MoveToDrop:
                            if (backSlides.getCurrentPosition() > 100) {
                                setOuttakeToBackdrop();
                            }
                            if (backSlides.getCurrentPosition() > backSlidePos.getTarget() - 100) {
                                outtakeState = CSCons.OuttakeState.ReadyToDrop;
                            }

                            break;
                        case ReadyToDrop:
                            setSlidesPosition(gamepad1);

                            setWristAndSlidesPosition(gamepad1);
                            wristServo.setPosition(outtakeWristPosition.getPosition());
                            dropPixels(gamepad1);

                            moveWristServo();
                            break;
                    }

                }

            }


        backSlidesMove(target);

    }

    private void hang(Gamepad gamepad){
        if (hanging && backSlides.getCurrentPosition() > 2000) {
           setOuttakeToBackdrop();
        }

        if (gamepad!=null && gamepad.a){
            //hang
            backSlidePos = CSCons.OuttakePosition.BOTTOM;
            target= backSlidePos.getTarget();
            hanging = true;
        }
    }

    private void setSlidesPosition(Gamepad gamepad){
        //TODO: what control for up/down target=target+10, target=target-10;
        if (outtakeState== CSCons.OuttakeState.ReadyToDrop){
            if (gamepad.left_bumper){
                target = target-10;
            }
            if (gamepad.right_bumper){
                target = target+10;
            }
            if (gamepad.right_stick_x<-0.7){
                outtakeState = CSCons.OuttakeState.Retract;
                setOuttakeToTransfer();
            }

        }

    }
    private void setupHang(){
        backSlidePos = CSCons.OuttakePosition.HIGH;
        setOuttakeToBackdrop();
        target= backSlidePos.getTarget();
    }

    private void liftSlide(CSCons.OuttakePosition outtakePosition){
        backSlidePos = outtakePosition;
        target= backSlidePos.getTarget();
        transfer.setTransferServoUp();
    }

    protected void backSlidesMove(int target) {

        int slidePos = backSlides.getCurrentPosition();

        double pid = outtakeController.calculate(slidePos, target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double liftPower = pid + ff;

        backSlides.setPower(liftPower);
        otherBackSlides.setPower(liftPower);
    }

    public CSCons.OuttakeState getOuttakeState() {
        return outtakeState;
    }

    public void setOuttakeState(CSCons.OuttakeState outtakeState) {
        this.outtakeState = outtakeState;
    }

    public void setOuttakeToTransfer(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeWristPosition = CSCons.OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
    }

    public void setOuttakeToPickup(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAnglePickup);
        outtakeMovement.setPosition(CSCons.wristOuttakePickup);
        outtakeMovementRight.setPosition(CSCons.wristOuttakePickup);
        outtakeWristPosition = CSCons.OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
    }

    public void setOuttakeToBackdrop(){
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementBackdrop);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementBackdrop);
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleBackdrop);
    }

    public void setOuttakeToHub(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleBackdrop);
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementHub);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementHub);
        backSlidePos= CSCons.OuttakePosition.AUTO;
        target=backSlidePos.getTarget();
    }

    public void closeFingers(){
        outtakeServo1.setPosition(servo1Down);
        outtakeServo2.setPosition(servo2Down);
        fingersElapsedTime = new ElapsedTime();
    }

    protected void liftOuttakeFingers(){
        outtakeServo1.setPosition(servo1Up);
        outtakeServo2.setPosition(servo2Up);
        fingersElapsedTime = new ElapsedTime();
    }

    public boolean areFingersOpened(){
        return fingersElapsedTime.milliseconds()>openFingerWaitTime;
    }

    public boolean areFingersClosed(){
        return  fingersElapsedTime.milliseconds()>closeFingerWaitTime;
    }

    public void setOuttakeWristPosition(CSCons.OuttakeWrist outtakeWristPosition) {
        this.outtakeWristPosition = outtakeWristPosition;
    }

    protected void setWristAndSlidesPosition(Gamepad gamepad){
        if (outtakeState == CSCons.OuttakeState.ReadyToDrop ) {
            if (gamepad.dpad_left) {
                outtakeWristPosition = CSCons.OuttakeWrist.angleRight;
            } else if (gamepad.dpad_up) {
                outtakeWristPosition = CSCons.OuttakeWrist.vertical;
            } else if (gamepad.dpad_right) {
                outtakeWristPosition = CSCons.OuttakeWrist.angleLeft;
            } else if (gamepad.dpad_down) {
                outtakeWristPosition = CSCons.OuttakeWrist.verticalDown;
            } else if (gamepad.touchpad && gamepad.touchpad_finger_1_x < -0.1) {
                outtakeWristPosition = CSCons.OuttakeWrist.flatRight;
            } else if (gamepad.touchpad && gamepad.touchpad_finger_1_x > 0.1) {
                outtakeWristPosition = CSCons.OuttakeWrist.flatLeft;
            }



        } else {
            if (gamepad.y) {
                outtakeWristPosition= CSCons.OuttakeWrist.vertical;
                if (gamepad.left_bumper){
                    backSlidePos= CSCons.OuttakePosition.MID;
                } else if (gamepad.right_bumper){
                    backSlidePos= CSCons.OuttakePosition.HIGH;
                } else {
                    backSlidePos= CSCons.OuttakePosition.LOW;
                }
                target = backSlidePos.getTarget();
                setOuttakeToBackdrop();
                outtakeState= CSCons.OuttakeState.MoveToDrop;

            }
            if (gamepad.a){
                outtakeWristPosition= CSCons.OuttakeWrist.verticalDown;
                if (gamepad.left_bumper){
                    backSlidePos= CSCons.OuttakePosition.MID;
                } else if (gamepad.right_bumper){
                    backSlidePos= CSCons.OuttakePosition.HIGH;
                } else {
                    backSlidePos= CSCons.OuttakePosition.LOW;
                }
                target = backSlidePos.getTarget();
                setOuttakeToBackdrop();
                outtakeState= CSCons.OuttakeState.MoveToDrop;
            }
            if (gamepad.x){
                outtakeWristPosition= CSCons.OuttakeWrist.flatLeft;
                if (gamepad.left_bumper){
                    backSlidePos= CSCons.OuttakePosition.MID;
                } else if (gamepad.right_bumper){
                    backSlidePos= CSCons.OuttakePosition.HIGH;
                } else {
                    backSlidePos= CSCons.OuttakePosition.LOW;
                }
                target = backSlidePos.getTarget();
                setOuttakeToBackdrop();
                outtakeState= CSCons.OuttakeState.MoveToDrop;
            }
            if (gamepad.b){
                outtakeWristPosition= CSCons.OuttakeWrist.flatRight;
                if (gamepad.left_bumper){
                    backSlidePos= CSCons.OuttakePosition.MID;
                } else if (gamepad.right_bumper){
                    backSlidePos= CSCons.OuttakePosition.HIGH;
                } else {
                    backSlidePos= CSCons.OuttakePosition.LOW;
                }
                target = backSlidePos.getTarget();
                setOuttakeToBackdrop();
                outtakeState= CSCons.OuttakeState.MoveToDrop;
            }

        }



    }

    public void moveWristServo(){
        if (outtakeWristPosition == CSCons.OuttakeWrist.angleLeft) {
            wristServo.setPosition(CSCons.wristAngleLeft);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.angleRight) {
            wristServo.setPosition(CSCons.wristAngleRight);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.vertical) {
            wristServo.setPosition(CSCons.wristVertical);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.flatLeft) {
            wristServo.setPosition(CSCons.wristFlatLeft);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.flatRight) {
            wristServo.setPosition(CSCons.wristFlatRight);
        }
        if (outtakeWristPosition == CSCons.OuttakeWrist.verticalDown) {
            wristServo.setPosition(CSCons.wristVerticalDown);
        }
    }

    public CSCons.OuttakeWrist getOuttakeWristPosition() {
        return outtakeWristPosition;
    }

    protected void dropPixels(Gamepad gamepad){
        if(gamepad.y){
            if (outtakeWristPosition== CSCons.OuttakeWrist.vertical){
                outtakeServo2.setPosition(servo2Up);
            } else if (outtakeWristPosition== CSCons.OuttakeWrist.angleLeft){
                outtakeServo2.setPosition(servo2Up);
            } else if (outtakeWristPosition== CSCons.OuttakeWrist.angleRight){
                outtakeServo2.setPosition(servo2Up);
            } else if (outtakeWristPosition== CSCons.OuttakeWrist.verticalDown){
                outtakeServo1.setPosition(servo1Up);
            }
        }

        if (gamepad.a){
            if (outtakeWristPosition== CSCons.OuttakeWrist.vertical){
                outtakeServo1.setPosition(servo1Up);
            }
            if (outtakeWristPosition== CSCons.OuttakeWrist.verticalDown){
                outtakeServo2.setPosition(servo2Up);
            }
        }

        if (gamepad.b){
            if (outtakeWristPosition== CSCons.OuttakeWrist.flatLeft){
                outtakeServo2.setPosition(servo2Up);
            } else if (outtakeWristPosition== CSCons.OuttakeWrist.flatRight){
                outtakeServo1.setPosition(servo1Up);
            } else if (outtakeWristPosition== CSCons.OuttakeWrist.angleRight) {
                outtakeServo1.setPosition(servo1Up);
            }
        }

        if (gamepad.x){
            if (outtakeWristPosition== CSCons.OuttakeWrist.flatRight){
                outtakeServo2.setPosition(servo2Up);
            } else if (outtakeWristPosition== CSCons.OuttakeWrist.flatLeft){
                outtakeServo1.setPosition(servo1Up);
            }else if (outtakeWristPosition== CSCons.OuttakeWrist.angleLeft){
                outtakeServo1.setPosition(servo1Up);
            }
        }

        if (gamepad.right_stick_y>0.2 && Math.abs(gamepad.right_stick_x)<0.2){
            liftOuttakeFingers();
        }
        if (gamepad.right_stick_y<-0.6 && Math.abs(gamepad.right_stick_x)<0.2){
            closeFingers();
        }

    }
}

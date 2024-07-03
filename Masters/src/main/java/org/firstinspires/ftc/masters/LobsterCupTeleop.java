package org.firstinspires.ftc.masters;


import static org.firstinspires.ftc.masters.CSCons.servo1Down;
import static org.firstinspires.ftc.masters.CSCons.servo1Up;
import static org.firstinspires.ftc.masters.CSCons.servo2Down;
import static org.firstinspires.ftc.masters.CSCons.servo2Up;
import static org.firstinspires.ftc.masters.CSCons.transferPush;
import static org.firstinspires.ftc.masters.CSCons.transferUp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons.DriveMode;
import org.firstinspires.ftc.masters.CSCons.HookPosition;
import org.firstinspires.ftc.masters.CSCons.OuttakePosition;
import org.firstinspires.ftc.masters.CSCons.OuttakeState;
import org.firstinspires.ftc.masters.CSCons.OuttakeWrist;
import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;
import org.firstinspires.ftc.masters.components.Shooter;
import org.firstinspires.ftc.masters.components.Transfer;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name = "Lobster Cup Teleop", group = "competition")
public class LobsterCupTeleop extends LinearOpMode {

    DriveTrain drivetrain = null;
    Outake outake;
    Intake intake;
    Transfer transfer;
    Shooter shooter;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();


//    DcMotor intake = null;
//    Servo intakeHeight = null;

    Servo planeRaise;

    Servo outtakeRotation;
    Servo outtakeMovement;
    Servo outtakeMovementRight;

//    Servo transferServo;

    private Servo wristServo;
    private Servo outtakeServo1, outtakeServo2;
    private OuttakeWrist outtakeWristPosition;



    int numberOfPixelsInRamp= 0;

    double outtakeRotationTarget;

    CSCons.TransferStatus currentTransferStatus;


    ElapsedTime liftFingers = new ElapsedTime();


//    private enum OuttakeWrist{
//        flatRight (CSCons.wristFlatRight), angleRight(CSCons.wristAngleRight), verticalDown(CSCons.wristVerticalDown),
//        vertical(CSCons.wristVertical), angleLeft(CSCons.wristAngleLeft), flatLeft(CSCons.wristFlatLeft);
//
//        double position;
//        private OuttakeWrist(double position){
//            this.position= position;
//        }
//
//        public double getPosition(){
//            return position;
//        }
//    }

    private DriveMode driveMode = DriveMode.NORMAL;
    private OuttakeState outtakeState = OuttakeState.ReadyToTransfer;

    private CSCons.IntakeDirection intakeDirection = CSCons.IntakeDirection.OFF;
    private HookPosition hookPosition = HookPosition.OPEN;

    @Override
    public void runOpMode() {

        drivetrain = new DriveTrain(hardwareMap);
        transfer = Transfer.getInstance(hardwareMap);

        outake = new Outake(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);


        /*
    CONTROLS:
    TODO: Left stick - drive
    TODO: Triggers - turning
    DRIVE CONTROLS:
    TODO: Right stick - intake:
        up - intake on
        down - intake reverse
        right - intake off
    TODO: D-pad - stack control
        up - intake top // Done
        down - intake bottom // Done
        left - intake down by 1
        right - stack up by 1
    TODO: Touchpad:
        two finger click - drone
    TODO: stick Buttons:
        left stick - release transfer
        right stick - transfer
    SCORE CONTROLS:
    TODO: ABXY & Back Bumper - outtake control initial
        A - outtake vertical
        Y - outtake vertical flip
        X - outtake horizontal
        B - outtake horizontal flip
        LB - outtake mid
        RB - outtake high
        NB - outtake low
    TODO: ABXY & Back Bumper - outtake control final
        vertical:
            A - bottom pixel drop
            Y - top pixel drop
        horizontal:
            X - left pixel drop
            B - right pixel drop
        angled right:
            X - left pixel drop
            Y - right pixel drop
        angled left:
            Y - left pixel drop
            B - right pixel drop
        right stick:
            up - grab
            down - drop
            right - drive mode
        dpad:
            up - vertical
            down - vertical flip
            left - rotate left
            right - rotate right
       touchpad:
            left - flat
            right - flat flip
    HANG CONTROLS:
    TODO: PS button - hang mode
        sets up hang
        A - hang
        */


        planeRaise = hardwareMap.servo.get("planeRaise");
        wristServo = hardwareMap.servo.get("wrist");


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);

        hookPosition = HookPosition.OPEN;
        planeRaise.setPosition(CSCons.droneFlat);

//        outtakeController = new PIDController(p, i, d);
//        outtakeController.setPID(p, i, d);

        outtakeServo1.setPosition(CSCons.servo1Up);
        outtakeServo2.setPosition(CSCons.servo2Up);


        outtakeWristPosition = OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);



        ElapsedTime intakeElapsedTime = null, outtakeElapsedTime = null, pickupElapsedTime = null;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean buttonPushed = false;
//        boolean intakeStackButtonPushed= false;
        boolean slideNeedstoGoDown = false;
        ElapsedTime slidesElapsedTime;
        ElapsedTime buttonPushedTime=new ElapsedTime();

        waitForStart();

        runtime.reset();
        ElapsedTime elapsedTime;
        ElapsedTime colorSensorElapsedTime = null;
//        ElapsedTime stackElapsedTime = new ElapsedTime();
        ElapsedTime closeClawElapsedTime = null;

        ElapsedTime transferElapsedTime = null;

        double[] clawStack = {CSCons.clawArmGround, CSCons.clawArm2, CSCons.clawArm3, CSCons.clawArm4, CSCons.clawArm5};
        int stackPosition = 0;
        boolean wristButtonPressed = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("DRIVE MODE", driveMode.name());


            if (gamepad1.ps){
                driveMode= DriveMode.HANG;
            }

            drivetrain.drive(gamepad1);
            transfer.update(gamepad1, outake.getOuttakeState());
            intake.update(gamepad1,driveMode, outake.getOuttakeState());
            outake.update(gamepad1,driveMode);
            shooter.update(gamepad1);



            if (gamepad2.dpad_down) {
                target -= 15;
            }

            //TODO: how do we do that with 1 controller
            if (gamepad1.x) {
//                backSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                otherBackSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                otherBackSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            outake.update(gamepad1, gamepad2, driveMode);

            switch (driveMode) {
                case NORMAL:

                    drivetrain.drive(gamepad1);

                    break;
                case HANG:
                    drivetrain.drive(gamepad1);
                    break;
                case END_GAME:
                    drivetrain.drive(gamepad1);
                    if (gamepad2.y) {
                        planeRaise.setPosition(CSCons.droneShooting);
                        elapsedTime = new ElapsedTime();
                        while (elapsedTime.time(TimeUnit.MILLISECONDS) < 500 && opModeIsActive()) {

                        }
                        planeRaise.setPosition((CSCons.droneFlat));
                    }

//                    if (gamepad2.left_bumper) { //down
//                        backSlidePos = OuttakePosition.BOTTOM;
//                        target = backSlidePos.getTarget();
//                    }
//
//                    if (gamepad2.right_bumper) { //up
//                        backSlidePos = OuttakePosition.HIGH;
//
//                        target = backSlidePos.getTarget();
//                        intakeHeight.setPosition(CSCons.intakeInit);
//                    }
//
//                    if (backSlides.getCurrentPosition() > 2000) {
//                        outtakeRotation.setPosition(CSCons.wristOuttakeAngleBackdrop);
//                        outtakeMovement.setPosition(CSCons.wristOuttakeMovementBackdrop);
//                        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementBackdrop);
//                    }

                    break;

            }


            //intakeSlidesMove(intakeSlideTarget);

            if (driveMode != DriveMode.END_GAME) {


                if (gamepad1.y) {
                    planeRaise.setPosition(CSCons.droneShooting);
                    elapsedTime = new ElapsedTime();
                    while (elapsedTime.time(TimeUnit.MILLISECONDS) < 500 && opModeIsActive()) {

                    }
                    planeRaise.setPosition((CSCons.droneFlat));
                }
//                if (gamepad1.right_stick_y >= 0.5 && !buttonPushed) {
//                    //intake start/stop
//                    if (intakeDirection == CSCons.IntakeDirection.OFF || intakeDirection == CSCons.IntakeDirection.BACKWARD) {
//                        intakeDirection = CSCons.IntakeDirection.ON;
//                        intake.setPower(CSCons.speed);
//                    } else {
//                        intakeDirection = CSCons.IntakeDirection.OFF;
//                        intake.setPower(0);
//                    }
//                    buttonPushed = true;
//                }
//                if (gamepad1.right_stick_x > 0.5) {
//                    buttonPushed = false;
//                }

//                if (gamepad1.right_stick_x < 0.5 && !buttonPushed) {
//                    if (intakeDirection == CSCons.IntakeDirection.OFF || intakeDirection == CSCons.IntakeDirection.ON) {
//                        intakeDirection = CSCons.IntakeDirection.BACKWARD;
//                        intake.setPower(-CSCons.speed);
//                        stackPosition=5;
//                        intakeHeight.setPosition(CSCons.intakeAboveTop);
//                    } else {
//                        intakeDirection = CSCons.IntakeDirection.OFF;
//                        intake.setPower(0);
//                    }
//                    buttonPushed = true;
//                }

                // Use the D-pad to control the intake when it is not ready to drop
//                if (gamepad2.dpad_down && outtakeState!=OuttakeState.ReadyToDrop) {
//                    intakeHeight.setPosition(CSCons.intakeBottom);
//                    stackPosition = 0;
//                }
//                if (gamepad2.dpad_up && outtakeState!=OuttakeState.ReadyToDrop) {
//                    stackPosition = 5;
//                    intakeHeight.setPosition(CSCons.intakeAboveTop);
//                }

//                // Change stack position
//                if (gamepad2.dpad_left && !intakeStackButtonPushed && outtakeState!=OuttakeState.ReadyToDrop) {
//                    stackPosition--;
//                    intakeStackButtonPushed = true;
//                }
//                if (gamepad2.dpad_right && !intakeStackButtonPushed && outtakeState!=OuttakeState.ReadyToDrop) {
//                    stackPosition++;
//                    intakeStackButtonPushed = true;
//                }

                // Clear button press status
//                if (!gamepad2.dpad_left && !gamepad2.dpad_right && outtakeState!=OuttakeState.ReadyToDrop) {
//                    intakeStackButtonPushed = false;
//                }

                if (gamepad2.left_bumper && driveMode != DriveMode.END_GAME) { //down
                    target -= 10;
                }

                if (gamepad2.right_bumper && driveMode != DriveMode.END_GAME) { //up
                    target += 10;
                }
//
//                switch (stackPosition) {
//                    case 0:
//                        intakeHeight.setPosition(CSCons.intakeBottom);
//                        break;
//                    case 1:
//                        intakeHeight.setPosition(CSCons.intake2);
//                        break;
//                    case 2:
//                        intakeHeight.setPosition(CSCons.intake3);
//                        break;
//                    case 3:
//                        intakeHeight.setPosition(CSCons.intake4);
//                        break;
//                    case 4:
//                        intakeHeight.setPosition(CSCons.intake5);
//                        break;
//                    case 5:
//                        intakeHeight.setPosition(CSCons.intakeAboveTop);
//                        break;
//                }
            }



                    switch (outtakeState) {
                        case ReadyToTransfer:

//                            if (gamepad2.left_stick_y>0.5 && Math.abs(gamepad2.left_stick_x)<0.5 ) { // if press x and hook is closed, open hook
//                                outtakeMovement.setPosition(CSCons.wristOuttakePickup);
//                                outtakeMovementRight.setPosition(CSCons.wristOuttakePickup);
//                                outtakeRotation.setPosition(CSCons.wristOuttakeAnglePickup);
//                                pickupElapsedTime = new ElapsedTime();
//
//                            }
//                            if (gamepad2.left_stick_y<-0.5 && Math.abs(gamepad2.left_stick_x)<0.5 ) { // closes hook with x
//
//                                outtakeServo1.setPosition(servo1Up);
//                                outtakeServo2.setPosition(servo2Up);
//
//                                outtakeMovement.setPosition(CSCons.wristOuttakeMovementM);
//                                outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementM);
//
//                                outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
//                                outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
//                                outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);
//                            }
//
//                            if (pickupElapsedTime!=null &&  pickupElapsedTime.milliseconds()>250){
//                                outtakeServo1.setPosition(servo1Down);
//                                outtakeServo2.setPosition(servo2Down);
//                                pickupElapsedTime =null;
//                            }


                        if (gamepad2.left_trigger > 0.5) {
                            backSlidePos = OuttakePosition.LOW;
                            outtakeElapsedTime = closeHook();
                            outtakeState = OuttakeState.ClosingHook;
                            transferServo.setPosition(transferUp);

                        }
                        if (gamepad2..right_trigger > 0.5) {
                            backSlidePos = OuttakePosition.MID;
                            outtakeElapsedTime = closeHook();
                            outtakeState = OuttakeState.ClosingHook;
                            transferServo.setPosition(transferUp);
                        }

                            if (gamepad2.dpad_left ) {
                                outtakeWristPosition = OuttakeWrist.angleRight;
                            } else if (gamepad2.dpad_up ) {
                                outtakeWristPosition = OuttakeWrist.vertical;
                            } else if (gamepad2.dpad_right ) {
                                outtakeWristPosition = OuttakeWrist.angleLeft;
                            } else if (gamepad2.dpad_down ) {
                                outtakeWristPosition = OuttakeWrist.verticalDown;
                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x<-0.1){
                                outtakeWristPosition = OuttakeWrist.flatRight;
                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x>0.1){
                                outtakeWristPosition= OuttakeWrist.flatLeft;
                            }


                            break;
                        case MoveToTransfer:
                            if (backSlides.getCurrentPosition() < 50) {
                              liftOuttakeFingers();
                                outtakeState = OuttakeState.ReadyToTransfer;
                                target = 0;
                            }


                            //if touch sensor => ready to transfer
                            break;
                        case ReadyToDrop:

                            if (gamepad2.dpad_left ) {
                                outtakeWristPosition = OuttakeWrist.angleRight;

                            } else if (gamepad2.dpad_up ) {
                                outtakeWristPosition = OuttakeWrist.vertical;
                            } else if (gamepad2.dpad_right ) {

                                outtakeWristPosition = OuttakeWrist.angleLeft;

                            } else if (gamepad2.dpad_down ) {
                                outtakeWristPosition = OuttakeWrist.verticalDown;

                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x<-0.1){
                                outtakeWristPosition = OuttakeWrist.flatRight;
                            } else if (gamepad2.touchpad && gamepad2.touchpad_finger_1_x>0.1){
                                outtakeWristPosition= OuttakeWrist.flatLeft;
                            }

                           setWristServoPosition();

                            if(gamepad2.y){
                                if (outtakeWristPosition== OuttakeWrist.vertical){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.angleLeft){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.angleRight){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.verticalDown){
                                    outtakeServo1.setPosition(servo1Up);
                                }
                            }

                            if (gamepad2.a){
                                if (outtakeWristPosition== OuttakeWrist.vertical){
                                    outtakeServo1.setPosition(servo1Up);
                                }
                                if (outtakeWristPosition== OuttakeWrist.verticalDown){
                                    outtakeServo2.setPosition(servo2Up);
                                }
                            }

                            if (gamepad2.b){
                                if (outtakeWristPosition== OuttakeWrist.flatLeft){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.flatRight){
                                    outtakeServo1.setPosition(servo1Up);
                                } else if (outtakeWristPosition== OuttakeWrist.angleRight) {
                                    outtakeServo1.setPosition(servo1Up);
                                }
                            }

                            if (gamepad2.x){
                                if (outtakeWristPosition== OuttakeWrist.flatRight){
                                    outtakeServo2.setPosition(servo2Up);
                                } else if (outtakeWristPosition== OuttakeWrist.flatLeft){
                                    outtakeServo1.setPosition(servo1Up);
                                }else if (outtakeWristPosition== OuttakeWrist.angleLeft){
                                    outtakeServo1.setPosition(servo1Up);
                                }
                            }


                            if (gamepad2.left_trigger > 0.5) {
                                backSlidePos = OuttakePosition.LOW;
                            }
                            if (gamepad2.right_trigger > 0.5) {
                                backSlidePos = OuttakePosition.MID;
                                target = backSlidePos.getTarget();
                            }

                            if (gamepad2.left_stick_y > 0.2) {

                                liftOuttakeFingers();
                                setOuttakeToTransfer();

                                //setup slides
                                if (backSlides.getCurrentPosition() > OuttakePosition.LOW.getTarget() + 400) {
                                    backSlidePos = OuttakePosition.BOTTOM;
                                    target = backSlidePos.getTarget();
                                    outtakeState = OuttakeState.MoveToTransfer;
                                } else if (!slideNeedstoGoDown) {
                                    outtakeElapsedTime = new ElapsedTime();
                                    slideNeedstoGoDown = true;
                                }
                            }

                            if (gamepad2.right_stick_y>0.2){
                                liftOuttakeFingers();
                                liftFingers= new ElapsedTime();
                            }
                            if (gamepad2.right_stick_y<-0.2  && liftFingers.milliseconds()>200){
                                outtakeServo1.setPosition(servo1Down);
                                outtakeServo2.setPosition(servo2Down);
                            }



                            if (slideNeedstoGoDown && outtakeElapsedTime != null && outtakeElapsedTime.milliseconds() > 500) {
                                backSlidePos = OuttakePosition.BOTTOM;
                                target = backSlidePos.getTarget();
                                outtakeState = OuttakeState.MoveToTransfer;
                                slideNeedstoGoDown = false;
                            }


                            //what button to mode back to transfer?
                            // what order to move (slide down or flip first?

                            break;
                        case MoveToDrop:
                            if (backSlides.getCurrentPosition() > 100) {
                                outtakeMovement.setPosition(CSCons.wristOuttakeMovementBackdrop);
                                outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementBackdrop);
                                outtakeRotationTarget = CSCons.wristOuttakeAngleBackdrop;
                                outtakeRotation.setPosition(outtakeRotationTarget);
                            }
                            if (backSlides.getCurrentPosition() > backSlidePos.getTarget() - 100) {
                                outtakeState = OuttakeState.ReadyToDrop;

                            }
                            break;
                        case Align:
                            break;
                        case BackUp:
                            break;
                        case ClosingHook:
                            if (outtakeElapsedTime != null && outtakeElapsedTime.time(TimeUnit.MILLISECONDS) > CSCons.transferToScoreOuttake) {
                                outtakeState = OuttakeState.MoveToDrop;
                                target = backSlidePos.getTarget();
                            }
                            break;
                    }



//            telemetry.addData("left y", gamepad1.left_stick_y);
//            telemetry.addData("left x", gamepad1.left_stick_x);
//            telemetry.addData("right x", gamepad1.right_stick_x);
//            telemetry.addData("Arm", clawArm.getPosition());
//            telemetry.addData("backSlides", backSlides.getCurrentPosition()) ;
//            telemetry.addData("intakeSides", intakeSlides.getCurrentPosition());// 1725, 2400,
//            telemetry.addData("time", runtime.time());
//            telemetry.addData("Status", colorSensor.status());
//            telemetry.addData("Outtake state", outtakeState.name());
//            telemetry.addData("Intake state", intakeState.name());
//            telemetry.addData("back left", leftRearMotor.getCurrentPosition());
//            telemetry.addData("back right", rightRearMotor.getCurrentPosition());
//            telemetry.addData("front left", leftFrontMotor.getCurrentPosition());
//            telemetry.addData("front right", rightFrontMotor.getCurrentPosition());
//            telemetry.addData("TOUCH:", touchBucket.isPressed());
//            telemetry.update();

            telemetry.addData("Wrist Position: ", outake.getOuttakeWristPosition().name());


                telemetry.update();
            }
        }



//    protected void backSlidesMove(int target) {
//
//        int slidePos = backSlides.getCurrentPosition();
//
//        double pid = outtakeController.calculate(slidePos, target);
//
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
//
//        double liftPower = pid + ff;
//
//        backSlides.setPower(liftPower);
//        otherBackSlides.setPower(liftPower);
//    }


    protected ElapsedTime closeHook(){
        ElapsedTime time = new ElapsedTime();
        hookPosition = HookPosition.CLOSED;
//        outtakeServo1.setPosition(servo1Down);
//        outtakeServo2.setPosition(servo2Down);
        return time;
    }

    protected void liftOuttakeFingers(){
        outtakeServo1.setPosition(servo1Up);
        outtakeServo2.setPosition(servo2Up);
    }

    protected void setOuttakeToTransfer(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);
        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeWristPosition = OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
    }

    protected void setOuttakeToPickup(){
        outtakeRotation.setPosition(CSCons.wristOuttakeAnglePickup);
        outtakeMovement.setPosition(CSCons.wristOuttakePickup);
        outtakeMovementRight.setPosition(CSCons.wristOuttakePickup);
        outtakeWristPosition = OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);
    }

//    protected void setWristServoPosition(){
//        if (outtakeWristPosition == OuttakeWrist.angleLeft) {
//            wristServo.setPosition(CSCons.wristAngleLeft);
//        }
//        if (outtakeWristPosition == OuttakeWrist.angleRight) {
//            wristServo.setPosition(CSCons.wristAngleRight);
//        }
//        if (outtakeWristPosition == OuttakeWrist.vertical) {
//            wristServo.setPosition(CSCons.wristVertical);
//        }
//        if (outtakeWristPosition == OuttakeWrist.flatLeft) {
//            wristServo.setPosition(CSCons.wristFlatLeft);
//        }
//        if (outtakeWristPosition == OuttakeWrist.flatRight) {
//            wristServo.setPosition(CSCons.wristFlatRight);
//        }
//        if (outtakeWristPosition == OuttakeWrist.verticalDown) {
//            wristServo.setPosition(CSCons.wristVerticalDown);
//        }
//    }

//    protected boolean has2Pixels(){
//        return !frontBreakBeam.getState() && !backBreakBeam.getState();
//    }

}

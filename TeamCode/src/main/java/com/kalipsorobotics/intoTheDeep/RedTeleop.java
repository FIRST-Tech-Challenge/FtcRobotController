/*
package com.kalipsorobotics.intoTheDeep;

import android.os.SystemClock;
import android.util.Log;

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePigeonAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
import com.kalipsorobotics.actions.sequences.IntakeSequence;
import com.kalipsorobotics.actions.sequences.TransferSequence;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.modules.ColorDetector;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.modules.RevLED;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends LinearOpMode {

    // BUTTON CONTRACT:  https://docs.google.com/document/d/1XlSuL8Y8_j8Tde5NbMEa1k2LSuc72HpBp4LqAUZ_pjQ/edit?tab=t.0

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        SparkfunOdometry sparkfunOdometry = new SparkfunOdometry(driveTrain, opModeUtilities, 0, 0, Math.toRadians(0));
        DriveAction driveAction = new DriveAction(driveTrain);
        Intake intake = new Intake(opModeUtilities);
        IntakeNoodleAction intakeNoodleAction = new IntakeNoodleAction(intake);
        IntakePivotAction intakePivotAction = new IntakePivotAction(intake);
        IntakeDoorAction intakeDoorAction = new IntakeDoorAction(intake);
        IntakeLinkageAction intakeLinkageAction = new IntakeLinkageAction(intake);
        Outtake outtake = new Outtake(opModeUtilities);
        OuttakePivotAction outtakePivotAction = new OuttakePivotAction(outtake);
        OuttakeSlideAction outtakeSlideAction = new OuttakeSlideAction(outtake);
        OuttakeClawAction outtakeClawAction = new OuttakeClawAction(outtake);
        OuttakePigeonAction outtakePigeonAction = new OuttakePigeonAction(outtake);
        TransferSequence transferSequence = new TransferSequence(hardwareMap, opModeUtilities, outtake, intake);
        IntakeSequence intakeSequence = new IntakeSequence(intakePivotAction, intakeLinkageAction);
        ColorDetector colorDetector = new ColorDetector(opModeUtilities, hardwareMap);


        boolean prevGamePadY = false;
        boolean prevGamePadX = false;
        boolean prevGamePadB = false;
        boolean prevGamePadA = false;
        boolean prevDpadLeft = false;
        boolean prevDpadUp = false;
        boolean prevDpadRight = false;
        boolean retracted = true;

        //CHANGE ACCORDING TO ALLIANCE

        boolean isRed = true;
        boolean takeInYellow = true;

        intakeLinkageAction.retract();
        intakePivotAction.moveUp();
        intakeDoorAction.close();

        outtakeClawAction.close();
        outtakePivotAction.moveIn();

        //Pigeon
        outtakePigeonAction.setPosition(OuttakePigeonAction.OUTTAKE_PIGEON_IN_POS);

        waitForStart();

        while (opModeIsActive()) {

            //Drive
            driveAction.move(gamepad1);

            //INTAKE
            //Noodle
            if (gamepad2.dpad_right && !prevDpadRight) {
                takeInYellow = !takeInYellow;
                Log.d ("teleop", "taking in yellow is " + takeInYellow);
            }
            //press dpad right to toggle taking in yellow
            if (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
                colorDetector.cycle(isRed, takeInYellow, intakeNoodleAction);
                Log.d("teleop", "intake cycling");
            } else if (gamepad2.left_bumper) {
                intakeNoodleAction.reverse();
                Log.d("teleop", "intake reversing");
            } else {
                intakeNoodleAction.stop();
            }


            //Pivot TODO make generic toggle
            if (gamepad2.x && !prevGamePadX) {
                intakePivotAction.togglePosition();
                Log.d("teleop", "intake toggled");
            }
//            //drive logging for sparkfun position
//            if (gamepad1.a) {
//                Log.d("teleop", "x: " + sparkfunOdometry.updatePosition().getX() + "y: " +
//                        sparkfunOdometry.updatePosition().getY());
//            }

            //Door
            //TODO
//            if (gamepad2.b && !prevGamePadB) {
//                transferSequence.sequence();
//                retracted = true;
//                Log.d("teleop", "transfering...");
//            }
            //save for later
            //Linkage
            if (gamepad2.a) {
                intakeSequence.setUpCheckDone();
            }
            if (gamepad2.a && !prevGamePadA || !intakeSequence.checkDone(1001)) {
                if (retracted) {
                    intakeLinkageAction.extend();
                    if (intakeSequence.checkDone(1000)) {
                        intakePivotAction.moveDown();
                        //}
                        retracted = false;
                        Log.d("teleop", "intake extended");
                    }
                }
            }
            if (gamepad2.b && !prevGamePadB){
                intakeLinkageAction.retract();
                intakePivotAction.moveUp();
                retracted = true;
                Log.d("teleop", "intake retracted");
            }

            //outtake pivot
            if (gamepad2.y && !prevGamePadY) {
                outtakePivotAction.togglePosition();
                Log.d("teleop", "outtake pivoted");
            }

            //linear slides
            if (gamepad2.right_stick_y != 0) {
                outtakeSlideAction.setPower(-gamepad2.right_stick_y);
                Log.d("teleop", "linear slide moving...");
            } else {
                outtakeSlideAction.idle();
            }



            //Claw
            if (gamepad2.left_stick_y != 0 && gamepad2.left_stick_y != 0.1) {
                intakeLinkageAction.control(gamepad2.left_stick_y);
                Log.d("teleop", "intake linkage moving...");
            }
            //outtake claw
            if (gamepad2.right_bumper) {
                outtakeClawAction.open();
                Log.d("teleop", "outtake claw is open...");
            } else outtakeClawAction.close();

            //dpad left for door toggle
            if (gamepad2.dpad_left && !prevDpadLeft) {
                intakeDoorAction.togglePosition();
                Log.d("teleop", "intake door has been toggled");
            }
            //Pivot
            if (gamepad2.dpad_up && !prevDpadUp) {
                outtakeSlideAction.toggle();
                Log.d("teleop", "outtake slide toggled");
            }
            //linear slides down
            if (gamepad2.dpad_down) {
                outtakeSlideAction.moveToPosition(0);
                outtakePivotAction.setPosition(0.825);
                Log.d("teleop", "outtake slide moved down");
            }


            if (gamepad2.a && gamepad1.a) {
                telemetry.addData("", "yes");
                telemetry.update();
            }


            prevDpadUp = gamepad2.dpad_up;
            prevDpadLeft = gamepad2.dpad_left;
            prevDpadRight = gamepad2.dpad_right;
            prevGamePadA = gamepad2.a;
            prevGamePadB = gamepad2.b;
            prevGamePadX = gamepad2.x;
            prevGamePadY = gamepad2.y;
        }
    }
}

*/

package com.kalipsorobotics.intoTheDeep;

import static com.kalipsorobotics.math.CalculateTickPer.degToTicksIntakeLS;
import static com.kalipsorobotics.math.CalculateTickPer.mmToTicksLS;

import android.util.Log;

import com.kalipsorobotics.actions.AutoHangAction;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.drivetrain.AngleLockTeleOp;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.drivetrain.MoveWallTeleOp;
import com.kalipsorobotics.actions.hang.HangHookAction;
import com.kalipsorobotics.actions.intake.IntakeReadyAction;
import com.kalipsorobotics.actions.intake.IntakeTransferAction;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.MoveIntakeLSAction;
import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePigeonAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.ColorDetector;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.tensorflow.CameraCapture;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RedTeleop extends LinearOpMode {

    protected boolean isRed = true;
    protected boolean takeInYellow = true;

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, -180);
        DriveAction driveAction = new DriveAction(driveTrain);
        MoveWallTeleOp moveWallTeleOp = null;
        AngleLockTeleOp angleLockTeleOp = null;
        //Intake intake = new Intake(opModeUtilities);
        IntakeClaw intakeClaw = new IntakeClaw(opModeUtilities);
//        IntakeNoodleAction intakeNoodleAction = new IntakeNoodleAction(intake, 0, false);
//        IntakePivotAction intakePivotAction = new IntakePivotAction(intake);
//        IntakeDoorAction intakeDoorAction = new IntakeDoorAction(intake);
        Outtake outtake = new Outtake(opModeUtilities);
        OuttakePivotAction outtakePivotAction = new OuttakePivotAction(outtake);
        OuttakeSlideAction outtakeSlideAction = new OuttakeSlideAction(outtake);
        //OuttakeClawAction outtakeClawAction = new OuttakeClawAction(outtake);
        //OuttakePigeonAction outtakePigeonAction = new OuttakePigeonAction(outtake);
        ColorDetector colorDetector = new ColorDetector(opModeUtilities, hardwareMap);
        SpecimenHangReady specimenHangReady = null;
        SpecimenWallReady specimenWallReady = null;
        // Target should always be 0
        MoveOuttakeLSAction.setGlobalLinearSlideMaintainTicks(0);
        //MoveOuttakeLSAction maintainOuttakeGlobalPos = new MoveOuttakeLSAction(outtake, 0);
//        MoveIntakeLSAction.setGlobalLinearSlideMaintainTicks(0);
//        MoveIntakeLSAction maintainIntakeGlobalPos = new MoveIntakeLSAction(intake, 0);
        IntakeTransferAction intakeTransferAction = null;
        AutoHangAction autoHangAction = new AutoHangAction(outtake);
        CameraCapture cameraCapture = new CameraCapture();
        SampleIntakeReady sampleIntakeReady = null;
        IntakeTransferReady intakeTransferReady = null;

        boolean intakePosPressed = false;
        boolean retractPosPressed = false;
        boolean hangPressed = false;
        boolean rightTriggerPressed = false;
        boolean leftTriggerPressed = false;
        boolean prevGamepad2DpadDown = false; //gp2DPadPressed
        double targetHangPos = 0;

        double hangStartPosition = 0;



        double intakeLinkagePos = 0.95;
        double intakeBigSweepPos = 0.5;
        double intakeBigPivotPos = IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS;
        double intakeSmallPivotPos = IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS;
        double intakeSmallSweepPos = IntakeClaw.INTAKE_SMALL_SWEEP_RETRACT_POS;
        double intakeClawPos = 0.5;
        intakeClaw.getIntakeLinkageServo().setPosition(0.95);
        intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
        intakeClaw.getIntakeBigPivotServo().setPosition(intakeBigPivotPos);
        intakeClaw.getIntakeSmallPivotServo().setPosition(intakeSmallPivotPos);
        intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos );
        //intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);

        double outtakeClawPos = 0.5;

        outtake.getOuttakeClaw().setPosition(outtakeClawPos);

        double outtakePivotPos = 0.5;

        outtake.getOuttakePivotServo().setPosition(outtakePivotPos);


        //CHANGE ACCORDING TO ALLIANCE


        //intakeLinkageAction.retract();



//        intakePivotAction.moveUp();
//        intakeDoorAction.close();

        //outtakeClawAction.close();
        outtakePivotAction.moveWall();

//        outtake.getHangHook1().setPosition(Outtake.HOOK1_DOWN_POS);
//        outtake.getHangHook2().setPosition(Outtake.HOOK2_DOWN_POS);

        //Pigeon
        //outtakePigeonAction.setPosition(OuttakePigeonAction.OUTTAKE_PIGEON_IN_POS);

        waitForStart();

        while (opModeIsActive()) {
            //=========DRIVER 1==========
            driveAction.move(gamepad1);

            //RESET POS
            if (gamepad1.dpad_up) {
                driveTrain.resetWheelOdom();
                wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 180);
                Log.d("teleop_odo", "   reset odometry");
            }

            if (gamepad1.a) {
                Log.d("gampad1",
                        "gamepad1.y" + gamepad1.y + "   moveWallCondition  " + (moveWallTeleOp == null || moveWallTeleOp.getIsDone()));
                if (moveWallTeleOp == null || moveWallTeleOp.getIsDone()) {
                    moveWallTeleOp = new MoveWallTeleOp(driveTrain, wheelOdometry);
                    moveWallTeleOp.setName("moveWallTeleOp");
                }
            }

            if (gamepad1.y) {
                Log.d("gampad1",
                        "gamepad1.y" + gamepad1.y + "   angle lock condition  " + (angleLockTeleOp == null || angleLockTeleOp.getIsDone()));

                if (angleLockTeleOp == null || angleLockTeleOp.getIsDone()) {
                    angleLockTeleOp = new AngleLockTeleOp(driveTrain, wheelOdometry);
                    angleLockTeleOp.setName("angleLockTeleOp");
                }
            }

            if (isGamePadDriveJoystickZero()) {

                if (angleLockTeleOp != null) {
                    angleLockTeleOp.update();
                }

                if (moveWallTeleOp != null) {
                    moveWallTeleOp.update();
                }

            } else {  //Manual control override

                if (angleLockTeleOp != null) {
                    angleLockTeleOp.setIsDone(true);
                }
                if (moveWallTeleOp != null) {
                    moveWallTeleOp.setIsDone(true);
                }

            }

            //===================HANG==================

            if (gamepad1.b) {
                hangPressed = false;
                prevGamepad2DpadDown = false;

            }

            if(gamepad1.left_bumper && gamepad1.right_bumper) {
                hangPressed = true;
                autoHangAction = new AutoHangAction(outtake);
                //hang initiated
            }

            if (hangPressed) {
                if (autoHangAction.getIsDone()) {
                    hangPressed = false;
                } else {
                    autoHangAction.update();
                    //maintainOuttakeGlobalPos.setPConstant(1);
                    //update hanging if was pressed and not done
                }
            }

            if (!gamepad1.x && !gamepad1.dpad_down){
                MoveOuttakeLSAction.setOverridePower(0);
            } else if(gamepad1.x && !gamepad1.dpad_down) {
                MoveOuttakeLSAction.setOverridePower(-0.1);
//                MoveOuttakeLSAction.setGlobal(outtake.getLinearSlide1().getCurrentPosition() - CalculateTickPer.mmToTicksLS(5));
                MoveOuttakeLSAction.setGlobal(outtake.getLinearSlide1().getCurrentPosition());
                //MoveOuttakeLSAction.incrementGlobal( CalculateTickPer.mmToTicksLS(5) * -1);
            } else if (gamepad1.dpad_down && !gamepad1.x) {
                if (!prevGamepad2DpadDown) {
                    outtake.getHangHook1().setPosition(0.245);
                    outtake.getHangHook2().setPosition(0.65);
                    MoveOuttakeLSAction.setOverridePower(-1);
//                    MoveOuttakeLSAction.ERROR_TOLERANCE_TICKS = CalculateTickPer.mmToTicksLS(1);
                    targetHangPos = outtake.getLinearSlide1().getCurrentPosition() - CalculateTickPer.mmToTicksLS(47);
                    MoveOuttakeLSAction.setGlobal(targetHangPos);
                    prevGamepad2DpadDown = true;
                } else {
                    if (Math.abs(outtake.getLinearSlide1().getCurrentPosition() - targetHangPos) <= MoveOuttakeLSAction.ERROR_TOLERANCE_TICKS){
                        MoveOuttakeLSAction.setOverridePower(0);
                    }
                }
//                if(hangStartPosition - outtake.getLinearSlide1().getCurrentPosition() <= 44.45) {
//                    //MoveOuttakeLSAction.setOverridePower(-0.85);
//                    MoveOuttakeLSAction.setGlobal(outtake.getLinearSlide1().getCurrentPosition() - CalculateTickPer.mmToTicksLS(1));
//                    //MoveOuttakeLSAction.incrementGlobal( CalculateTickPer.mmToTicksLS(1) * -1);
//                }
            }

            if(gamepad1.right_trigger == 1) {
                rightTriggerPressed = true;
            }
            if(gamepad1.left_trigger == 1) {
                leftTriggerPressed = true;
            }

            if(rightTriggerPressed) {
                outtake.getHangHook1().setPosition(Outtake.HOOK1_HANG_POS);
                outtake.getHangHook2().setPosition(Outtake.HOOK2_HANG_POS);
                rightTriggerPressed = false;
            }

            if(leftTriggerPressed) {
                outtake.getHangHook1().setPosition(Outtake.HOOK1_DOWN_POS);
                outtake.getHangHook2().setPosition(Outtake.HOOK2_DOWN_POS);
                leftTriggerPressed = false;
            }

            if(gamepad1.back) {
                outtakeClawPos = Outtake.OUTTAKE_CLAW_CLOSE;
            } else if (gamepad1.start) {
                outtakeClawPos = Outtake.OUTTAKE_CLAW_OPEN;
            }

            outtake.getOuttakeClaw().setPosition(outtakeClawPos);

            if(gamepad1.right_stick_y < 0) {
                outtakePivotPos += 0.01;
            } else if(gamepad1.right_stick_y > 0) {
                outtakePivotPos -= 0.01;
            }
            outtake.getOuttakePivotServo().setPosition(outtakePivotPos);


            //===============DRIVER 2===============
            //================OUTTAKE================

            //outtake pivot
//            if (gamepad2.dpad_left) {
//                outtakePivotAction.moveOut();
//            } else if (gamepad2.dpad_right) {
//                outtakePivotAction.moveIn();
//            }

            //outtake manual LS
            if (gamepad2.left_stick_y != 0) {
                MoveOuttakeLSAction.incrementGlobal( CalculateTickPer.mmToTicksLS(15) * -gamepad2.left_stick_y);
            }
            else {
                MoveOuttakeLSAction.incrementGlobal(0);
            }

            if (gamepad2.left_stick_button) {
                //outtake slide
                CalculateTickPer.MIN_RANGE_LS_TICKS = -2500;
                //intake slide
                CalculateTickPer.MIN_RANGE_INTAKE_TICKS = CalculateTickPer.degToTicksIntakeLS(-100);
            }

            if (gamepad2.left_trigger > 0.99) {
                //outtake slide
                outtake.getLinearSlide1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.getLinearSlide1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                CalculateTickPer.MIN_RANGE_LS_TICKS = outtake.getLinearSlide1().getCurrentPosition() - mmToTicksLS(25);

//                //intake slide
//                intake.getLinkageMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                intake.getLinkageMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                CalculateTickPer.MIN_RANGE_INTAKE_TICKS =
//                        intake.getLinkageMotor().getCurrentPosition() - degToTicksIntakeLS(10);
            }



            //outtake hang ready
            if (gamepad2.dpad_up) {
                if (specimenHangReady == null || specimenHangReady.getIsDone()) {
                    specimenHangReady = new SpecimenHangReady(outtake);
                    specimenHangReady.setName("specimenHangReady");
                }
            }

            if (isGamePadOuttakeJoystickZero()) {

                if (specimenHangReady != null) {
                    specimenHangReady.update();
                }

            } else {

                if (specimenHangReady != null) {
                    specimenHangReady.setIsDone(true);
                }

            }


/*
            if (gamepad2.dpad_down) {
                specimenWallReady = new SpecimenWallReady(outtake);
                specimenWallReady.setName("specimenWallReady");
            }
            if (specimenWallReady != null) {
                specimenWallReady.update();
            }*/


            if (gamepad2.dpad_down) {
                outtakePivotAction.moveWall();
            }


//            if (gamepad2.left_bumper) {
//                outtakeClawAction.open();
//            } else {
//                outtakeClawAction.close();
//            }

            //=============INTAKE===============


//            if (gamepad2.right_trigger > 0.5) {
//                colorDetector.cycle(isRed, takeInYellow, intakeNoodleAction);
//            } else if (gamepad2.right_bumper) {
//                intakeNoodleAction.reverse();
//            } else {
//                intakeNoodleAction.stop();
//            }

//            //intake pivot
//            if (gamepad2.y && !prevGamePad2Y) {
//                intakePivotAction.moveUp();
//            } else if (gamepad2.a && !prevGamePad2A) {
//                intakePivotAction.moveDown();
//            }

            //intake door + transfer
//            if (gamepad2.b) {
//                if (!(isTransferRunning(intakeTransferAction))) {
//                    intakeTransferAction = new IntakeTransferAction(intake, outtake);
//                    intakeTransferAction.setName("intakeTransferAction");
//                }
//            } else {
//                if (gamepad2.x) {
//                    if (intakeTransferAction != null) {
//                        intakeTransferAction.setIsDone(true);
//                    }
//                    intakeDoorAction.open();
//
//                } else if (!(isTransferRunning(intakeTransferAction))) {
//                    intakeDoorAction.close();
//                }
//            }
//
//            if (isGamePadIntakeJoystickZero()) {
//                if (intakeTransferAction != null) {
//                    intakeTransferAction.update();
//                }
//            } else {
//                if (intakeTransferAction != null) {
//                    intakeTransferAction.setIsDone(true);
//                }
//            }

            if (gamepad2.x && !gamepad2.right_bumper) {
//                //close
//                intakeClawPos = IntakeClaw.INTAKE_CLAW_CLOSE;
            } else if (gamepad2.b && !gamepad2.right_bumper) {
//                //open
//                intakeClawPos = IntakeClaw.INTAKE_CLAW_OPEN;
            }

//            intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);

//            if (-gamepad2.right_stick_y != 0) {
//                MoveIntakeLSAction.incrementGlobal(degToTicksIntakeLS(5) * -gamepad2.right_stick_y);
//            }


            if (-gamepad2.right_stick_y != 0) {
//                intakeLinkagePos += 0.01 * gamepad2.right_stick_y;
//                if (intakeLinkagePos < 0.57) {
//                    intakeLinkagePos = 0.57;
//                }
//                if (intakeLinkagePos > 0.95) {
//                    intakeLinkagePos = 0.95;
//                }
//                intakeClaw.getIntakeLinkageServo().setPosition(intakeLinkagePos);
            }

//            if (-gamepad2.right_stick_y > 0.5) {
//                //extend
//                intakeLinkagePos += 0.001;
//                intakeClaw.getIntakeLinkageServo().setPosition(0.57);
//            } else if (-gamepad2.right_stick_y < -0.5) {
//                //retract
//                intakeLinkagePos -= 0.001;
//                intakeClaw.getIntakeLinkageServo().setPosition(0.95);
//
//            }

            if (-gamepad2.right_stick_x > 0.5) {
//                intakeBigSweepPos -= 0.001;
//                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
            } else if (-gamepad2.right_stick_x < -0.5) {
//                intakeBigSweepPos += 0.001;
//                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);

            }

//            if (gamepad2.y) {
//                intakeBigPivotPos += 0.005;
//            } else if (gamepad2.a) {
//                intakeBigPivotPos -= 0.005;
//            }

//            if (gamepad2.right_bumper && !gamepad2.b && !gamepad2.x) {
//                intakeSmallPivotPos += 0.005;
//            } else if (gamepad2.right_trigger > 0) {
//                intakeSmallPivotPos -= 0.005;
//            }

//            if (intakeSmallPivotPos > 1) {
//                intakeSmallPivotPos = 1;
//            } else if(intakeSmallPivotPos < 0) {
//                intakeSmallPivotPos = 0;
//            }

            if (gamepad2.right_bumper && gamepad2.b) {
//                intakeSmallSweepPos += 0.01;
//                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
            } else if (gamepad2.right_bumper && gamepad2.x) {
//                intakeSmallSweepPos -= 0.01;
//                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
            }

            if(gamepad2.start) {
//                intakeBigPivotPos = IntakeClaw.INTAKE_BIG_PIVOT_INTAKE_READY_POS;
//                intakeSmallPivotPos = IntakeClaw.INTAKE_SMALL_PIVOT_INTAKE_READY_POS;
//                intakeClawPos = IntakeClaw.INTAKE_CLAW_OPEN;
            }

            if(gamepad2.back) {
//                intakeBigPivotPos = IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS;
//                intakeSmallPivotPos = IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS;
//                intakeClawPos = IntakeClaw.INTAKE_CLAW_CLOSE;
            }

            if(gamepad2.a) {
//                intakeBigPivotPos = IntakeClaw.INTAKE_BIG_PIVOT_TRANSFER_POS;
//                intakeSmallPivotPos = IntakeClaw.INTAKE_SMALL_PIVOT_TRANSFER_POS;
//                intakeClawPos = IntakeClaw.INTAKE_CLAW_CLOSE;
            }

            if(gamepad2.y) {
//                intakeSmallPivotPos -= 0.005;
//                intakeClaw.getIntakeSmallPivotServo().setPosition(intakeSmallPivotPos);
//
//                intakeBigPivotPos += 0.005;
//                intakeClaw.getIntakeBigPivotServo().setPosition(intakeBigPivotPos);
            }



            if (gamepad2.dpad_left){
                if (sampleIntakeReady == null || sampleIntakeReady.getIsDone()){
                    sampleIntakeReady = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw, outtake);
                    sampleIntakeReady.update();
                }
            } else{
//                intakeClaw.getIntakeSmallPivotServo().setPosition(intakeSmallPivotPos);
//                intakeClaw.getIntakeBigPivotServo().setPosition(intakeBigPivotPos);
            }

            if (gamepad2.dpad_right){
                if (intakeTransferReady == null || intakeTransferReady.getIsDone()){
                    intakeTransferReady = new IntakeTransferReady(IntakeClaw.INTAKE_LINKAGE_IN_POS, intakeClaw, outtake);
                    intakeTransferReady.update();
                }
            }

            telemetry.addData("outtakeClawPos", outtakeClawPos);
            telemetry.addData("outtakePivotPos", outtakePivotPos);
            telemetry.addData("intakeClawPos", intakeClawPos);
            telemetry.addData("intakeSmallPivotPos", intakeSmallPivotPos);
            telemetry.addData("intakeBigPivotPos", intakeBigPivotPos);
            telemetry.addData("intakeSmallSweepPos", intakeSmallSweepPos);
            telemetry.addData("intakeLinkagePos", intakeLinkagePos);
            telemetry.update();

            // Capture pictures from webcam every 500 milliseconds if holding dpad right with gamepad 1
            //DO NOT USE ALL HOLD FOR TOO LONG it will take up to much space.
            if (gamepad1.dpad_right) {
                cameraCapture.capture();
            }

            wheelOdometry.updatePosition();
            //maintainOuttakeGlobalPos.update();
            //maintainIntakeGlobalPos.update();
/*


            //INTAKE
            //Noodle
            if (gamepad2.dpad_left && !prevDpadLeft) {
                takeInYellow = !takeInYellow;
            }
            if (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
                colorDetector.cycle(isRed, takeInYellow, intakeNoodleAction);
            } else if (gamepad2.left_bumper) {
                intakeNoodleAction.reverse();
            } else {
                intakeNoodleAction.stop();
            }

            //intake pivot
            if (gamepad2.x && !prevGamePadX) {
                intakePivotAction.togglePosition();
            }

            //dpad left for door toggle
            if (gamepad2.dpad_right && !prevDpadRight) {
                intakeDoorAction.togglePosition();
            }

            //manual control intake slide
            if (gamepad2.left_stick_y != 0 && gamepad2.left_stick_y != 0.1) {
                intakeLinkageAction.control(gamepad2.left_stick_y);
            }



            //===============Outtake================

            //outtake pivot
            if (gamepad2.y && !prevGamePad2Y) {
                outtakePivotAction.togglePosition();
            }
            //outtake linear slides manual
            if (gamepad2.right_stick_y != 0) {
                outtakeSlideAction.setPower(-gamepad2.right_stick_y);
            } else if ((specimenWallReady == null || specimenWallReady.checkDoneCondition()) && (specimenHangReady == null || specimenHangReady.checkDoneCondition())) {
                outtakeSlideAction.idle();
            }

            //outtake claw
            if (gamepad2.right_bumper) {
                outtakeClawAction.open();
            } else outtakeClawAction.close();

//            //outtake linear slide toggle
//            if (gamepad2.dpad_up && !prevDpadUp) {
//                outtakeSlideAction.toggle();
//            }

            if (gamepad2.dpad_up && !prevDpadUp) {
                Log.d("detected press", "dpad up");
                if (specimenHangReady  == null || specimenHangReady.checkDoneCondition()) {
                    specimenHangReady = new SpecimenHangReady(outtake);
                    teleopActionSet.addAction(specimenHangReady);
                }
            }
            if (specimenHangReady != null) {
                specimenHangReady.update();
            }

            if (gamepad2.dpad_down && !prevDpadDown) {
                Log.d("detected press", "dpad down");
                if (specimenWallReady == null || specimenWallReady.checkDoneCondition()) {
                    specimenWallReady = new SpecimenWallReady(outtake);
                }
            }
            if (specimenWallReady != null) {
                specimenWallReady.update();
            }




            if (gamepad2.a && gamepad1.a) {
                telemetry.addData("", "yes");
                telemetry.update();
            }


            prevDpadUp = gamepad2.dpad_up;
            prevDpadLeft = gamepad2.dpad_left;
            prevDpadRight = gamepad2.dpad_right;
            prevDpadDown = gamepad2.dpad_down;
            prevGamePad2A = gamepad2.a;
            prevGamePadB = gamepad2.b;
            prevGamePadX = gamepad2.x;
            prevGamePad2Y = gamepad2.y;

            teleopActionSet.updateCheckDone();


*/

        }
    }

    private boolean isLinearSlidesRunning(SpecimenHangReady specimenHangReady, SpecimenWallReady specimenWallReady) {
        if (specimenHangReady == null) {
            return false;
        }
        if (specimenWallReady == null) {
            return false;
        }

        if (!specimenHangReady.checkDoneCondition()) {
            return true;
        }

        if (!specimenWallReady.checkDoneCondition()) {
            return true;
        }

        return false;
    }

    private boolean isTransferRunning(IntakeTransferAction intakeTransferAction) {
        return intakeTransferAction != null && !intakeTransferAction.getIsDone();
    }

    private boolean isGamePadDriveJoystickZero() {
        boolean isGamePadDriveJoystickZero =
                ((gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) && (gamepad1.right_stick_x == 0));
        return isGamePadDriveJoystickZero;
    }

    private boolean isGamePadOuttakeJoystickZero() {
        boolean isGamePadOuttakeJoystickZero =
                ((gamepad2.left_stick_y == 0));
    return isGamePadOuttakeJoystickZero;
    }

    private boolean isGamePadIntakeJoystickZero() {
        boolean isGamePadIntakeJoystickZero =
                ((gamepad2.right_stick_y == 0));
        return isGamePadIntakeJoystickZero;
    }

}


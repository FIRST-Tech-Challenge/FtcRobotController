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

import static com.kalipsorobotics.math.CalculateTickPer.mmToTicksLS;

import android.util.Log;

import com.kalipsorobotics.actions.AutoHangAction;
import com.kalipsorobotics.actions.TransferAction;
import com.kalipsorobotics.actions.drivetrain.AngleLockTeleOp;
import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.drivetrain.MoveWallTeleOp;
import com.kalipsorobotics.actions.intake.IntakeTransferAction;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.BasketReadyAction;
import com.kalipsorobotics.actions.outtake.MoveOuttakeLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.tensorflow.CameraCapture;
import com.kalipsorobotics.utilities.KGamePad;
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
        IntakeClaw intakeClaw = new IntakeClaw(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        OuttakePivotAction outtakePivotAction = new OuttakePivotAction(outtake);
        OuttakeSlideAction outtakeSlideAction = new OuttakeSlideAction(outtake);
        SpecimenHangReady specimenHangReady = null;
        // Target should always be 0
        MoveOuttakeLSAction moveOuttakeLSAction = new MoveOuttakeLSAction(outtake, 0);
        MoveOuttakeLSAction.setGlobalLinearSlideMaintainTicks(0);
        AutoHangAction autoHangAction = new AutoHangAction(outtake);
        CameraCapture cameraCapture = new CameraCapture();
        SampleIntakeReady sampleIntakeReady = null;
        SampleIntakeAction sampleIntakeAction = null;
        IntakeTransferReady intakeTransferReady = null;
        TransferAction transferAction = null;
        BasketReadyAction basketReadyAction = null;
        OuttakeTransferReady outtakeTransferReady = null;
        SpecimenHang specimenHang = null;
        KGamePad kGamePad2 = new KGamePad(gamepad2);
        KGamePad kGamePad1 = new KGamePad(gamepad1);

        double intakeLinkagePos = IntakeClaw.INTAKE_LINKAGE_IN_POS;
        double intakeBigSweepPos = 0.5;
        double intakeBigPivotPos = IntakeClaw.INTAKE_BIG_PIVOT_RETRACT_POS;
        double intakeSmallPivotPos = IntakeClaw.INTAKE_SMALL_PIVOT_RETRACT_POS;
        double intakeSmallSweepPos = IntakeClaw.INTAKE_SMALL_SWEEP_RETRACT_POS;
        double intakeClawPos = IntakeClaw.INTAKE_CLAW_OPEN;

        double outtakeClawPos = Outtake.OUTTAKE_CLAW_OPEN;
        double outtakePivotPos = 0;


        //CHANGE ACCORDING TO ALLIANCE

        outtakePivotAction.moveWall();

        // GAMEPAD 1
        boolean resetWheelOdomPressed;
        boolean angleLockPressed;
        boolean hangPressed = false;
        boolean hangHooksUpPressed;
        boolean hangHooksDownPressed;
        boolean prevGamepad2DpadDown = false; //gp2DPadPressed
        double targetHangPos = 0;

        // GAMEPAD 2
        double outtakeLSStickValue;
        double outtakePivotStickValue;
        double intakeStickValue;
        double sweepStickValue;
        boolean outtakeOverrideOn;
        boolean specimenHangReadyPressed;
        boolean intakeClawOpenClose;
        boolean intakeClawOpenCloseWasPressed = false;
        boolean intakeOverrideOn;
        boolean intakeReadyPressed;
        boolean intakeTransferReadyPressed;
        boolean transferPressed;
        boolean intakePressed;
        boolean outtakeTransferReadyPressed;
        boolean outtakeClawOpenClose;
        boolean outtakeClawOpenCloseWasPressed = false;
        boolean basketReadyPressed;
        boolean specimenHangPressed;
        boolean specimenReadyPressed;

        intakeClaw.init();
        outtake.init();

        waitForStart();

        while (opModeIsActive()) {
            // GAMEPAD 1 ASSIGNMENTS ==============================================
            resetWheelOdomPressed = kGamePad1.isToggleDpadUp();
            angleLockPressed = kGamePad1.isToggleButtonY();
            hangHooksUpPressed = kGamePad1.isRightTriggerPressed();

            hangHooksUpPressed = kGamePad1.isRightTriggerPressed();
            hangHooksDownPressed = kGamePad1.isLeftTriggerPressed();

            // GAMEPAD 2 ASSIGNMENTS ==============================================
            outtakeLSStickValue = gamepad2.right_stick_y;
            intakeStickValue = gamepad2.left_stick_y;
            sweepStickValue = gamepad2.left_stick_x;
            outtakeOverrideOn = kGamePad2.isRightTriggerPressed();
            specimenHangReadyPressed = kGamePad2.isToggleButtonB();
            intakeClawOpenClose = kGamePad2.isToggleLeftBumper();
            intakeOverrideOn = kGamePad2.isLeftTriggerPressed();
            intakeReadyPressed = kGamePad2.isToggleDpadLeft();
            intakeTransferReadyPressed = kGamePad2.isToggleDpadRight();
            transferPressed = kGamePad2.isToggleDpadUp();
            intakePressed = kGamePad2.isToggleDpadDown();
            outtakeTransferReadyPressed = kGamePad2.isToggleButtonX();
            outtakeClawOpenClose = kGamePad2.isToggleRightBumper();
            outtakePivotStickValue = gamepad2.right_stick_x;
            basketReadyPressed = kGamePad2.isToggleButtonY();
            specimenHangPressed = kGamePad2.isToggleButtonA();
            specimenReadyPressed = kGamePad2.isToggleButtonB();

            //RESET POSITIONS TO CURRENT
            intakeLinkagePos = intakeClaw.getIntakeLinkageServo().getServo().getPosition();
            intakeBigSweepPos = intakeClaw.getIntakeBigSweepServo().getServo().getPosition();
            intakeBigPivotPos = intakeClaw.getIntakeBigPivotServo().getServo().getPosition();
            intakeSmallPivotPos = intakeClaw.getIntakeSmallPivotServo().getServo().getPosition();
            intakeSmallSweepPos = intakeClaw.getIntakeSmallSweepServo().getServo().getPosition();
            outtakePivotPos = outtake.getOuttakePivotServo().getServo().getPosition();

            //=========DRIVER 1==========
            driveAction.move(gamepad1);

            //RESET POS
            if (resetWheelOdomPressed) {
                driveTrain.resetWheelOdom();
                wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 180);
                Log.d("teleop_odo", "   reset odometry");
            }

            if (angleLockPressed) {
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

            //HANG
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

            if(hangHooksUpPressed) {
                outtake.getHangHook1().setPosition(Outtake.HOOK1_HANG_POS);
                outtake.getHangHook2().setPosition(Outtake.HOOK2_HANG_POS);
            }

            if(hangHooksDownPressed) {
                outtake.getHangHook1().setPosition(Outtake.HOOK1_DOWN_POS);
                outtake.getHangHook2().setPosition(Outtake.HOOK2_DOWN_POS);
            }



            //===============DRIVER 2===============

            //INTAKE

            if(intakeClawOpenClose) {
                intakeClawOpenCloseWasPressed = true;
            }

            if(!intakeClawOpenClose) {
                if(intakeClawOpenCloseWasPressed) {
                    if(intakeClawPos == IntakeClaw.INTAKE_CLAW_CLOSE) {
                        intakeClawPos = IntakeClaw.INTAKE_CLAW_OPEN;
                        intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);
                    } else if (intakeClawPos == IntakeClaw.INTAKE_CLAW_OPEN){
                        intakeClawPos = IntakeClaw.INTAKE_CLAW_CLOSE;
                        intakeClaw.getIntakeClawServo().setPosition(intakeClawPos);
                    }
                }

                intakeClawOpenCloseWasPressed = false;
            }

            if (-intakeStickValue != 0) {
                intakeLinkagePos += 0.01 * intakeStickValue;
                if (intakeLinkagePos < 0.57) {
                    intakeLinkagePos = 0.57;
                }
                if (intakeLinkagePos > 0.95) {
                    intakeLinkagePos = 0.95;
                }
                intakeClaw.getIntakeLinkageServo().setPosition(intakeLinkagePos);
            }

            if (-sweepStickValue > 0.5 && intakeOverrideOn) {
                intakeBigSweepPos -= 0.005;
                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
            } else if (-sweepStickValue < -0.5 && intakeOverrideOn) {
                intakeBigSweepPos += 0.005;
                intakeClaw.getIntakeBigSweepServo().setPosition(intakeBigSweepPos);
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

            if ((-sweepStickValue > 0.5) && !intakeOverrideOn) {
                intakeSmallSweepPos += 0.005;
                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
                Log.d("sweeping",  "" + intakeSmallSweepPos);
            } else if ((-sweepStickValue < -0.5) && !intakeOverrideOn) {
                intakeSmallSweepPos -= 0.005;
                intakeClaw.getIntakeSmallSweepServo().setPosition(intakeSmallSweepPos);
                Log.d("sweeping",  "" + intakeSmallSweepPos);
            }

            if(intakeReadyPressed) {
                if(sampleIntakeReady != null) {
                    Log.d("teleop", "sample intake ready done: " + sampleIntakeReady.getIsDone());
                } else {
                    Log.d("teleop", "sample intake ready null");
                }

                if (sampleIntakeReady == null || sampleIntakeReady.getIsDone()){
                    sampleIntakeReady = new SampleIntakeReady(IntakeClaw.INTAKE_LINKAGE_EXTEND_POS, intakeClaw);
                    Log.d("teleop", "made new sample intake  ready");
                    sampleIntakeReady.setName("sampleIntakeReady");
                }

            }
            if (sampleIntakeReady != null){
                Log.d("teleop", "running sample intake  ready");
                sampleIntakeReady.updateCheckDone();
            }

            if(intakePressed) {
                if (sampleIntakeAction == null || sampleIntakeAction.getIsDone()){
                    sampleIntakeAction = new SampleIntakeAction(intakeClaw);
                    sampleIntakeAction.setName("sampleIntakeAction");
                }

            }
            if (sampleIntakeAction != null){
                sampleIntakeAction.updateCheckDone();
            }

            if (intakeTransferReadyPressed){
                if(intakeTransferReady != null) {
                    Log.d("teleop", "intake transfer ready done: " + intakeTransferReady.getIsDone());
                } else {
                    Log.d("teleop", "intake transfer ready null");
                }

                if (intakeTransferReady == null || intakeTransferReady.getIsDone()){
                    intakeTransferReady = new IntakeTransferReady(intakeClaw);
                    Log.d("teleop", "made new intake transfer ready");
                    intakeTransferReady.setName("intakeTransferReady");
                }

            }
            if (intakeTransferReady != null){
                intakeTransferReady.updateCheckDone();
            }

            if(transferPressed) {
                if (transferAction == null || transferAction.getIsDone()){
                    transferAction = new TransferAction(intakeClaw, outtake);
                    transferAction.setName("transferAction");
                }

            }
            if (transferAction != null){
                transferAction.updateCheckDone();
            }

            //OUTTAKE

            //outtake manual LS
            if (outtakeLSStickValue != 0) {
                MoveOuttakeLSAction.incrementGlobal( CalculateTickPer.mmToTicksLS(15) * -outtakeLSStickValue);
            } else {
                MoveOuttakeLSAction.incrementGlobal(0);
            }

//            if (gamepad2.left_stick_button) {
//                //outtake slide
//                CalculateTickPer.MIN_RANGE_LS_TICKS = -2500;
//                //intake slide
//                CalculateTickPer.MIN_RANGE_INTAKE_TICKS = CalculateTickPer.degToTicksIntakeLS(-100);
//            }

            if (outtakeOverrideOn) {
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

            if (outtakeTransferReadyPressed) {
                if (outtakeTransferReady == null || outtakeTransferReady.getIsDone()) {
                    outtakeTransferReady = new OuttakeTransferReady(outtake);
                    outtakeTransferReady.setName("outtakeTransferReady");
                }

            }
            if (outtakeTransferReady != null){
                outtakeTransferReady.updateCheckDone();
            }

            //outtake hang ready
            if (specimenHangReadyPressed) {
                if (specimenHangReady == null || specimenHangReady.getIsDone()) {
                    specimenHangReady = new SpecimenHangReady(outtake);
                    specimenHangReady.setName("specimenHangReady");
                }

            }

            if (specimenHangReady != null){
                specimenHangReady.updateCheckDone();
            }

            if(specimenHangPressed) {
                if(specimenHang == null || specimenHang.getIsDone()){
                   specimenHang = new SpecimenHang(outtake);
                   specimenHang.setName("specimanHang");
                }
            }

            if(specimenHang != null){
                specimenHang.updateCheckDone();
            }

            if(basketReadyPressed) {
                if (basketReadyAction == null || basketReadyAction.getIsDone()){
                    basketReadyAction = new BasketReadyAction(outtake);
                    basketReadyAction.setName("basketReadyAction");
                }

            }
            if (basketReadyAction != null){
                basketReadyAction.updateCheckDone();
            }

            if (isGamePadOuttakeJoystickZero()) {
                if (specimenHangReady != null) {
                    specimenHangReady.updateCheckDone();
                }
            } else {
                if (specimenHangReady != null) {
                    specimenHangReady.setIsDone(true);
                }
            }

            if(outtakeClawOpenClose) {
                outtakeClawOpenCloseWasPressed = true;
            }

            if(!outtakeClawOpenClose) {
                if(outtakeClawOpenCloseWasPressed){
                    if(outtakeClawPos == Outtake.OUTTAKE_CLAW_CLOSE) {
                        outtakeClawPos = Outtake.OUTTAKE_CLAW_OPEN;
                        outtake.getOuttakeClaw().setPosition(outtakeClawPos);
                    } else if (outtakeClawPos == Outtake.OUTTAKE_CLAW_OPEN){
                        outtakeClawPos = Outtake.OUTTAKE_CLAW_CLOSE;
                        outtake.getOuttakeClaw().setPosition(outtakeClawPos);
                    }

                    outtakeClawOpenCloseWasPressed = false;
                }
            }

            if (outtakePivotStickValue != 0) {
                outtakePivotPos += 0.01 * outtakePivotStickValue;
                outtake.getOuttakePivotServo().setPosition(outtakePivotPos);
                Log.d("outtake", " this is outtake pivot" + outtakePivotPos);
            }


            // Capture pictures from webcam every 500 milliseconds if holding dpad right with gamepad 1
            //DO NOT USE ALL HOLD FOR TOO LONG it will take up to much space.
            if (gamepad1.dpad_right) {
                cameraCapture.capture();
            }
            moveOuttakeLSAction.update();
            wheelOdometry.updatePosition();

            telemetry.addData("odometry: ", wheelOdometry.getCurrentPosition().toString());
            telemetry.addData("big sweep pos: ", intakeBigSweepPos);
            telemetry.addData("small sweep pos: ", intakeSmallSweepPos);
            telemetry.update();

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


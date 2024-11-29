package com.kalipsorobotics.intoTheDeep;

import android.os.SystemClock;

import com.kalipsorobotics.actions.DriveAction;
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
import com.kalipsorobotics.utilities.KColor;
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
        ColorDetector colorDetector = new ColorDetector(opModeUtilities);


        boolean prevGamePadY = false;
        boolean prevGamePadX = false;
        boolean prevGamePadB = false;
        boolean prevGamePadA = false;
        boolean prevDpadLeft = false;
        boolean prevDpadUp = false;
        boolean retracted = true;

        //CHANGE ACCORDING TO ALLIANCE

        boolean isRed = true;

        intakeLinkageAction.retract();
        SystemClock.sleep(500);
        intakePivotAction.moveUp();
        intakeDoorAction.close();

        outtakeClawAction.close();
        outtakePivotAction.moveIn();

        //Pigeon
        outtakePigeonAction.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            //Drive
            driveAction.move(gamepad1);
            boolean takeInYellow = gamepad2.dpad_right;
            //INTAKE
            //Noodles
            //HOLD DPAD RIGHT TO TAKE IN YELLOW
            if (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
                colorDetector.cycle(isRed, takeInYellow, intakeNoodleAction);
            } else if (gamepad2.left_bumper) {
                intakeNoodleAction.reverse();
            } else {
                intakeNoodleAction.stop();
            }


            //Pivot TODO make generic toggle
            if (gamepad2.x && !prevGamePadX) {
                intakePivotAction.togglePosition();
            }

            //Door
            //TODO alan you should make this whole chunky thing maybe its own teleop ACTION
            if (gamepad2.b && !prevGamePadB) {
                transferSequence.sequence();
                retracted = true;
            }
            //save for later
            //Linkage
            //TODO make sure its not blocking w/ the sleeps
            if (gamepad2.a && !prevGamePadA) {
                if (retracted) {
                    intakeSequence.extend();
                    retracted = false;
                } else {
                    intakeSequence.extend();
                    retracted = true;
                }
            }

            if (gamepad2.y && !prevGamePadY) {
                outtakePivotAction.togglePosition();
            }


            if (gamepad2.right_stick_y != 0) {
                outtakeSlideAction.setPower(gamepad2.right_stick_y);
            } else {
                outtakeSlideAction.idle();
            }



            //Claw
            if (gamepad2.left_stick_y != 0 && gamepad2.left_stick_y != 0.1) {
                intakeLinkageAction.control(gamepad2.left_stick_y);
            }

            if (gamepad2.right_bumper) {
                outtakeClawAction.open();
            } else outtakeClawAction.close();

            //dpad left for door toggle
            if (gamepad2.dpad_left && !prevDpadLeft) {
                intakeDoorAction.togglePosition();
            }
            //Pivot
            if (gamepad2.dpad_up && !prevDpadUp) {
                outtakeSlideAction.toggle();

            }
            if (gamepad2.dpad_down) {
                outtakeSlideAction.moveToPosition(0);
                outtakePivotAction.setPosition(0.825);
            }

            if (gamepad2.a && gamepad1.a) {
                telemetry.addData("", "yes");
            }


            prevDpadUp = gamepad2.dpad_up;
            prevDpadLeft = gamepad2.dpad_left;
            prevGamePadA = gamepad2.a;
            prevGamePadB = gamepad2.b;
            prevGamePadX = gamepad2.x;
            prevGamePadY = gamepad2.y;
        }
    }
}

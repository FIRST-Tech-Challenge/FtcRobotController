package com.kalipsorobotics.intoTheDeep;

import android.os.SystemClock;
import android.util.Log;

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
            //drive logging for sparkfun position
            if (gamepad1.a) {
                Log.d("teleop", "x: " + sparkfunOdometry.updatePosition().getX() + "y: " +
                        sparkfunOdometry.updatePosition().getY());
            }

            //Door
            //TODO
            if (gamepad2.b && !prevGamePadB) {
                transferSequence.sequence();
                retracted = true;
                Log.d("teleop", "transfering...");
            }
            //save for later
            //Linkage
            //TODO make sure its not blocking w/ the sleeps

            if (gamepad2.a && !prevGamePadA || !intakeSequence.checkDone(500)) {
                if (retracted) {
                    intakeLinkageAction.extend();
                    intakeSequence.setUpCheckDone();
                    if (intakeSequence.checkDone(500)) {
                        intakePivotAction.moveDown();
                    }
                    retracted = false;
                    Log.d("teleop", "intake extended");
                } else {
                    intakeLinkageAction.retract();
                    intakePivotAction.moveUp();
                    retracted = true;
                    Log.d("teleop", "intake retracted");
                }
            }

            //outtake pivot
            if (gamepad2.y && !prevGamePadY) {
                outtakePivotAction.togglePosition();
                Log.d("teleop", "outtake pivoted");
            }

            //linear slides
            if (gamepad2.right_stick_y != 0) {
                outtakeSlideAction.setPower(gamepad2.right_stick_y);
                Log.d("teleop", "linear slide moving...");
            } else {
                outtakeSlideAction.stop();
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

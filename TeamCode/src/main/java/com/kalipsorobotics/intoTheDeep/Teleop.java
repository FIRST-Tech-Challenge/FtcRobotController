package com.kalipsorobotics.intoTheDeep;

import android.os.SystemClock;

import com.kalipsorobotics.actions.Action;
import com.kalipsorobotics.actions.DriveAction;
import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeLinkageAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.actions.outtake.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.OuttakePigeonAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.OuttakeSlideAction;
import com.kalipsorobotics.localization.SparkfunOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
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
        OuttakeSlideAction outtakeSlideAction = new OuttakeSlideAction(outtake, outtakePivotAction);
        OuttakeClawAction outtakeClawAction = new OuttakeClawAction(outtake);
        OuttakePigeonAction outtakePigeonAction = new OuttakePigeonAction(outtake);
        Action action = new Action() {
            @Override
            public boolean checkDoneCondition() {
                return false;
            }

            @Override
            public void update() {

            }
        };

        boolean prevGamePadY = false;
        boolean prevGamePadX = false;
        boolean prevGamePadB = false;
        boolean prevGamePadA = false;
        boolean prevDpadLeft = false;
        boolean prevDpadUp = false;
        boolean retracted = true;


        intakeLinkageAction.retract();
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
            //Noodles
            if (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
                intakeNoodleAction.run();
            } else if (gamepad2.left_bumper) {
                intakeNoodleAction.reverse();
            } else {
                intakeNoodleAction.stop();
            }
    

            //Pivot TODO make generic toggle
            if (gamepad2.x && !prevGamePadX) {
                intakePivotAction.togglePosition();
            }
            prevGamePadX = gamepad2.x;

            //Door
            if (gamepad2.b && !prevGamePadB) {
                retracted = true;
            }
            prevGamePadB = gamepad2.b;
            //save for later
            //Linkage
            if (gamepad2.a && !prevGamePadA) {
                if (retracted) {
                    intakeLinkageAction.extend();
                    SystemClock.sleep(1700);
                    intakePivotAction.moveDown();
                    retracted = false;
                } else {
                    intakeLinkageAction.retract();
                    intakePivotAction.togglePosition();
                    retracted = true;
                }
            }
            prevGamePadA = gamepad2.a;

            //LinearSlide toggle
      //      if (gamepad2.y && !prevGamePadY) {
       //         while (!outtakeSlideAction.toggle()) {
      //              outtakeSlideAction.toggle();
      //          }
      //      }
       //     prevGamePadY = gamepad2.y;
            //OUTTAKE
            //LinearSlide


//            if (-gamepad2.right_stick_y > 0.1) {
//                outtakeSlideAction.setPower(-gamepad2.right_stick_y);
//            } else if (-gamepad2.right_stick_y < -0.1) {
//                outtakeSlideAction.setPower(-gamepad2.right_stick_y);
//            } else {
//                outtakeSlideAction.idle();
//            }


            //Claw
            if (gamepad2.right_bumper) {
                outtakeClawAction.open();
            } else outtakeClawAction.close();

            //dpad left for door toggle
            if (gamepad2.dpad_left && !prevDpadLeft) {
                intakeDoorAction.togglePosition();
            }
            prevDpadLeft = gamepad2.dpad_left;


            //Pivot
            if (gamepad2.dpad_up && !prevDpadUp) {
                outtakeSlideAction.Toggle();

            }
            if (gamepad2.dpad_down) {
                outtakeSlideAction.down();
                outtakePivotAction.setPosition(0.925);
            }
            prevDpadUp = gamepad2.dpad_up;

            if (gamepad2.a && gamepad1.a) {
                telemetry.addData("", "yes");
            }
        }
    }
}

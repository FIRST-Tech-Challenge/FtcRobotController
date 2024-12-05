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

import com.kalipsorobotics.actions.drivetrain.DriveAction;
import com.kalipsorobotics.actions.intake.MoveIntakeLSAction;
import com.kalipsorobotics.actions.intake.IntakeDoorAction;
import com.kalipsorobotics.actions.intake.IntakeNoodleAction;
import com.kalipsorobotics.actions.intake.IntakePivotAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePigeonAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.ColorDetector;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        DriveAction driveAction = new DriveAction(driveTrain);
        Intake intake = new Intake(opModeUtilities);
        IntakeNoodleAction intakeNoodleAction = new IntakeNoodleAction(intake, 0, false);
        IntakePivotAction intakePivotAction = new IntakePivotAction(intake);
        IntakeDoorAction intakeDoorAction = new IntakeDoorAction(intake);
        //IntakeLinkageAction intakeLinkageAction = new IntakeLinkageAction(intake);
        Outtake outtake = new Outtake(opModeUtilities);
        OuttakePivotAction outtakePivotAction = new OuttakePivotAction(outtake);
        OuttakeSlideAction outtakeSlideAction = new OuttakeSlideAction(outtake);
        OuttakeClawAction outtakeClawAction = new OuttakeClawAction(outtake);
        OuttakePigeonAction outtakePigeonAction = new OuttakePigeonAction(outtake);
        ColorDetector colorDetector = new ColorDetector(opModeUtilities, hardwareMap);
        SpecimenHangReady specimenHangReady = null;
        SpecimenWallReady specimenWallReady = null;
        // Targer should always be 0
        MoveLSAction.setGlobalLinearSlideMaintainTicks(0);
        MoveLSAction maintainOuttakeGlobalPos = new MoveLSAction(outtake, 0);
        MoveIntakeLSAction.setGlobalLinearSlideMaintainTicks(0);
        MoveIntakeLSAction maintainIntakeGlobalPos = new MoveIntakeLSAction(intake, 0);
//        MoveLSAction.globalLinearSlideMaintainTicks);


        boolean prevGamePadY = false;
        boolean prevGamePadX = false;
        boolean prevGamePadB = false;
        boolean prevGamePadA = false;
        boolean prevDpadLeft = false;
        boolean prevDpadUp = false;
        boolean prevDpadRight = false;
        boolean prevDpadDown = false;
        boolean retracted = true;

        //CHANGE ACCORDING TO ALLIANCE

        boolean isRed = true;
        boolean takeInYellow = false;

        //intakeLinkageAction.retract();



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

            //================OUTTAKE================

            //outtake pivot
            if (gamepad2.dpad_left) {
                outtakePivotAction.moveOut();
            } else if (gamepad2.dpad_right) {
                outtakePivotAction.moveIn();
            }

            //outtake manual LS
            if (gamepad2.left_stick_y != 0) {
                MoveLSAction.incrementGlobal( CalculateTickPer.mmToTicksLS(15) * -gamepad2.left_stick_y);
            }


            //outtake hang ready
            if (gamepad2.dpad_up) {
                specimenHangReady = new SpecimenHangReady(outtake);
                specimenHangReady.setName("specimenHangReady");
            }
            if (specimenHangReady != null) {
                specimenHangReady.update();
            }

//            if (gamepad2.dpad_down) {
//                specimenWallReady = new SpecimenWallReady(outtake);
//                specimenWallReady.setName("specimenWallReady");
//            }
//            if (specimenWallReady != null) {
//                specimenWallReady.update();
//            }


            if (gamepad2.dpad_down) {
                outtakePivotAction.moveWall();
            }


            if (gamepad2.left_bumper) {
                outtakeClawAction.open();
            } else {
                outtakeClawAction.close();
            }

            //=============INTAKE===============

            if (gamepad2.right_trigger > 0.5) {
                colorDetector.cycle(isRed, takeInYellow, intakeNoodleAction);
            } else if (gamepad2.right_bumper) {
                intakeNoodleAction.reverse();
            } else {
                intakeNoodleAction.stop();
            }

            //intake pivot
            if (gamepad2.y && !prevGamePadY) {
                intakePivotAction.moveUp();
            } else if (gamepad2.a && !prevGamePadA) {
                intakePivotAction.moveDown();
            }

            //intake door
            if (gamepad2.x) {
                intakeDoorAction.open();
            } else {
                intakeDoorAction.close();
            }

            //intake slides manual
            /*if (-gamepad2.right_stick_y != 0) {
                intakeLinkageAction.control(gamepad2.right_stick_y);
            }*/
            if (-gamepad2.right_stick_y != 0) {
                MoveIntakeLSAction.incrementGlobal(degToTicksIntakeLS(5) * -gamepad2.right_stick_y);
            }

            maintainOuttakeGlobalPos.update();
            maintainIntakeGlobalPos.update();
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
            if (gamepad2.y && !prevGamePadY) {
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
            prevGamePadA = gamepad2.a;
            prevGamePadB = gamepad2.b;
            prevGamePadX = gamepad2.x;
            prevGamePadY = gamepad2.y;

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

}


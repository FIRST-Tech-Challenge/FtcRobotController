package org.firstinspires.ftc.teamcode.bots;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class FSMBot extends RollerIntakeBot{
    private RollerIntakeBot robot = new RollerIntakeBot(this.opMode);

    private ElapsedTime timeSince = new ElapsedTime();
    public ElapsedTime subRetractTimer = new ElapsedTime();
    public ElapsedTime subIntakeTimer = new ElapsedTime();

    public ElapsedTime pivotUpTimer = new ElapsedTime();

    public ElapsedTime slidesUpTimer = new ElapsedTime();

    public boolean isSlideUp = false;

    public boolean isArmUp = false;

    public ElapsedTime slidesDownTimer = new ElapsedTime();
    public ElapsedTime pivotDownTimer = new ElapsedTime();

    public ElapsedTime outtakeTimer = new ElapsedTime();

    public ElapsedTime hangTimer = new ElapsedTime();



    public enum gameState {
        INIT_READY,
        SUBMERSIBLE_INTAKE_1,
        SUBMERSIBLE_INTAKE_2,
        WALL_INTAKE_1,
        WALL_INTAKE_2,
        SPECIMEN_SCORING_HIGH,
        SPECIMEN_SCORING_LOW,
        SAMPLE_SCORING_HIGH_1,
        SAMPLE_SCORING_HIGH_2,

        SAMPLE_SCORING_HIGH_3,

        SAMPLE_SCORING_LOW_1,
        SAMPLE_SCORING_LOW_2,
        ARM_DOWN,
        PIVOT_DOWN,
        PRE_DRIVE,
        DRIVE,
        HANG_UP,
        HANG_DOWN,
        DEFAULT;
    }

    public boolean auto;
    public boolean intakeDown = false;

    public int intakeSlideMinimum = 180;

    public double normalIntakePitchTarget = 0;

    public double normalIntakeRollTarget = 0;

    public static double groundIntakePitchTarget = -55;
    public static double groundIntakeRollTarget = 0;

    public static double wallIntakePitchTarget;
    public static double wallIntakeRollTarget;

    public double specimenHighOuttakeRollTarget;

    public double sampleOuttakePitchTarget;

    public double sampleOuttakeRollTarget;

    public boolean slideUp = false;
    public boolean hingeDown = false;
    public boolean wallIntakeDone = false;
    public boolean specimenScored = false;



    public gameState currentState = gameState.INIT_READY;

    public FSMBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwmap){
        super.init(ahwmap);
        robot.isAuto = false;
        robot.init(ahwmap);
        timeSince.reset();
        auto = isAuto;

    }

    public void waitForState(gameState goalState) {
        telemetry.addData("Waiting for state ", goalState.toString());
        while (opMode.opModeIsActive() && currentState != goalState) {
            telemetry.addData("Current state: ", currentState);
            telemetry.update();
            sleep(50);
        }
        telemetry.addData("state reached: ", goalState);
        telemetry.update();
        sleep(100);
    }

    protected void onTick() {
        super.onTick();
        telemetry.addData("state: ", currentState);
        telemetry.update();
        switch (currentState) {
            case INIT_READY:
                pitchTo(0); //sets differential wrist positions
                rollTo(0);
                currentState = gameState.PRE_DRIVE;
                break;
            case SUBMERSIBLE_INTAKE_1:
                robot.slideRunToPosition(intakeSlideMinimum);
                robot.pivotRunToPosition(0);
                //
                if(subIntakeTimer.milliseconds() > 140){
                    currentState = gameState.SUBMERSIBLE_INTAKE_2;
                }
                subRetractTimer.reset();
                break;
            case SUBMERSIBLE_INTAKE_2:
                robot.pitchTo(groundIntakePitchTarget);
                robot.rollTo(groundIntakeRollTarget);
                robot.intake(true);
//                robot.slideControl(gamepad1.dpad_right, gamepad1.dpad_left);
//                retractSubIntake(gamepad1.b);
                robot.pivotRunToPosition(0);
                //
//                currentState = gameState.ARM_DOWN;
                break;
            case WALL_INTAKE_1:
                //pivot up
                robot.pitch(wallIntakePitchTarget);
                robot.rollTo(wallIntakeRollTarget);
                robot.pivotRunToPosition(0);
                //face the right direction
                currentState = gameState.WALL_INTAKE_2;
                break;
            case WALL_INTAKE_2:
                //arm up
                robot.slideRunToPosition(100);
                //wait for drive
                if (wallIntakeDone) {
                    currentState = gameState.SPECIMEN_SCORING_HIGH;
                }
                break;
            case SPECIMEN_SCORING_HIGH:
                //pivot up
                robot.pivotRunToPosition(-1400);
                //wait
                robot.rollTo(specimenHighOuttakeRollTarget);
                //arm up
                robot.slideRunToPosition(sampleSlideDropOffPos);
                //wait for drive
                if(specimenScored) {
                    currentState = gameState.ARM_DOWN;
                }
                break;
            case SPECIMEN_SCORING_LOW:
                //pivot up
                //wait
                //arm up
                //wait for drive
                currentState = gameState.ARM_DOWN;
                break;
            case SAMPLE_SCORING_HIGH_1:
                robot.pivotRunToPosition(samplePivotDropOffPos);
                isArmUp = true;
                //pivot up
                //wait
//                raiseSlidesSample(gamepad1.a);

                break;
            case SAMPLE_SCORING_HIGH_2:
                isSlideUp = true;
                robot.pivotRunToPosition(samplePivotDropOffPos);
                robot.slideRunToPosition(sampleSlideDropOffPos);
                robot.pitchTo(sampleOuttakePitchTarget);
                robot.rollTo(sampleOuttakeRollTarget);
                outtakeTimer.reset();
//                if(gamepad1.a){
//                    currentState = gameState.SAMPLE_SCORING_HIGH_3;
//                }
                //arm up
                //wait for score
                break;
            case SAMPLE_SCORING_HIGH_3:
                robot.outake(true);
                if(outtakeTimer.milliseconds() > 600) {
                    currentState = gameState.ARM_DOWN;
                }
                //outtake
                //wait arm down
                break;
            case SAMPLE_SCORING_LOW_1:
                //pivot up
                //wait
                currentState = gameState.SAMPLE_SCORING_LOW_2;
                break;
            case SAMPLE_SCORING_LOW_2:
                //arm up
                //wait for score
                break;
            case PRE_DRIVE:
                robot.pitchTo(normalIntakePitchTarget);
                robot.rollTo(normalIntakeRollTarget);
                robot.pivotRunToPosition(0);
                robot.slideRunToPosition(0);
                robot.stopRoller();
                currentState = gameState.DRIVE;
//                subIntake(gamepad1.b);
//                raisePivotSample(gamepad1.a);
                //reset slide, pivot motors, close claw
                break;
            case DRIVE:

                break;
            case HANG_UP:
                robot.rollTo(groundIntakeRollTarget);
                robot.pitchTo(groundIntakePitchTarget);
                robot.pivotRunToPosition(-2100);
                break;
            case HANG_DOWN:
                robot.pivotRunToPosition(0);
                robot.slideRunToPosition(770);
                break;
            case ARM_DOWN:
                robot.pitchTo(normalIntakePitchTarget);
                robot.rollTo(normalIntakeRollTarget);
                robot.slideRunToPosition(0);
                if(slidesDownTimer.milliseconds() > 300 && robot.getSlidePosition() < 200) {
                    currentState = gameState.PIVOT_DOWN;
                }
                break;
            case PIVOT_DOWN:
                robot.pivotRunToPosition(0);
                if(pivotDownTimer.milliseconds() > 100) {
                    currentState = gameState.PRE_DRIVE;
                }
                break;
        }
    }
    public void subIntake (boolean button){
        if(button) {
            intakeDown = true;
            currentState = gameState.SUBMERSIBLE_INTAKE_1;

            subIntakeTimer.reset();
        }
    }
    public void driveSlides (boolean up, boolean down){
        if(up && (robot.getSlidePosition() < maximumSlidePos)){
            robot.slideRunToPosition(robot.getSlidePosition() + 50);
        } else if
        (down && (robot.getSlidePosition() > minimumSlidePos)){
            robot.slideRunToPosition(robot.getSlidePosition() - 50);
        }

    }
    public void retractSubIntake (boolean button){
        if(button) {
            intakeDown = false;
            currentState = gameState.PRE_DRIVE;
        }
    }

    public void raisePivotSample (boolean button){
        if(button){
            currentState = gameState.SAMPLE_SCORING_HIGH_1;
            pivotUpTimer.reset();
        }
    }

    public void raiseSlidesSample (boolean button){
        if(button) {
            currentState = gameState.SAMPLE_SCORING_HIGH_2;

        }
    }

    public void setSlidePos (int pos){
        sampleSlideDropOffPos = pos;
    }


//    public void setIntake(boolean button1, boolean button2) {
//        if (button1 || button2 && timer3.milliseconds() > 500) {
//            intakeDown = true;
//            currentState = gameState.INTAKE_READY;
//            timer3.reset();
//        }
//    }
//
//    public void drive(boolean button) {
//        currentState = gameState.DRIVE;
//    }
//
//
//    public void startIntaking(boolean button1) {
//        if (button1 && timer4.milliseconds() > 200) {
//            rollIntake = !rollIntake;
//            if (rollIntake){
//                intakeSpin.setPower(.7);
//            } else {
//                intakeSpin.setPower(0);
//            }
//            timer4.reset();
//        }
//    }
//
//    public void stopIntaking(boolean button1) {
//        if (button1) {
//            intakeSpin.setPower(0);
//            rollIntake = false;
//        }
//    }
//
//    public void reverseIntaking(boolean button) {
//        if (button && timer6.milliseconds() > 200) {
//            reverseIntake = !reverseIntake;
//            if (reverseIntake){
//                intakeSpin.setPower(-02);
//            } else {
//                intakeSpin.setPower(0);
//            }
//            timer6.reset();
//        }
//    }
//
//    public void setStartTransfer(boolean button) {
//        startTransfer = button;
//        if (button) { timer2.reset(); }
//    }
//
//    public void startTransferring(boolean button) {
//        transfer = button;
//    }
//
//    public void changeTransferPosition(boolean goUp, boolean goDown) {
//        if (timer7.milliseconds() > 300) {
//            if (goUp) {
//                INTAKE_HINGE_TRANSFER += 0.015;
//                timer7.reset();
//            }
//            if (goDown) {
//                INTAKE_HINGE_TRANSFER -= 0.015;
//                timer7.reset();
//            }
//        }
//    }
//
//    public void setSlideHeight(boolean increaseHeight, boolean decreaseHeight) {
//
//        if (timer5.milliseconds() > 200){
//            if (increaseHeight) {
//                slidePosition++;
//            }
//            if (decreaseHeight) {
//                slidePosition--;
//            }
//            if (slidePosition < 0) {
//                slidePosition = 0;
//            }
//            if (slidePosition > maxSlideHeight) {
//                slidePosition = maxSlideHeight;
//            }
//            slideHeight = slidePosition * 300 + BASE_SLIDE_HEIGHT;
//            if (currentState != gameState.LINEAR_SLIDE_UP) {
//                //move slide to slideHeight
//            }
//            timer5.reset();
//        }
//    }
//
//    public void hangUp(boolean button){
//        if(button) {
//            currentState = gameState.HANG_UP;
//            startHang = true;
//        }
//    }
//
//    public void hangDown(boolean button) {
//        if(button) {
//            downHang = true;
//        }
//    }
//
//    public void slideUp(boolean up) {
//        if (up) {
//            slideUp = true;
//        }
//        currentState = gameState.LINEAR_SLIDE_UP;
//    }
//
//    public void slideDown(boolean down) {
//        if (down) {
//            slideUp = false;
//        }
//        currentState = gameState.LINEAR_SLIDE_DOWN;
//    }


}

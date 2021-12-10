package org.firstinspires.ftc.teamcode.OpModes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Frenzy", group = "Robot15173")
public class FrenzyModeBase extends LinearOpMode {

    // Declare OpMode Members
    FrenzyBot robot = new FrenzyBot();
    IBaseOdometry odometry = null;

    // Timing related variables
    long GAMEPAD_LOCKOUT_TIME_MS = 200;
    Deadline gamepadRateLimit;

    // Intake related variables
    boolean changedIntake = false;
    boolean intakeReverse = false;

    // Rotator related variable
    boolean changedRotator1 = false;
    boolean changedRotator2 = false;

    private static String TAG = "FrenzyModeBase";

    @Override
    public void runOpMode() {
        try {
            try{
                gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT_TIME_MS, TimeUnit.MILLISECONDS);

                robot.init(this, this.hardwareMap, telemetry);


                int startXInches = 50;
                int startYInches = 15;
                int startHeading = 180;

                odometry =  VSlamOdometry.getInstance(this.hardwareMap, 20, startXInches, startYInches, startHeading);

                Thread odometryThread = new Thread(odometry);
                odometryThread.start();
            } catch (Exception ex) {
                telemetry.addData("Init", ex.getMessage());
            }
            telemetry.addData("Bot Info", robot.printInfo());

            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            while (opModeIsActive()) {
                handleDriveTrain();
                handleSpecialActions();
                if (robot.isIntakeRunning() && !robot.isIntakeBoxEmpty()){
                    robot.stopIntake();
                    changedIntake = !changedIntake;
                }

                sendTelemetry();
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(1000);
        }
        finally {
            robot.activateIntake(0);
            robot.activateRotatorRight(0);
            robot.activateLift(0);
            if (odometry != null) {
                odometry.stop();
            }
        }
    }

    private void sendTelemetry() {
        telemetry.addData("Left front", "%.3f", robot.getLeftOdometer());
        telemetry.addData("Right front", "%.3f", robot.getRightOdometer());
        telemetry.addData("Left Back", "%.3f", robot.getLeftBackOdometer());
        telemetry.addData("Right Back", "%.3f", robot.getRightBackOdometer());
        telemetry.addData("X", "%.3f", odometry.getCurrentX());
        telemetry.addData("Y", "%.3f", odometry.getCurrentY());
        telemetry.addData("Heading", "%.3f", odometry.getOrientation());
        telemetry.addData("Heading Adjusted", "%.3f", odometry.getAdjustedCurrentHeading());
        telemetry.addData("List position", "%d", robot.getLiftPosition());
        telemetry.update();
    }

    private void handleDriveTrain() {
        // DRIVING
        double drive = -gamepad1.left_stick_y; //negative to invert positive to normal
        double turn = 0;
        double ltrigger = gamepad1.left_trigger;
        double rtrigger = gamepad1.right_trigger;
        if (ltrigger > 0) {
            turn = -ltrigger;
        } else if (rtrigger > 0) {
            turn = rtrigger;
        }

        double strafe = gamepad1.right_stick_x;

        if (Math.abs(strafe) > 0) {
            if (strafe > 0) { // if want to invert change sign
                robot.strafeRight(Math.abs(strafe));
            } else {
                robot.strafeLeft(Math.abs(strafe));
            }
        } else {
            robot.move(drive, turn);
        }
    }

    protected void handleSpecialActions() {
        handleLift();
//        handleLiftManual();
        handleDropper();
        handleIntake();
        handleOuttake();
        handleTurntable();
        handleTurntable2();
    }

    protected void handleIntake() {
        if (isButtonPressable()) {
            if (gamepad2.right_bumper){
                startGamepadLockout();
                changedIntake = !changedIntake;

                if (changedIntake){
                    robot.startIntake();
                } else {
                    robot.stopIntake();
                }
            }
        }
    }

    protected void handleOuttake() {
        if (isButtonPressable()) {
            if (gamepad2.left_bumper) {
                startGamepadLockout();

                intakeReverse = !intakeReverse;

                if (intakeReverse) {
                    robot.reverseIntake();
                } else {
                    robot.stopIntake();
                }
            }
        }
    }

    protected void handleTurntable() {
        if (isButtonPressable()) {
            if (gamepad1.y) {
                startGamepadLockout();
                changedRotator1 = !changedRotator1;

                if (changedRotator1) {
                    robot.startTurntableRed(); //change back MAKE SURE
                } else {
                    robot.stopTurntable();
                }
            }
        }
    }
    protected void handleTurntable2() {
        if (isButtonPressable()) {
            if (gamepad1.x) {
                startGamepadLockout();
                changedRotator2 = !changedRotator2;

                if (changedRotator2) {
                    robot.startTurntableRedGradual();
                } else {
                    robot.startTurntableRedGradual();
                }
            }
        }
    }


    protected void handleLiftManual() {
        if (isButtonPressable()) {
            double liftVal = gamepad2.right_stick_y;
            robot.activateLift(liftVal);

        }
    }

    protected void handleLift() {
        if (isButtonPressable() && !robot.isLiftBusy()) {
            //lower level
            double liftVal = gamepad2.left_stick_y;
            if (liftVal > 0.5) {
                startGamepadLockout();
                robot.liftToLower();
            } else if (liftVal < -0.5) {
                startGamepadLockout();
                robot.liftToLevel1();
            }
            //upper level
            double liftValUpper = gamepad2.right_stick_y;
            if (liftValUpper > 0.5) {
                startGamepadLockout();
                robot.liftToLower();
            } else if (liftValUpper < -0.5) {
                startGamepadLockout();
                robot.liftToLevel3();
            }
        }
    }

    protected void handleDropper() {
        if ( gamepad2.dpad_up){
            robot.dropElement();
        }
        else if (gamepad2.dpad_down){
            robot.resetDropper();
        }
    }

    private void startGamepadLockout() {
        gamepadRateLimit.reset();
    }
    private boolean isButtonPressable() {
        return gamepadRateLimit.hasExpired();
    }
}

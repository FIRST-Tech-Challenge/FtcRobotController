package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.FrenzyBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;
import org.firstinspires.ftc.teamcode.odometry.VSlamOdometry;

import java.util.concurrent.TimeUnit;


public class FrenzyModeBase extends LinearOpMode {

    // Declare OpMode Members
    FrenzyBot robot = new FrenzyBot();
    IBaseOdometry odometry = null;

    protected boolean robotMoving = false;

    private boolean emergencyMode = false;

    // Timing related variables
    long GAMEPAD_LOCKOUT_TIME_MS = 200;
    Deadline gamepadRateLimit;

    // Intake related variables
    boolean changedIntake = false;
    boolean intakeReverse = false;

    // Rotator related variable
    boolean changedRotator1 = false;
    boolean changedRotator2 = false;

    boolean towerDefault = false;

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
                    if (!emergencyMode) {
                        robot.smartStopIntake();//robot.smartStopIntake();
//                        robot.stopIntake();
                    }
                    else{
                        robot.stopIntake();
                    }
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
        telemetry.addData("Lift position", "%d", robot.getLiftPosition());
        telemetry.addData("Turret position", "%d", robot.getTurretPosition());
        telemetry.update();
    }

    private void handleDriveTrain() {
        // DRIVING
        double drive = -gamepad1.left_stick_y; //negative to invert positive to normal
        double turn = 0;
        double ltrigger = gamepad1.right_trigger;
        double rtrigger = gamepad1.left_trigger;
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
        robotMoving = Math.abs(drive) > 0.02 || Math.abs(turn) > 0.02;
    }

    protected void handleSpecialActions() {
//        handleLift();
        handleLiftManual();
        handleDropper();
        handleIntake();
        handleOuttake();
        handleTurntable();
        handleTower();
        depositToTeamHub();
        depositToSharedHub();
        handleIntakeDropper();
        handleEmergency();
    }

    protected void depositToTeamHub() {
    }

    protected void depositToSharedHub() {
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
                    robot.stopOuttake();
                }
            }
        }
    }

    protected void handleTurntable() {
    }


    protected void handleTower() {
        if (isButtonPressable()) {
            double turretValue = gamepad2.left_stick_x;
            robot.activateTurret(turretValue);
        }
    }

    protected void handleTowerRemoveElement() {
        if (isButtonPressable()) {
            if (gamepad1.dpad_left) {
                startGamepadLockout();
                robot.turretToTeamHubRed();
            } else if(gamepad1.dpad_right) {
                robot.towerToTeamHubBlue();
            }
        }
    }


    protected void handleLiftManual() {
        if (isButtonPressable()) {
            double liftVal = -gamepad2.right_stick_y;
            robot.activateLift(liftVal);

        }
    }

    protected void handleDropper() {
        if (isButtonPressable()) {
            if (gamepad1.dpad_right) {
                robot.dropElement();
                startGamepadLockout();
            } else if (gamepad1.dpad_left) {
                robot.resetDropper();
                startGamepadLockout();
            }
        }
    }

    protected void handleIntakeDropper() {
        if (isButtonPressable()) {
            if (gamepad2.dpad_down) {
                robot.intakeDropperUp();
//                robot.smartStopIntake();
                startGamepadLockout();
            } else if (gamepad2.dpad_up) {
                robot.intakeDropperDown();
                startGamepadLockout();
            }
            else if (gamepad2.dpad_right || gamepad2.dpad_left){
                robot.intakeDropperHalfWay();
                startGamepadLockout();
            }
        }
    }

    protected void handleEmergency() {
        if (isButtonPressable()) {
            if (gamepad1.a) {
                startGamepadLockout();
                setEmergencyMode(!emergencyMode);
            }
        }
    }

    protected void startGamepadLockout() {
        gamepadRateLimit.reset();
    }
    protected boolean isButtonPressable() {
        return gamepadRateLimit.hasExpired();
    }

    public boolean isEmergencyMode() {
        return emergencyMode;
    }

    public void setEmergencyMode(boolean emergencyMode) {
        this.emergencyMode = emergencyMode;
    }
}

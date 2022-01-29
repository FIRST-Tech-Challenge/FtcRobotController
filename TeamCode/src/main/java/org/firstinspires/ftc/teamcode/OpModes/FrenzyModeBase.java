package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private boolean manualDriveMode = false;
    private boolean manualLiftMode = false;

    // Timing related variables
    long GAMEPAD_LOCKOUT_TIME_MS = 500;
    Deadline gamepad1RateLimit;
    Deadline gamepad2RateLimit;

    // Intake related variables
    boolean changedIntake = false;
    boolean intakeReverse = false;

    // Rotator related variable
    boolean changedRotator1 = false;
    boolean changedRotator2 = false;

    boolean towerDefault = false;

    private static String TAG = "FrenzyModeBase";

    private ElapsedTime opModeTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            try{
                gamepad1RateLimit = new Deadline(GAMEPAD_LOCKOUT_TIME_MS, TimeUnit.MILLISECONDS);
                gamepad2RateLimit = new Deadline(GAMEPAD_LOCKOUT_TIME_MS, TimeUnit.MILLISECONDS);
                robot.setTeleOp(true);

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
            opModeTime.reset();
            while (opModeIsActive()) {
                handleDriveTrain();
                handleSpecialActions();
                if (robot.isIntakeRunning() && !robot.isIntakeBoxEmpty()){
                    if (!emergencyMode) {
                        robot.smartStopIntakeAsync();
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
//        telemetry.addData("range", String.format("%.01f in", robot.getDistance()));
        telemetry.addData("Tape Position", String.format("%.01f in", robot.getTapePosition()));
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
            if (isManualDriveMode()) {
                strafe = strafe * 0.8;
            }
            if (strafe > 0) { // if want to invert change sign
                robot.strafeRight(Math.abs(strafe));
            } else {
                robot.strafeLeft(Math.abs(strafe));
            }
        } else {
            if (isManualDriveMode()){
                drive = drive*0.8;
            }
            robot.move(drive, turn);
        }
        robotMoving = Math.abs(drive) > 0.02 || Math.abs(turn) > 0.02;
    }

    protected void handleSpecialActions() {
        handleDropper();
        handleIntake();
        handleOuttake();
        handleTurntable();
        if (isManualLiftMode()) {
            handleManualTape();
            handleTower();
            handleLiftManual();
            handleTapeMeasure();
            handleTapeMeasureUpDown();
            handleManualTurretOffset();
        }
        else {
            depositToTeamHub();
            depositToSharedHub();
            handleScoring();
        }
        handleIntakeDropper();
        handleEmergency();
        handleManualDrive();
        handleManualLift();
    }

    protected void depositToTeamHub() {
    }

    protected void depositToSharedHub() {
    }

    protected void handleIntake() {
        if (isGamepad2Pressable()) {
            if (gamepad2.right_bumper){
                lockGamepad2();
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
        if (isGamepad2Pressable()) {
            if (gamepad2.left_bumper) {
                lockGamepad2();

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
        double turretValue = gamepad2.right_stick_x;
        robot.activateTurret(turretValue);
    }

    protected void handleManualTurretOffset(){
        if (isGamepad2Pressable()) {
            if (gamepad2.b){
                lockGamepad2();
                //turret can be reset only once within the first 10 seconds of the match
                if (!robot.isTurretOffsetDefined()) {
                    robot.defineTurretOffset();
                }
            }
        }
    }

    protected void handleTowerRemoveElement() {
        if (isGamepad1Pressable()) {
            if (gamepad1.dpad_left) {
                lockGamepad1();
                robot.turretToTeamHubRed();
            } else if(gamepad1.dpad_right) {
                lockGamepad1();
                robot.towerToTeamHubBlue();
            }
        }
    }

    protected void handleLiftManual() {
        double liftVal = -gamepad2.right_stick_y;
        robot.activateLift(liftVal);
    }

    protected void handleTapeMeasure() {
        double val = gamepad2.left_stick_x;
        robot.activateTapeMeasure(val);
    }

    protected void handleTapeMeasureUpDown() {
        if (isGamepad2Pressable()) {
            lockGamepad2();
            double val = gamepad2.left_stick_y;
            robot.moveTapeMeasureUpDown(-val);
        }
    }

    protected void handleDropper() {
        if (isGamepad1Pressable()) {
            if (gamepad1.dpad_right) {
                lockGamepad1();
                robot.dropElement();
            } else if (gamepad1.dpad_left) {
                lockGamepad1();
                robot.resetDropper();
            }
        }
    }

    protected void handleIntakeDropper() {
        if (isGamepad2Pressable()) {
            if (gamepad2.dpad_down) {
                lockGamepad2();
                robot.intakeDropperUp();
//                robot.smartStopIntake();
            } else if (gamepad2.dpad_up) {
                lockGamepad2();
                robot.intakeDropperDown();
            }
            else if (gamepad2.dpad_right || gamepad2.dpad_left){
                lockGamepad2();
                robot.intakeDropperHalfWay();
            }
        }
    }

    protected void handleEmergency() {
        if (isGamepad1Pressable()) {
            if (gamepad1.a) {
                lockGamepad1();
                setEmergencyMode(!emergencyMode);
            }
        }
    }

    protected void handleScoring() {
        if (isGamepad2Pressable()) {
            if (gamepad2.a) {
                lockGamepad2();
                robot.scoreAndFoldAsync();
            }
        }
    }

    protected void handleManualDrive() {
        if (isGamepad1Pressable()) {
            if (gamepad1.start) {
                lockGamepad1();
                setManualDriveMode(!manualDriveMode);
            }
        }
    }

    protected void handleManualLift() {
        if (isGamepad2Pressable()) {
            if (gamepad2.start) {
                lockGamepad2();
                setManualLiftMode(!manualLiftMode);
                if (isManualLiftMode()) {
                    robot.liftToLevelEndgame();
                }
            }
        }
    }

    protected void handleManualTape() {
        if (isGamepad2Pressable()) {
            if (gamepad2.a) {
                lockGamepad2();
                robot.liftTapeEndgame();
            }
        }
    }

    protected void lockGamepad1() {
        gamepad1RateLimit.reset();
    }

    protected void lockGamepad2() {
        gamepad2RateLimit.reset();
    }

    protected boolean isGamepad1Pressable() {
        return gamepad1RateLimit.hasExpired();
    }

    protected boolean isGamepad2Pressable() {
        return gamepad2RateLimit.hasExpired();
    }

    public boolean isEmergencyMode() {
        return emergencyMode;
    }

    public void setEmergencyMode(boolean emergencyMode) {
        this.emergencyMode = emergencyMode;
    }

    public boolean isManualDriveMode() {
        return manualDriveMode;
    }

    public void setManualDriveMode(boolean manualDriveMode) {
        this.manualDriveMode = manualDriveMode;
    }

    public boolean isManualLiftMode() {
        return manualLiftMode;
    }

    public void setManualLiftMode(boolean manualLiftMode) {
        this.manualLiftMode = manualLiftMode;
    }
}

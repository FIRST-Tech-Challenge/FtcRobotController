package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.bots.PowerPlayBaseBot;

import java.util.concurrent.TimeUnit;

@TeleOp(name="PowerPlay", group="Robot15173")
//@Disabled
public class PowerPlayMode extends LinearOpMode {

    PowerPlayBaseBot robot = new PowerPlayBaseBot();
    private ElapsedTime opModeRunTime = new ElapsedTime();

    // Timing related variables
    long GAMEPAD_LOCKOUT_TIME_MS = 500;
    Deadline gamepad1RateLimit;
    Deadline gamepad2RateLimit;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            try {
                gamepad1RateLimit = new Deadline(GAMEPAD_LOCKOUT_TIME_MS, TimeUnit.MILLISECONDS);
                gamepad2RateLimit = new Deadline(GAMEPAD_LOCKOUT_TIME_MS, TimeUnit.MILLISECONDS);

                robot.init(this, this.hardwareMap, telemetry);
            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            opModeRunTime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                handleDriveTrain();
                handleSpecialActions();
                sendTelemetry();

                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
        }
    }

    private void sendTelemetry() {
//        telemetry.addData("Left front", "%.3f", robot.getLeftOdometer());
//        telemetry.addData("Right front", "%.3f", robot.getRightOdometer());
//        telemetry.addData("Left Back", "%.3f", robot.getLeftBackOdometer());
//        telemetry.addData("Right Back", "%.3f", robot.getRightBackOdometer());
//        telemetry.addData("X", "%.3f", odometry.getCurrentX());
//        telemetry.addData("Y", "%.3f", odometry.getCurrentY());
//        telemetry.addData("Heading", "%.3f", odometry.getOrientation());
//        telemetry.addData("Heading Adjusted", "%.3f", odometry.getAdjustedCurrentHeading());
//        telemetry.addData("Lift position", "%d", robot.getLiftPosition());
//        telemetry.addData("Turret position", "%d", robot.getTurretPosition());
////        telemetry.addData("range", String.format("%.01f in", robot.getDistance()));
//        telemetry.addData("Tape Position", String.format("%.01f in", robot.getTapePosition()));
        telemetry.update();
    }

    protected void handleDriveTrain() {
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
            if (strafe > 0) {
                robot.strafeRight(Math.abs(strafe));
            } else {
                robot.strafeLeft(Math.abs(strafe));
            }
        } else {
            robot.move(drive, turn);
        }
    }

    protected void handleSpecialActions() {
        handleGrabber();
    }

    protected void handleGrabber() {
        if (isGamepad1Pressable()) {
            if (gamepad1.dpad_right) {
                lockGamepad1();
                robot.grabCone();
            } else if (gamepad1.dpad_left) {
                lockGamepad1();
                robot.releaseCone();
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

}

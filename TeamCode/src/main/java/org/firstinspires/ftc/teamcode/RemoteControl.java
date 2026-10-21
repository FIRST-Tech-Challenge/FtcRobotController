package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class RemoteControl extends LinearOpMode {
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        double intakeSpeed = 0.85;
        double triggerSpeed = 0.5;
        double shooterVelocity = 1675;
        double pValue = 90;
        boolean halfWheelSpeed = false;
        double wheelSpeedMultiplier = 1.0;

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        r.setpValue(pValue);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "10/19/2026, 6:05 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        waitForStart();
        r.intakePower(intakeSpeed);
        r.shooterVelocity(shooterVelocity);
        ElapsedTime telemetryTimer = new ElapsedTime();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            r.setWheelPower(
                -gamepad1.left_stick_y * wheelSpeedMultiplier,
                -gamepad1.left_stick_x * wheelSpeedMultiplier,
                gamepad1.right_stick_x * wheelSpeedMultiplier);

            // Control the trigger
            if(gamepad1.right_trigger > 0) {
                r.triggerPower(triggerSpeed);
            }
            else if(gamepad1.left_trigger > 0) {
                r.triggerPower(-triggerSpeed);
            }
            else {
                r.triggerPower(0);
            }

            // Control the intake
            if(gamepad1.circle) {
                r.intakePower(intakeSpeed);
            }
            else if(gamepad1.square) {
                r.intakePower(-intakeSpeed);
            }
            else if(gamepad1.cross) {
                r.intakePower(0);
            }

            // Control the shooter
            if (gamepad1.triangleWasReleased()) {
                shooterVelocity *= -1;
                r.shooterVelocity(shooterVelocity);
            }
            if(gamepad1.dpad_up) {
                r.shooterVelocity(0);
            }
            if (gamepad1.rightBumperWasReleased()) {
                shooterVelocity = shooterVelocity + 5;
                r.shooterVelocity(shooterVelocity);
            }
            if (gamepad1.leftBumperWasReleased()) {
                shooterVelocity = shooterVelocity - 5;
                r.shooterVelocity(shooterVelocity);
            }

            if (gamepad1.dpadDownWasReleased()) {
                halfWheelSpeed = !halfWheelSpeed;
                wheelSpeedMultiplier = halfWheelSpeed ? 0.5 : 1;
            }

            // Control the lumberjack
            if (gamepad1.dpad_right) {
                r.setLumberjackPosition(0.66);
            }
            if (gamepad1.dpad_left) {
                r.setLumberjackPosition(0.03);
            }

            // Gamepad2 stuff - find good values for trigger, pValue, and intake
            if (gamepad2.dpadUpWasReleased()) {
                triggerSpeed = Math.min(1.0, triggerSpeed + 0.05);
            }
            if (gamepad2.dpadDownWasReleased()) {
                triggerSpeed = Math.max(-1.0, triggerSpeed - 0.05);
            }

            if (gamepad2.rightBumperWasReleased()) {
                r.adjustpValue(2);
            }
            if (gamepad2.leftBumperWasReleased()) {
                r.adjustpValue(-2);
            }

            if (gamepad2.circleWasReleased()) {
                intakeSpeed = Math.min(1, intakeSpeed + 0.05);
                r.intakePower(intakeSpeed);
            }
            if (gamepad2.squareWasReleased()) {
                intakeSpeed = Math.max(-1, intakeSpeed - 0.05);
                r.intakePower(intakeSpeed);
            }

            // Show the elapsed game time and wheel power, but only every half-second.
            if (telemetryTimer.seconds() >= 0.5) {
                telemetry.addData("Trigger speed setting", "%.0f%%", triggerSpeed * 100);
                telemetry.addData("Intake speed setting", "%.0f%%", intakeSpeed * 100);
                telemetry.addData("Wheel speed multiplier", "%.0f%%", wheelSpeedMultiplier * 100);
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
            idle();
        }
        r.releaseLumberjack();
    }
}
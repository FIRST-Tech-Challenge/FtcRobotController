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

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        r.setpValue(60);

        double intakeSpeed = 0.5;
        //Starting robot shooter speed. Ideal for shooting. Can be adjusted in-game.
        double shooterSpeed = 0.76;
        int shooterDirection = 1;
        double triggerSpeed = 1.0;
        double triggerPower = 0;
        boolean previousRightBumper = false;
        boolean previousLeftBumper = false;
        boolean previousTriangle = false;
        boolean previousDpadRight = false;
        boolean previousDpadLeft = false;
        boolean previousDpadDown = false;
        boolean previousRightBumper2 = false;
        boolean previousLeftBumper2 = false;
        boolean previousDpadUp2 = false;
        boolean previousDpadDown2 = false;
        boolean previousCircle2 = false;
        boolean previousSquare2 = false;
        boolean halfWheelSpeed = false;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "10/18/2026, 4:44 pm"); // Todo: Update this date when the code is updated
        telemetry.update();
        waitForStart();
        r.intakePower(intakeSpeed);
        r.shooterPower(shooterSpeed * shooterDirection);

        // Timer used to throttle telemetry updates to every half-second
        ElapsedTime telemetryTimer = new ElapsedTime();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // r.start();
            boolean speedChanged = false;
            
            if(gamepad1.dpad_up) {
                shooterSpeed = 0.0;
                speedChanged = true;
            }
            if (gamepad1.dpad_down && !previousDpadDown) {
                halfWheelSpeed = !halfWheelSpeed;
            }
            double wheelSpeedMultiplier = halfWheelSpeed ? 0.5 : 1.0;
            r.setWheelPower(
                    -gamepad1.left_stick_y * wheelSpeedMultiplier,
                    -gamepad1.left_stick_x * wheelSpeedMultiplier,
                    gamepad1.right_stick_x * wheelSpeedMultiplier);

            if (gamepad2.dpad_up && !previousDpadUp2) {
                triggerSpeed = Math.min(1.0, triggerSpeed + 0.05);
            }
            if (gamepad2.dpad_down && !previousDpadDown2) {
                triggerSpeed = Math.max(0.0, triggerSpeed - 0.05);
            }

            if(gamepad1.right_trigger > 0)
            {
                triggerPower = triggerSpeed;
                r.triggerPower(triggerPower);
            }

            if(gamepad1.left_trigger > 0)
            {
                triggerPower = -triggerSpeed;
                r.triggerPower(triggerPower);
            }

            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                triggerPower = 0;
                r.triggerPower(triggerPower);
            }

            if(gamepad1.circle){
                intakeSpeed = 0.5;
                r.intakePower(intakeSpeed);
            }
            else if(gamepad1.square){
                intakeSpeed = -0.5;
                r.intakePower(intakeSpeed);
            }
            else if(gamepad1.cross){
                intakeSpeed = 0;
                r.intakePower(intakeSpeed);
            }

            if (gamepad1.right_bumper && !previousRightBumper) {
                shooterSpeed = Math.min(1.0, shooterSpeed + 0.01);
                speedChanged = true;
            }
            if (gamepad1.left_bumper && !previousLeftBumper) {
                shooterSpeed = Math.max(0.0, shooterSpeed - 0.01);
                speedChanged = true;
            }
            if (gamepad1.triangle && !previousTriangle) {
                shooterDirection *= -1;
                speedChanged = true;
            }

            if (gamepad2.right_bumper && !previousRightBumper2) {
                r.adjustpValue(2);
            }
            if (gamepad2.left_bumper && !previousLeftBumper2) {
                r.adjustpValue(-2);
            }
            if (gamepad1.dpad_right && !previousDpadRight) {
                r.setLumberjackPosition(0.65);
            }
            if (gamepad1.dpad_left && !previousDpadLeft) {
                r.setLumberjackPosition(0.06);
            }

            if (gamepad2.circle && !previousCircle2) {
                intakeSpeed = Math.min(1, intakeSpeed + 0.05);
                r.intakePower(intakeSpeed);
            }
            if (gamepad2.square && !previousSquare2) {
                intakeSpeed = Math.max(-1, intakeSpeed - 0.05);
                r.intakePower(intakeSpeed);
            }

            if (speedChanged) {
                r.shooterPower(shooterSpeed * shooterDirection);
            }

            previousRightBumper = gamepad1.right_bumper;
            previousLeftBumper = gamepad1.left_bumper;
            previousTriangle = gamepad1.triangle;
            previousDpadRight = gamepad1.dpad_right;
            previousDpadLeft = gamepad1.dpad_left;
            previousDpadDown = gamepad1.dpad_down;
            previousRightBumper2 = gamepad2.right_bumper;
            previousLeftBumper2 = gamepad2.left_bumper;
            previousDpadUp2 = gamepad2.dpad_up;
            previousDpadDown2 = gamepad2.dpad_down;
            previousCircle2 = gamepad2.circle;
            previousSquare2 = gamepad2.square;

            // Show the elapsed game time and wheel power, but only every half-second.
            if (telemetryTimer.seconds() >= 0.5) {
                telemetry.addData("Trigger speed setting", "%.0f%%", triggerSpeed * 100);
                telemetry.addData("Intake speed setting", "%.0f%%", intakeSpeed * 100);
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
            idle();
        }
        r.releaseLumberjack();
    }
}
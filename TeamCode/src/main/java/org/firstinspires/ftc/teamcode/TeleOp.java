package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Teleop mode code goes here
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Robot")
public class TeleOp extends RobotOpMode {
    double drive = 0;
    double turn = 0;
    @Override
    public void init() {
        telemetryPrinter.helloWorld("Chameleons");
    }

    @Override
    public void init_loop() {
        telemetryPrinter.debug("INIT_LOOP");
    }

    @Override
    public void start() {
        telemetryPrinter.print("Starting TeleOp...");

    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            this.drive = -gamepad1.left_stick_y;
            telemetryPrinter.debug(String.format("drive = %d", this.drive));
        } else {
            telemetryPrinter.debug("Not enough push to move fwd/back");
            this.drive = 0;
        }
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            this.turn = gamepad1.left_stick_x;
            telemetryPrinter.debug(String.format("turn = %d", this.turn));
        } else {
            telemetryPrinter.debug("Not enough push to turn");
            this.turn = 0;
        }

        robot.driveRobot(this.drive, this.turn);

    }

    @Override
    public void stop() {
        telemetryPrinter.print("Stopping TeleOp...");
    }
}

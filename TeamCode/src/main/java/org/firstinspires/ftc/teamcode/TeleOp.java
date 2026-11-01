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
    double heading = 0;
    @Override
    public void init() {
        telemetryPrinter.helloWorld("Chameleons");
        robot.init();
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
        heading = robot.getHeading();
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;

        double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
        double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

        if (Math.abs(adjustedLy) > 0.1) {
            this.drive = -adjustedLy;
            telemetryPrinter.debug(String.format("drive = %d", this.drive));
        } else {
            telemetryPrinter.debug("Not enough push to move fwd/back");
            this.drive = 0;
        }
        if (Math.abs(adjustedLx) > 0.1) {
            this.turn = adjustedLx;
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

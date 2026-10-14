package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Shoot and Park", group="Robot")
public class ShootPark extends LinearOpMode {
    private static final double SHOOTER_POWER = 0.8;
    private static final double TRIGGER_POWER = 1.0;
    private static final long ONE_SECOND_MS = 1000;

    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot, then park");
        telemetry.addData("Route", "(500, -800) to (1150, -2050) mm");
        telemetry.addData("Version", "v20261013-07");
        telemetry.update();
        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        // Start motors
        robot.intakePower(-0.8);
        robot.shooterPower(SHOOTER_POWER);
        sleep(2000); // let shooters get up to speed

        // Shoot 1st round
        robot.triggerPower(TRIGGER_POWER);
        sleep(ONE_SECOND_MS);
        robot.triggerPower(0);
        sleep(ONE_SECOND_MS);

        // Shoot 2nd round
        robot.triggerPower(TRIGGER_POWER);
        sleep(ONE_SECOND_MS);
        robot.triggerPower(0);
        sleep(ONE_SECOND_MS);

        // Park
        robot.intakePower(0);
        robot.shooterPower(0);
        robot.driveToPos(1000, -600, 0, 10, 2, 6); // Waypoint to avoid pole
        robot.driveToPos(1500, -2000, 0, 10, 2, 10);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
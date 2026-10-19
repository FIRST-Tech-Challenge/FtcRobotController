package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Shoot and Park Auto", group="Robot")
public class ShootParkAuto extends LinearOpMode {
    private static final double SHOOTER_POWER = 0.8;
    private static final double TRIGGER_POWER = 1.0;
    private static final long ONE_SECOND_MS = 1000;

    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot, then park");
        telemetry.addData("Route", "(500, -800) to (1150, -2050) mm");
        telemetry.addData("Version", "v20261014-01");
        telemetry.update();
        waitForStart();

        // Start motors
        robot.shooterPower(SHOOTER_POWER);
        sleep(3000); // let shooters get up to speed

        // Shoot 1st round
        robot.intakePower(-0.8);
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
        robot.driveToPos(900, -300, 0, 50, 2, 2, 0.5); // Waypoint to avoid pole
        robot.driveToPos(1200, -2100, 0, 50, 2, 4, 0.5);

        robot.odo.update();
        Pose2D pose = robot.odo.getPosition();
        telemetry.addData("X (mm)", "%.1f", -pose.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
        telemetry.addData("Heading (degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
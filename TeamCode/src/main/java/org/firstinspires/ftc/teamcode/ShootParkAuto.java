package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Shoot and Park Auto", group="Robot")
public class ShootParkAuto extends LinearOpMode {
    private static final double SHOOTER_VELOCITY = 1850;
    private static final double TRIGGER_POWER = 1;
    private static final double INTAKE_POWER = 0.8;

    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot, then park");
        telemetry.addData("Route", "(500, -800) to (1150, -2050) mm");
        telemetry.addData("Version", "v20261021-24");
        telemetry.update();
        waitForStart();

        // Start motors
        robot.shooterVelocity(SHOOTER_VELOCITY);
        robot.setpValue(90);
        sleep(4000); // let shooters get up to speed

        // Shoot 1st round: balls 1 & 2
        robot.intakePower(INTAKE_POWER);
        robot.triggerPower(TRIGGER_POWER);
        sleep(100);
        robot.triggerPower(0);
        sleep(1500);

        // Shoot 2nd round: balls 3 & 4
        robot.triggerPower(TRIGGER_POWER);
        sleep(500);
        robot.triggerPower(0);
        sleep(1000);

        // This is good
        robot.driveToPos(0, -150, 0, 25, 5, 1, 0.4); // Waypoint to get off the wall
        robot.driveToPos(400, -20, -85, 50,2, 3, 0.7); // Move to collect balls
        robot.driveToPos(1100, 0, -85, 50, 2, 2, 0.3); // Collect them
        sleep(1000);
        robot.driveToPos(0, 0, 0, 20, 1, 4); // Go back to shooting position

        // Shoot 3rd round: balls 5 & 6
        robot.triggerPower(TRIGGER_POWER);
        sleep(100);
        robot.triggerPower(0);
        sleep(1500);

        // Shoot 4th round: balls 7 & 8
        robot.triggerPower(TRIGGER_POWER);
        sleep(500);
        robot.triggerPower(0);
        sleep(1000);

        robot.shooterVelocity(0);
        robot.driveToPos(450, -950, -90, 50, 2, 3, 0.7); // make sure the lumberjack doesn't hit the flower
        robot.setLumberjackPosition(0.66);
        sleep(700);
        RobotLog.vv("Rockin' Robots", "Lumberjack Log 1");
        robot.driveToPos(750, -950, -90, 50, 2, 1, 0.3); // collect balls
        sleep(1000);
        RobotLog.vv("Rockin' Robots", "Lumberjack Log 2");
        robot.driveToPos(450, -950, -90, 50, 2, 3, 0.7); // make sure the lumberjack doesn't hit the flower
        robot.setLumberjackPosition(0.03);
        sleep(700);
        RobotLog.vv("Rockin' Robots", "Lumberjack Log 3");

        // Park
        robot.intakePower(0);
        robot.shooterVelocity(0);
        RobotLog.vv("Rockin' Robots", "Drive to the parking box");
        robot.driveToPos(1200, -2100, 0, 50, 2, 4, 0.4);

        robot.odo.update();
        Pose2D pose = robot.odo.getPosition();
        telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
        telemetry.addData("Heading (degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
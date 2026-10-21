package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Shoot and Park Auto", group="Robot")
public class ShootParkAuto extends LinearOpMode {
    private static final double SHOOTER_POWER = 0.76;
    private static final double TRIGGER_POWER = 1;

    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Shoot, then park");
        telemetry.addData("Route", "(500, -800) to (1150, -2050) mm");
        telemetry.addData("Version", "v20261020-08");
        telemetry.update();
        waitForStart();

        // Start motors
        robot.shooterPower(SHOOTER_POWER);
        robot.setpValue(70);
        sleep(3000); // let shooters get up to speed

        // Shoot 1st round: balls 1 & 2
        robot.intakePower(0.8);
        robot.triggerPower(TRIGGER_POWER);
        sleep(100);
        robot.triggerPower(0);
        sleep(1000);

        // Shoot 2nd round: balls 3 & 4
        robot.triggerPower(TRIGGER_POWER);
        sleep(500);
        robot.triggerPower(0);
        sleep(1000);

        robot.driveToPos(500, 13, -90, 50,2, 3, 0.6); // Move to collect balls
        robot.driveToPos(700, 20, -85, 50, 2, 2, 0.3); // Collect them
/*
        robot.driveToPos(0, 0, 0, 50, 2, 4); // Go back to shooting position
        // Shoot 3rd round: balls 5 & 6
        robot.intakePower(0.8);
        robot.triggerPower(TRIGGER_POWER);
        sleep(100);
        robot.triggerPower(0);
        sleep(1000);

        // Shoot 4th round: balls 7 & 8
        robot.triggerPower(TRIGGER_POWER);
        sleep(500);
        robot.triggerPower(0);
        sleep(1000);

        // Park
        robot.intakePower(0);
        robot.shooterPower(0);
        RobotLog.vv("Rockin' Robots", "Drive to the waypoint");
        robot.driveToPos(900, -300, 0, 50, 2, 2, 0.6); // Waypoint to avoid pole
        RobotLog.vv("Rockin' Robots", "Drive to the parking box");
        robot.driveToPos(1200, -2100, 0, 50, 2, 4, 0.4);
*/
        robot.odo.update();
        Pose2D pose = robot.odo.getPosition();
        telemetry.addData("X (mm)", "%.1f", -pose.getX(DistanceUnit.MM));
        telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
        telemetry.addData("Heading (degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}
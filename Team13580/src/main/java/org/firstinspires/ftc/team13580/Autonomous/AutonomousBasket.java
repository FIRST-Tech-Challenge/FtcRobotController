package org.firstinspires.ftc.team13580.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team13580.RobotHardware;

@Autonomous(name="Basket", group="Robot")
public class AutonomousBasket extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init();
        double heading;
        waitForStart();

        robot.leftHand.setPosition(-0.8);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        robot.encoderDrive(0.7, -12, 12, 12, -12, 15);
        robot.encoderDrive(0.8, -35, -35, 35, 35, 15);
        robot.encoderArm(87, 12);
        robot.setSpooliePower(100);
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        //robot.encoderSpoolie(2000, 127, 15);
    }


}
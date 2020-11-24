package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "R_mid3_P3w_park")
public class R_mid3_P3w_park extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        robot.moveWobbleGoalServo(false);
        robot.moveBackward(60, 0.5);
        telemetry.addData("Moving Backwards", 60);
        telemetry.update();
        robot.shootHighGoal(3);
        telemetry.addData("Shooting Rings", 3);
        telemetry.update();
        robot.moveLeft(24, 0.5);
    }
}
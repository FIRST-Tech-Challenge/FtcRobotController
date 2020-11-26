package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name= "R_Power_park")
public class R_Power_park extends LinearOpMode {
    final boolean debug = true;

    @Override
    public void runOpMode(){

        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        robot.moveAngle(0,-53,0.5);
        robot.shootHighGoal(3);
        robot.moveAngle(0,-20,0.5);
        sleep(500);
        stop();
    }



}

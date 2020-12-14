package org.firstinspires.ftc.teamcode.Qualifier_2.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_2.Robot;

@Autonomous(name = "Park")
@Disabled
public class R_park extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot=new Robot(this, BasicChassis.ChassisType.ENCODER);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        robot.moveForward(73, 0.5);
        sleep(5000);
//        robot.moveForwardOdometry(80, 0.8);
    }
}
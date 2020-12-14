package org.firstinspires.ftc.teamcode.Qualifier_2.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Qualifier_2.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_2.Components.Navigations.Odometry;
import org.firstinspires.ftc.teamcode.Qualifier_2.Robot;

@Autonomous(name= "shooter_autonomous")
@Disabled
public class R_mid3_park extends LinearOpMode {
    final boolean debug = true;

    @Override
    public void runOpMode(){

        Robot robot=new Robot(this, BasicChassis.ChassisType.ENCODER);
        Odometry odom = new Odometry();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        odom.init(this);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        robot.moveBackward(57,0.5);
        robot.shootHighGoal(3);
        robot.moveBackward(7,0.5);
        sleep(500);
        stop();
    }



}

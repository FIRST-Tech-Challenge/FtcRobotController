package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "R_mid3_P1w_park")
@Disabled
public class R_mid3_P1w_park extends LinearOpMode {
    final boolean debug = true;

    @Override
    public void runOpMode(){

        Robot robot=new Robot(this, BasicChassis.ChassisType.IMU, false, false);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        robot.moveWobbleGoalServo(false);
        robot.moveAngle(0,- 65,0.5);
        robot.moveWobbleGoalServo(true);
        robot.moveAngle(20,5, 0.5);
        sleep(2000);
        //robot.shootHighGoal(3);
        robot.moveAngle(0,-15,0.5);
        sleep(500);
        stop();
    }



}

package org.firstinspires.ftc.team12395.v1.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12395.v1.RobotHardware;

@Autonomous(name =  "Auto By Encoder60 pts", group = "Robot")
public class AutoByEncoder60 extends LinearOpMode{
    RobotHardware robot = new RobotHardware(this);
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();

        waitForStart();
        runtime.reset();


        //strafing is always 2 inches less than the inches stated in code

        //driving is always 1 inch more than said in code.

        //keeps slides retracted until they are used
        robot.setHorizontalPosition(0);

        robot.setIntakePosition(1);
        robot.setOutClawPosition(1);
        robot.setVerticalPower(0);


        robot.driveEncoder(.75,-20,-20,-20,-20);
        robot.driveEncoder(.75,7,-7,-7,7);
        robot.driveEncoder(.75,-9.6,-9.6,-9.6,-9.6);
        robot.SetSlidePosition(robot.SLIDE_HIGH_RUNG);
        robot.setHorizontalPosition(0);
        sleep(950);
        robot.SetSlidePosition(robot.SLIDE_START);
        robot.setOutClawPosition(0);
        robot.setVerticalPower(1);
        sleep(900);


        robot.driveEncoder(.75,5,5,5,5);
        //strafe right
        robot.driveEncoder(0.75,-33.5,33.5,33.5,-33.5);
        //forward
        robot.driveEncoder(0.75, -24, -24, -24, -24);
        //strafe right
        robot.driveEncoder(0.75, -16, 16, 16, -16);
        //drive backward and drop of specimen in oz
        robot.driveEncoder(0.6, 39, 39, 39, 39);
        //drive forward for alignment.
        robot.driveEncoder(0.75, -6, -6, -6, -6);
        sleep(1000);
        robot.setHorizontalPosition(1);
        robot.driveEncoder(0.1,3,3,3,3);
        sleep(100);
        robot.setInClawPosition(1);
        sleep(300);
        robot.setHorizontalPosition(0);
        sleep(750);
        robot.setOutClawPosition(1);
        sleep(100);
        robot.setInClawPosition(0);


        robot.driveEncoder(0.75,53,-53,-53,53);

        robot.setVerticalPower(0);

        robot.driveEncoder(.4,-24,-24,-24,-24);





        robot.SetSlidePosition(robot.SLIDE_HIGH_RUNG);
        robot.setHorizontalPosition(0);

        sleep(950);
        robot.SetSlidePosition(robot.SLIDE_START);
        robot.setOutClawPosition(0);
        robot.setVerticalPower(1);
        sleep(900);

        robot.driveEncoder(.6,5,5,5,5);
        //strafe right
        robot.driveEncoder(0.75,-33.5,33.5,33.5,-33.5);

        robot.driveEncoder(0.6, 20, 20, 20, 20);

        sleep(1000);
        robot.setHorizontalPosition(1);
        robot.driveEncoder(0.4,5,5,5,5);

    }
}

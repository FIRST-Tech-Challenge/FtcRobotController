package org.firstinspires.ftc.team12395.v1.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team12395.v1.RobotHardware;

@Autonomous(name =  "Auto By Encoder Bucket", group = "Robot")
@Disabled
public class AutoByEncoderBucket extends LinearOpMode{
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
        robot.setIntakePosition(1);
        robot.setOutClawPosition(0);
        robot.setOutClawPosition(1);
//
        robot.SetSlidePosition(robot.SLIDE_START);
        robot.setVerticalPower(0);
//
//
        robot.driveEncoder(1, -36, -36, -36, -36);
        robot.driveEncoder(.5, -5,-5,5,5);
        robot.SetSlidePosition(robot.SLIDE_HIGH_BASKET);
        sleep(700);
        robot.setVerticalPower(1);
        sleep(1000);
        robot.setOutClawPosition(0);
        sleep(700);
        robot.setOutClawPosition(1);
        robot.setVerticalPower(0);
        sleep(500);
        robot.setOutClawPosition(0);
        robot.driveEncoder(.75, -19.75,-19.75,19.75,19.75);
        robot.SetSlidePosition(robot.SLIDE_START);
        sleep(500);
        robot.setIntakePosition(0);
        sleep(250);
        robot.driveEncoder(.75,12,-12,-12,12);
        robot.setHorizontalPosition(0);
        robot.driveEncoder(.75,7.85,7.85,7.85,7.85);
        robot.setHorizontalPosition(0);
        robot.driveEncoder(.5,-3,3,3,-3);
        sleep(750);
        robot.setInClawPosition(1);
        sleep(500);
        robot.setIntakePosition(1);
        robot.setHorizontalPosition(1);
        sleep(800);
        robot.setIntakePosition(1);
        robot.driveEncoder(.75,13,13,-13,-13);
        robot.driveEncoder(.6,-11,-11,-11,-11);
        robot.setOutClawPosition(1);
        sleep(150);
        robot.setInClawPosition(0);
//deposit second sample into basket
        robot.SetSlidePosition(robot.SLIDE_HIGH_BASKET);
        sleep(700);
        robot.setVerticalPower(1);
        sleep(1000);
        robot.setOutClawPosition(0);
        sleep(700);
        robot.setOutClawPosition(1);
        robot.setVerticalPower(0);
        sleep(500);
        robot.setOutClawPosition(0);
        robot.SetSlidePosition(robot.SLIDE_START);
        //go get the third sample
        robot.driveEncoder(.6,-11.5,-11.5,11.5,11.5);
        robot.setIntakePosition(0);
        sleep(500);
        robot.setIntakePosition(0);
        robot.setHorizontalPosition(0);
        sleep(250);

        robot.driveEncoder(.6,7.7,7.7,7.7,7.7);
        robot.driveEncoder(.6,-3.5,3.5,3.5,-3.5);

        robot.setHorizontalPosition(0);
        sleep(750);
        robot.setInClawPosition(1);
        sleep(500);
        robot.setIntakePosition(1);
        robot.setHorizontalPosition(1);
        sleep(800);
        robot.setIntakePosition(1);
        robot.driveEncoder(.75,6,6,-6,-6);
        robot.driveEncoder(.6,-6.3,-6.3,-6.3,-6.3 );
        robot.setOutClawPosition(1);
        sleep(150);
        robot.setInClawPosition(0);
        //deposit third sample into basket
        robot.SetSlidePosition(robot.SLIDE_HIGH_BASKET);
        sleep(700);
        robot.setVerticalPower(1);
        sleep(1000);
        robot.setOutClawPosition(0);
        sleep(700);
        robot.setOutClawPosition(1);
        robot.setVerticalPower(0);
        sleep(500);
        robot.setOutClawPosition(0);
        robot.SetSlidePosition(robot.SLIDE_START);

        robot.driveEncoder(1,50,50,50,50);
    }
}

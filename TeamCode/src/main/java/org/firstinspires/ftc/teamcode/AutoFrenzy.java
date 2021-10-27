package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ShitHead2", group="Pushbot")
public class AutoFrenzy extends LinearOpMode {
    public static final double dPower = 0.75;

    Hardware robot = new Hardware();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        //Do computer vision here
//THE BELOW CODE IS FOR THE CAROUSEL BEING ON THE LEFT OF THE ROBOT
        int x = 1; //get an input of 1,2,3 on where the item is from comp vision
        if(x == 1){
            move(-90,800);
            move(0,500);
            move(90,800);
        }
        else if(x == 2){
            move(90,300);
            move(0, 400);
            move(-90, 300);
        }
        else if(x == 3){
            move(-90,800);
            move(0,500);
            move(90,800);
        }
        //intake
        robot.intake.set(.50);
        move(180,1000);
        sleep(1000);
        robot.intake.set(0);
        //getting to the carousel
        move( 90, 1000); // rotate 90 degrees
        move( 0, 800); //drives to wall
        move(90,1000);//driving parrallel to back wall towards carousel
        move(0,400);
        //spins motor for platform
        robot.carousel.set(.25);
        sleep(1000);
        robot.carousel.set(0);
        //code to drive parallel of the wall towards the parking area
        move(180,200);
        move(-90,2000);
        move(0,500);
        robot.m.driveRobotCentric(0,0,0);

    }
    public void move(double power, double direction, long SLEEP){
        direction = (2*Math.PI*(direction+90)/360);
        robot.m.driveRobotCentric(power*Math.cos(direction),power*Math.sin(direction),0);
        sleep(SLEEP);
        robot.m.driveRobotCentric(0,0,0);
        sleep(100);
    }
    public void move(double direction, long SLEEP){
        move(dPower,direction,SLEEP);
    }
}
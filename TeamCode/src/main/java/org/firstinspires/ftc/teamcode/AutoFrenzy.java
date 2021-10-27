package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ShitHead", group="Pushbot")
public class AutoFrenzy extends LinearOpMode {

    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous: ", "waiting for start");
        telemetry.update();

        waitForStart();

        //Do computer vision here
//THE BELOW CODE IS FOR THE CAROUSEL BEING ON THE LEFT OF THE ROBOT
        int x = 1; //get an input of 1,2,3 on where the item is from comp vision
        if(x == 1){
            move(.75,'r',800);
            move(.75,'f',500);
            move(.75,'l',800);
        }
        else if(x == 2){
            move(.75,'l',300);
            move(.75, 'f', 400);
            move(.75,'r',300);
        }
        else if(x == 3){
            move(.75,'l',800);
            move(.75,'f',500);
            move(.75,'r',800);
        }
        //intake
        robot.intake.set(.50);
        move(.50,'b',1000);
        sleep(1000);
        robot.intake.set(0);
        //getting to the carousel
        move(.75, 'l', 1000); // rotate 90 degrees
        move(.75, 'f', 800); //drives to wall
        move(.75,'l',1000);//driving parrallel to back wall towards carousel
        move(.75,'f',400);
        //spins motor for platform
        robot.carousel.set(.25);
        sleep(1000);
        robot.carousel.set(0);
        //code to drive parallel of the wall towards the parking area
        move(.75,'b',200);
        move(.75,'r',2000);
        move(.75,'f',500);
        motorstop();

    }
    public void motorstop(){
        robot.m0.set(0);
        robot.m1.set(0);
        robot.m2.set(0);
        robot.m3.set(0);
        sleep(100);
    }
    public void move(double power, char direction, long SLEEP){
        switch (direction){
            case 'f':
                robot.m0.set(power);
                robot.m1.set(power);//dirve forward
                robot.m2.set(power);
                robot.m3.set(power);
                sleep(SLEEP);
                break;
            case 'b':
                robot.m0.set(-power);
                robot.m1.set(-power);//reverse
                robot.m2.set(-power);
                robot.m3.set(-power);
                sleep(SLEEP);
                break;
            case 'l':
                robot.m0.set(-power);
                robot.m1.set(-power);
                robot.m2.set(power);//turn left
                robot.m3.set(power);
                sleep(SLEEP);
                break;
            case 'r':
                robot.m0.set(power);
                robot.m1.set(power);//turn right
                robot.m2.set(-power);
                robot.m3.set(-power);
                sleep(SLEEP);
                break;
            case 'x':
                robot.m0.set(1);
                robot.m1.set(.25);
                robot.m2.set(1);//idk what this is, pls tell me Daniel (i assume it is strafing)
                robot.m3.set(.25);
                sleep(SLEEP);
                break;
            case 'y':
                robot.m0.set(.25);
                robot.m1.set(1);
                robot.m2.set(.25);//put comments, u bastard
                robot.m3.set(1);
                sleep(SLEEP);
                break;
        }
        motorstop();
    }
}
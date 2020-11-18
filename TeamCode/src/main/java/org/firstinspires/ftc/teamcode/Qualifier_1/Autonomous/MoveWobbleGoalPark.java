package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "MoveWobbleGoalPark")
public class MoveWobbleGoalPark extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this);
        ElapsedTime runtime = new ElapsedTime();

        waitForStart();

        int i = 1;  // FOR TESTING ONLY

        if (i==4){
            robot.moveForward(121, 0.8);
            sleep(200);
            robot.moveForward(-46, 0.8);
        } else if (i==1) {
            robot.moveForward(80,0.8);
            sleep(200);
            robot.turnOdometry(60,0.8);
            sleep(200);
            robot.turnOdometry(0,0.8); //TODO: -60, or 0
            sleep(200);
            robot.moveForward(-8,0.8);
        } else {
            robot.moveForward(73, 0.8);
        }
    }
}


package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Chassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name = "MoveWobbleGoalPark")
public class MoveWobbleGoalPark extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER);
        ElapsedTime runtime = new ElapsedTime();
        Chassis chassis = new Chassis(this);

        waitForStart();

        int i = 4;  // FOR TESTING ONLY

        if (i==4){
            robot.moveForward(121, 0.8);
            sleep(200);
            chassis.moveForward(109, 0.8);
            sleep(200);
            chassis.turnInPlace(-14, 0.6);
            chassis.moveForward(-44, 0.8);
        } else if (i==1) {
            chassis.turnInPlace(-2,0.8);
            sleep(200);
            chassis.moveForward(80,0.8);
            sleep(200);
            chassis.moveForward(-8,0.8);
        } else {
            chassis.moveForward(6, 0.5);
            sleep(200);
            chassis.turnInPlace(20, 0.6);
            sleep(200);
            chassis.moveForward(65, 0.8);
        }
    }
}


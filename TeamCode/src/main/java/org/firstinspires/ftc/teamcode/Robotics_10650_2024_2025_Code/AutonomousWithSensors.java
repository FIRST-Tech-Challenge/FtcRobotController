package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "AutonomousWithSensors")
public class AutonomousWithSensors extends LinearOpMode {

    RobotInitialize robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotInitialize(this);

        waitForStart(); // Waits for a person to press start on the control hub
        // then it runs the rest of the program

        //forward
        robot.bleft.setVelocity(-400);
        robot.fleft.setVelocity(400);
        robot.fright.setVelocity(400);
        robot.bright.setVelocity(-400);
        sleep(2000);
//        robot.strafeL(1000, 500);
//        sleep(2000);
//        robot.strafeR(1000, 500);
        /*robot.goStraight(920, 250);
        robot.newTurnFunction(-93);
        robot.goStraight(900, 200);
        robot.newTurnFunction(5);
        robot.goStraight(2000, 2000);*/




        // while (opModeIsActive()) {

        //}

        //
        //turn(45);
        //turn(0);
        //Thread.sleep(10000);
        //turn(-20);



        // Not necessary to put power value as negative to go backwards
        //goStraight(-100, 0.3);

        //makeSquare();

//        setMotorVelocity(200);
//        Thread.sleep(5000);
        //robot.makeSquare();

        // Shutdown motors when the code ends
        robot.stopMotors();

    }
}
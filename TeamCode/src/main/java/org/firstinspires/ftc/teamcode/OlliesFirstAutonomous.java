package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="testAuto")
public class OlliesFirstAutonomous extends LinearOpMode {

    RobotClass robot;

    @Override
    public void runOpMode(){

        robot= new RobotClass(hardwareMap, telemetry, this);

        waitForStart();

        try {
            robot.pivotLeft(.5, 90);
          //  robot.turnRight(.5, 90);
        }catch (Exception e){

        }
        try {
            wait(30_000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

    }
}
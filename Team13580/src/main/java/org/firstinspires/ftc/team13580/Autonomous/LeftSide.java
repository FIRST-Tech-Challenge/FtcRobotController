package org.firstinspires.ftc.team13580.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.team13580.RobotHardware;

@Autonomous(name="Left Side", group="Robot")
public class LeftSide extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();
        double heading;
        waitForStart();

        robot.encoderDrive(0.7,24.0,24.0,24.0,24.0, 15);
        robot.encoderArm(60, 10);
        sleep(50);
        robot.encoderDrive(0.5,8,8,8,8,20);
        sleep(100);
        robot.encoderSpoolie(0.2,10,20);

        sleep(100);
        robot.leftHand.setPosition(0);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(300);
        robot.encoderDrive(3.0,-10,-10,-10,-10,15);
        //sleep(100);
        robot.encoderArm(-70 ,15);
        //robot.encoderDrive(5.0,0.4,0.4,-0.4,-0.4 ,20);




        //robot.encoderArm(-70 ,15);
        robot.encoderDrive(0.8,-31.5,31.5,31.5,-31.5, 15);
        robot.encoderDrive(0.8,28.3,28.3,28.3,28.3,15);
        robot.encoderDrive(0.8,4.3,4.3,4.3,4.3,20);
        robot.encoderDrive(0.8, 14,-14,-14,14,20);

    }
}

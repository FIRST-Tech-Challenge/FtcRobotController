package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "BasicAutonomous", group = "teamcode")


public class BasicAutonomous extends LinearOpMode {
    Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException{
        robot.leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        robot.rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        robot.leftBack  = hardwareMap.get(DcMotor.class, "left_back_drive");
        robot.rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");


        waitForStart();

        while (opModeIsActive()){
            moveForward(0.1,1);
            moveForward(0.1,3);
        }



    }
    public void moveForward(double power, long time){

        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightBack.setPower(power);

        sleep(time);
    }
    public void turnRight(double power, long time){

        robot.leftFront.setPower(-power);
        robot. leftBack.setPower(-power);
        robot. rightFront.setPower(power);
        robot.  rightBack.setPower(power);

        sleep(time);
    }
    public void turnLeft(double power, long time){

        robot.leftFront.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightFront.setPower(-power);
        robot. rightBack.setPower(-power);

        sleep(time);
    }

}

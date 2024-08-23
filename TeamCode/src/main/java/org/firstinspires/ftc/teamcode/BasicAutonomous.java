package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "BasicAutonomous", group = "teamcode")


public class BasicAutonomous extends LinearOpMode {
    Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException{
        robot.leftFront e = hardwareMap.get(DcMotor.class, "leftFront");
        robot.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        robot.leftBack  = hardwareMap.get(DcMotor.class, "leftRear");
        robot.rightBack = hardwareMap.get(DcMotor.class, "rightRear");


        waitForStart();

        while (opModeIsActive()){

        }



    }
    public void moveFoward(double power, long time){

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

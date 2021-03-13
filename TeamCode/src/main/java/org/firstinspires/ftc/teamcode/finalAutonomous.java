package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class finalAutonomous extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooter;
    private DcMotor belt;

    //code to play once the OpMode is active
    public void runOpMode() {

        //hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shooter = hardwareMap.get(DcMotor.class, "buzz");
        belt = hardwareMap.get(DcMotor.class, "belt");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        shooter.setPower(.8);
        sleep(1000);
        belt.setPower(.5);
        sleep(5000);
        shooter.setPower(0);
        belt.setPower(0);
        move(.5, 2750);
    }
    public void move(double speed, int time){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        sleep(time);
    }
    public void strafeLeft(){
        frontLeft.setPower(-0.5);
        backLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backRight.setPower(-0.5);
    }
    public void strafeRight(){
        frontLeft.setPower(0.5);
        backLeft.setPower(-0.5);
        frontRight.setPower(-0.5);
        backRight.setPower(0.5);
    }
}

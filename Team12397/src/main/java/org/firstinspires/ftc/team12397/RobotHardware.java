package org.firstinspires.ftc.team12397;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotHardware {

    private LinearOpMode myOpMode = null;

    public ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;


    private DcMotor slideMotorL = null;
    private DcMotor slideMotorR = null;

    private Servo servoA = null;
    private Servo servoB = null;

/**
  Initialize all the robot's hardware.
  This method must be called ONCE when the OpMode is initialized.
**/
    public void init() {
        leftFront = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        leftBack = myOpMode.hardwareMap.get(DcMotor.class, "left_back");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "right_back");

        slideMotorL = myOpMode.hardwareMap.get(DcMotor.class, "slide_motor_left");
        slideMotorR = myOpMode.hardwareMap.get(DcMotor.class, "slide_motor_right");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        slideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

}

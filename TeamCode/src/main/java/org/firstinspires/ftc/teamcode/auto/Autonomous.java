package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (name = "name")
public class Autonomous extends LinearOpMode {

    protected DcMotor left_front;
    protected DcMotor right_front;
    protected DcMotor left_back;
    protected DcMotor right_back;

    @Override
    public void runOpMode() throws InterruptedException {
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        right_back = hardwareMap.get(DcMotor.class, "right-back");

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        right_front.setPower(1);
        left_front.setPower(1);
        right_back.setPower(1);
        left_back.setPower(1);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);
        left_back.setPower(0);
        waitForStart();

        moveForward(1, 1);

    }

    public void moveForward(int power, int time) {
        right_front.setPower(power);
        left_front.setPower(power);
        right_back.setPower(power);
        left_back.setPower(power);
        sleep( time);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);
        left_back.setPower(0);

    }

    public void moveStrafing(int power, int time){
        right_front.setPower(power);
        left_front.setPower(power);
        right_back.setPower(-power);
        left_back.setPower(-power);
        sleep(time);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);
        left_back.setPower(0);

    }

    public void moveTurning(int power, int time){
        right_front.setPower(-power);
        left_front.setPower(power);
        right_back.setPower(-power);
        left_back.setPower(power);
        sleep( time);
        right_front.setPower(0);
        left_front.setPower(0);
        right_back.setPower(0);
        left_back.setPower(0);
    }



   /*
    Strafing:
    lf +
    rf +
    lb -
    rb -
    Turning
    lf +
    rf -
    lb +
    rb -
     */

    //Drive the robot forward
}

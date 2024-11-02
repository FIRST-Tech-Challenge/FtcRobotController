package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@TeleOp(name="rewritten")
public class MecanumDrive extends OpMode {

    //Initializes motor names
    DcMotor flm;
    DcMotor blm;
    DcMotor frm;
    DcMotor brm;

    public void moveRobot(){
        double vertical; // used to move robot forwards and backwards
        double horizontal; // used to move robot left and right
        double pivot;      // used to rotate robot

        vertical = gamepad1.left_stick_y; //if its going forwards instead of backwards change sign
        horizontal = gamepad1.left_stick_x; //if its going left instead of right change sign

        flm.setDirection(DcMotorSimple.Direction.REVERSE);
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        frm.setDirection(DcMotorSimple.Direction.FORWARD);
        brm.setDirection(DcMotorSimple.Direction.FORWARD);

        flm.setPower((-vertical + horizontal) * 0.7);
        blm.setPower((-vertical - horizontal) * 0.7);
        frm.setPower((-vertical - horizontal) * 0.7);
        brm.setPower((-vertical + horizontal) * 0.7);
    }
    public void turnRobot(){
        double pivot;      // used to rotate robot
        pivot = gamepad1.right_stick_x;

        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

        flm.setPower(pivot * 0.7);
        blm.setPower(pivot * 0.7);
        frm.setPower(pivot * 0.7);
        brm.setPower(pivot * 0.7);
    }

    @Override
    public void init(){//initializes hardware map
        flm = hardwareMap.get(DcMotor.class, "flm");     //front left motor
        blm = hardwareMap.get(DcMotor.class, "blm");     //back left motor
        frm = hardwareMap.get(DcMotor.class, "frm");     //front right motor
        brm = hardwareMap.get(DcMotor.class, "brm");     //back right motor

        //reverses specific motors
        blm.setDirection(DcMotorSimple.Direction.REVERSE);
        flm.setDirection(DcMotorSimple.Direction.REVERSE);
        //brm.setDirection(DcMotorSimple.Direction.REVERSE);
        //frm.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void init_loop(){

    }

    @Override public void loop(){
        moveRobot();
    }

}

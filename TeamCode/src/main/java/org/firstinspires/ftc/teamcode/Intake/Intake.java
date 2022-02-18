package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    DcMotor frontIntakeMotor;
    DcMotor backIntakeMotor;
    Servo intakeFlap;
    //Toggle
    Boolean prevInput;
    Boolean itToggle;
    Boolean prevHoldToggle;
    float prevHoldReverse;
    int togg = 1;
    double time;
    double reverseTime;
    float prevHold;
    public Intake(HardwareMap hardwareMap){
        frontIntakeMotor = hardwareMap.dcMotor.get("frontMotorIntake");
        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeFlap = hardwareMap.servo.get("intakeFlapServo");
        backIntakeMotor = hardwareMap.dcMotor.get("backMotorIntake");
        prevInput = false;
        itToggle = false;
        prevHoldToggle = false;
        prevHold = 0;
        prevHoldReverse = 0;
        time = 0;
        reverseTime = 0;
    }


    public void intakeToggle(Telemetry telemetry, Boolean inp) {
        if(inp && !prevInput) {
            itToggle = !itToggle;
            prevInput = true;
            telemetry.addLine("toggle intake");
        }
        prevInput = inp;
        if (itToggle) {
            frontIntakeMotor.setPower(1);
            backIntakeMotor.setPower(1);
            telemetry.addLine("toggle on");
        } else {
            frontIntakeMotor.setPower(0);
            backIntakeMotor.setPower(0);
            telemetry.addLine("toggle off");
        }
    }


    public void intakeHold(Telemetry telemetry, float inp, LinearOpMode op) {
        if (inp > 0) {
            time = op.time;
            frontIntakeMotor.setPower(1);
            backIntakeMotor.setPower(1);
            telemetry.addLine("Pressed(Intake)");

        } else if (inp == 0) {
            frontIntakeMotor.setPower(0);
            time = op.time;

        }

        else if (op.time > time + 2 && inp == 0 && op.time + 2 > reverseTime){
            backIntakeMotor.setPower(0);
            telemetry.addLine("Not Pressed(Intake)");
        }
        prevHold = inp;
        telemetry.addData("time passed", op.time - time );

    }

    public void reverse(Telemetry telemetry, float inp, LinearOpMode op){
        if (inp > 0) {
            reverseTime = op.time;
            frontIntakeMotor.setPower(-1);
            backIntakeMotor.setPower(-1);
            telemetry.addLine("Pressed(Reverse)");

        } else if (inp == 0) {
            frontIntakeMotor.setPower(0);
            reverseTime = op.time;

        }

        else if (op.time > time + 2 && inp == 0 && op.time > 2 + time){
            backIntakeMotor.setPower(0);
            telemetry.addLine("Not Pressed(Reverse)");
        }
        prevHoldReverse = inp;
        telemetry.addData("time passed", op.time - time );
    }


    public void autonomousIntake(int position, int power){
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontIntakeMotor.setTargetPosition(position);
        backIntakeMotor.setTargetPosition(position);
        frontIntakeMotor.setPower(power);
        backIntakeMotor.setPower(power);
    }

    public void flapHalfOpen(Boolean imp){
        if (imp){
            intakeFlap.setPosition(0.4);
        }
    }

    public void flapFullOpen(Boolean imp){
        if (imp){
            intakeFlap.setPosition(0.2);
        }
    }

    public void resetFlap(Boolean imp){
        if (imp){
            intakeFlap.setPosition(1);
        }
    }

}
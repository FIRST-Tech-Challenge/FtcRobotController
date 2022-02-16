package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor frontIntakeMotor;
    DcMotor backIntakeMotor;

    //Toggle
    Boolean prevInput;
    Boolean itToggle;
    Boolean prevHoldToggle;
    public Intake(HardwareMap hardwareMap){
        frontIntakeMotor = hardwareMap.dcMotor.get("frontMotorIntake");
        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backIntakeMotor = hardwareMap.dcMotor.get("backMotorIntake");
        prevInput = false;
        itToggle = false;
        prevHoldToggle = false;
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




    public void intakeHold(Boolean inp, LinearOpMode op) {
        if (inp) {
            frontIntakeMotor.setPower(1);
            backIntakeMotor.setPower(1);
        } else if (prevHoldToggle == true && inp == false) {
            frontIntakeMotor.setPower(0);
            op.sleep(2000);
            backIntakeMotor.setPower(0);
        }
        prevHoldToggle = inp;


    }

    public void autonomousIntake(int position, int power){
        frontIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontIntakeMotor.setTargetPosition(position);
        backIntakeMotor.setTargetPosition(position);
        frontIntakeMotor.setPower(power);
        backIntakeMotor.setPower(power);
    }




}
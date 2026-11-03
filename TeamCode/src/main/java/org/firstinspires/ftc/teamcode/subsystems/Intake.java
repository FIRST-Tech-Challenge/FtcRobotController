package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake{
    DcMotor armMotor;
    DcMotor intakeMotor;

    public Intake(DcMotor armMotor) {
    }

    public void init(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ArmIntakeIn(){
        armMotor.setPower(1);
    }

    public void ArmIntakeOut(){
        armMotor.setPower(-1);
    }

}

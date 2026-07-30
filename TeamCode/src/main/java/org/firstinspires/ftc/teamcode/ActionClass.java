package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActionClass {
    DcMotor Intake;
    public ActionClass(HardwareMap hardwareMap){
        Intake = hardwareMap.get(DcMotor.class,"Intake");
    }

    public void Intake_On(){
        Intake.setPower(1);
    }

    public void Intake_Off(){
        Intake.setPower(0);
    }

    public void Intake_R(){
        Intake.setPower(-1);
    }
}

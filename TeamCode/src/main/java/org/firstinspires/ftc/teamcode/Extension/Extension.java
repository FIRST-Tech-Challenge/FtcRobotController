package org.firstinspires.ftc.teamcode.Extension;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Extension {
    DcMotorEx motor;
    HardwareMap hardwareMap;

    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }
}

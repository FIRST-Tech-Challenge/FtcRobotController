package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Intake {
    private CRServo Intake;

    public Intake(HardwareMap hardwareMap) {
        this.Intake = hardwareMap.get(CRServo.class, "Intake");
        this.Intake.setDirection(CRServo.Direction.FORWARD);
    }
    public void setPower(double power) {
        Intake.setPower(power);
    }
}
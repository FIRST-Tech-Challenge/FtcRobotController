package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Deprecated
public class ManualMotor {
    public String name;
    public double speedMultiplier;

    private final HardwareMap hardware;
    private final DcMotor DCmotor;

    ManualMotor(String name, double speedMultiplier, HardwareMap hardware) {
        this.name = name;
        this.speedMultiplier = speedMultiplier;

        this.hardware = hardware;

        DCmotor = hardware.dcMotor.get(name);
    }

    ManualMotor(String name, HardwareMap hardware) {
        this.name = name;
        this.speedMultiplier = 1;

        this.hardware = hardware;

        DCmotor = hardware.dcMotor.get(name);
    }

    public void SetPower(double power) { DCmotor.setPower(power); }
    public void SetPower(int power) { SetPower(power / 100); }
}

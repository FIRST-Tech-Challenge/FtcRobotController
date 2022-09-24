package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide extends SubsystemBase {
    private HardwareMap hw;
    private String name;
    private MotorEx slideMotor;

    public LinearSlide(final HardwareMap hw, final String name){
        this.hw = hw;
        this.name = name;
        slideMotor = new MotorEx(hw, "slideMotor");
    }

    @Override
    public void periodic(){

    }
}

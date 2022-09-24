package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class Drivetrain extends SubsystemBase {

    private DcMotorEx frontR;
    private DcMotorEx frontL;
    private DcMotorEx backR;
    private DcMotorEx backL;
    private List<DcMotorEx> motors;
    private HardwareMap hw;


    public Drivetrain(HardwareMap hw) {
        this.hw = hw;
        DcMotorEx[] motorArr = {frontL, frontR, backR, backL};
        motors = Arrays.asList(motorArr);

        // Initialize all motors with the name of their variable
        for (int i = 0; i < motorArr.length; i++)
            motorArr[i] = hw.get(DcMotorEx.class, motorArr[i].getClass().getSimpleName());
    }

    @Override
    public void periodic() {

    }


}

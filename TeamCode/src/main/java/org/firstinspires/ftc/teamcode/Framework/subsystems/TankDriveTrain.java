package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class TankDriveTrain extends SubsystemBase {

    private DcMotorEx frontR;
    private DcMotorEx frontL;
    private DcMotorEx backR;
    private DcMotorEx backL;

    private List<DcMotorEx> motors;

    public TankDriveTrain(HardwareMap hw){
        frontR = hw.get(DcMotorEx.class, "frontR");
        backR = hw.get(DcMotorEx.class, "backR");
        frontL = hw.get(DcMotorEx.class, "frontL");
        backL = hw.get(DcMotorEx.class, "backL");

//        frontR.setDirection(DcMotorSimple.Direction.REVERSE);
//        backR.setDirection(DcMotorSimple.Direction.REVERSE);

//        motors.add(frontR);
//        motors.add(frontL);
//        motors.add(backR);
//        motors.add(backL);
//
//        for(DcMotorEx motor : motors) motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    public void setPower(double left, double right){
        frontR.setPower(right);
        backR.setPower(right);
        backL.setPower(left);
        frontL.setPower(left);
    }

}

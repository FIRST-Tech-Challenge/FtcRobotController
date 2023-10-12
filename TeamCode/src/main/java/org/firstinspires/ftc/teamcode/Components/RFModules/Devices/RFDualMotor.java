package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RFDualMotor extends RFMotor{
    DcMotorEx motor2;
    public RFDualMotor(String p_motorName, String p_motorName2, boolean p_resetPos) {
        super(p_motorName, p_resetPos);
        motor2 = op.hardwareMap.get(DcMotorEx.class, p_motorName2);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setRawPower(double p_power){
        super.setRawPower(p_power);
        motor2.setPower(p_power);
    }
    public void setDirection2(DcMotorSimple.Direction p_direction){
        motor2.setDirection(p_direction);
    }

    public void setPower(double p_power){
        super.setPower(p_power);
        motor2.setPower(super.getPower());
    }
}

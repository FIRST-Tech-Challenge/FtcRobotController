package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmMotorV1 {
    public HardwareMap hwMap;
    public DcMotor arm;
    public String name_;
    public int position;

    public ArmMotorV1(HardwareMap hardwareMap, String configName){
        this.hwMap = hardwareMap;
        this.name_ = configName;
    }

    public void init(){
        arm = hwMap.get(DcMotor.class, name_);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        position = arm.getCurrentPosition();
    }

    public int getPos(){
        return position;
    }

    public void setPower(double power){
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(power);
        position = arm.getCurrentPosition();
    }

    public void moveTo(int pos, double power, boolean wait){
        double current_position = arm.getCurrentPosition();
        if (current_position > position){
            setPower(-power);
            arm.setTargetPosition(pos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (current_position < position){
            setPower(power);
            arm.setTargetPosition(pos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        position = pos;
    }
    public void hold(double power){
        moveTo(position, power, false);
    }
}

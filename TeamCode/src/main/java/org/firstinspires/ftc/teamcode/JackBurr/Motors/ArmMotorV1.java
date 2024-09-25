package org.firstinspires.ftc.teamcode.JackBurr.Motors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmMotorV1 {
    public HardwareMap hwMap;
    public DcMotor arm;
    public String name_;
    public Telemetry telem_;
    public int position;

    public ArmMotorV1(HardwareMap hardwareMap, String configName, Telemetry telem){
        this.hwMap = hardwareMap;
        this.name_ = configName;
        this.telem_ = telem;
    }

    public DcMotor init(HardwareMap hwMap){
        telem_.addLine(name_);
        arm =  hwMap.get(DcMotor.class, name_);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        position = arm.getCurrentPosition();
        this.hwMap = hwMap;
        return arm;
    }

    public int get_encoder_pos(){
        return arm.getCurrentPosition();
    }

    public int get_target_position(){
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

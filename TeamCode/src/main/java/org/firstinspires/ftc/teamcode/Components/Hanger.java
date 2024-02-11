package org.firstinspires.ftc.teamcode.Components;


import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
@Config
public class Hanger extends RFMotor {
    double permaPower = 0.0;
    boolean up = false;
    public static double max = 6000,
            min = -15,
            RESISTANCE = 400,
            kS = 0.3,
            kV = 9.2786E-3,
            kA = 9E-3,
            MAX_UP_VELO = 2000,
            MAX_DOWN_VELO = -1080,
            MAX_ACCEL = 10000,
            MAX_DECEL = -10000,
            kP = 0,
            kD = 0;
//    DcMotorEx encoder;
    public Hanger(){
        super("hangerMotor", !isTeleop);
        super.setDirection(DcMotorSimple.Direction.REVERSE);
        setConstants(
                max, min, RESISTANCE, kS, kV, kA, MAX_UP_VELO, MAX_DOWN_VELO, MAX_ACCEL, MAX_DECEL, kP, kD);
        super.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        encoder = op.hardwareMap.get(DcMotorEx.class, "motorLeftBack");
        if(!isTeleop){
//            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void setPermaPower(double p_permaPower){
        permaPower=signum(p_permaPower)*min(abs(p_permaPower), 0.3);
        up = false;
    }
    @Override
    public void setPower(double p_power){
    if (this.getCurrentPosition() < max && this.getCurrentPosition()>0) {
            setRawPower(-p_power + permaPower);
        }
    else if (getCurrentPosition()>max){
        setRawPower(min(0,-p_power));
    }
    else{
        setRawPower(max(0,-p_power));
    }
    if (abs(p_power) > 0.1) {
      up = false;
    }
    }
    public void up(){
        this.setPosition(max,0);
        up = true;
    }
    public void update(){

        if (up) {
            setPosition(max, 0);
        } else {
            setTarget(super.getCurrentPosition());
        }
//        this.setCurrentPosition(encoder.getCurrentPosition());
//        this.setCurrentVelocity(encoder.getVelocity());
        packet.put("hangPos", super.getCurrentPosition());
        packet.put("hangerPower", super.getPower());
        packet.put("upping",up);
    }
}

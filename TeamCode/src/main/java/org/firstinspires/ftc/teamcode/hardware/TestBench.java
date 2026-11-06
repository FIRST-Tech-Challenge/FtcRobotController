package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TestBench {



    private DcMotorEx motor0;


    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    public double ticksPerRev;

    public void init(HardwareMap haMap){
        motor0 = haMap.get(DcMotorEx.class, "motor0");
        motor1 = haMap.get(DcMotorEx.class, "motor1");
        motor2 = haMap.get(DcMotorEx.class, "motor2");
        motor3 = haMap.get(DcMotorEx.class, "motor3");
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ticksPerRev = motor3.getMotorType().getTicksPerRev();


    }

    public void setMotorSpeed(double speed){

//        motor3.setVelocity(speed);
        motor3.setPower(speed);
        //motor3.setVelocity(1500, AngleUnit.DEGREES);
        //motor3.setVelocity(speed);
    }

    public double getMotorRev(){
        return motor3.getCurrentPosition() / ticksPerRev;  //normalizing ticks to revolutions
    }
    public DcMotorEx getMotor0() {
        return motor0;
    }
    public DcMotorEx getMotor1() {
        return motor1;
    }

    public DcMotorEx getMotor2() {
        return motor2;
    }

    public DcMotorEx getMotor3() {
        return motor3;
    }

    public void setMotor0(DcMotorEx motor3) {
        this.motor0 = motor0;
    }
    public void setMotor1(DcMotorEx motor3) {
        this.motor1 = motor1;
    }
    public void setMotor2(DcMotorEx motor3) {
        this.motor2 = motor2;
    }
    public void setMotor3(DcMotorEx motor3) {
        this.motor3 = motor3;
    }

}

package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Usefuls.Math.M;
import org.firstinspires.ftc.teamcode.Usefuls.Motor.PID;


@Config
public class Arm {


    private static final double TICKS_TO_DEGREES = 0.1150214;
    public static double targetArmPosition = 100;
    public static double sKp = 0.075, sKi = 0, sKd = 0.003;
    public static double powerA = 1, powerB = -1;
    private final DcMotorEx a;
    private final DcMotorEx armEncoder;
    private final PID armController;
    private double error;


    public Arm(HardwareMap hardwareMap, DcMotorEx armEncoder) {
        a = hardwareMap.get(DcMotorEx.class, "arm");
        this.armEncoder = armEncoder;
        this.armController = new PID(new PID.Coefficients(sKp, sKi, sKd), () -> (this.getCurrentArmPosition()) - targetArmPosition, factor -> {
            this.a.setPower(M.clamp(factor, powerA, powerB)); //b is down
        });

    }

    public void resetEncoder() {
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getError() {
        return getCurrentArmPosition() - targetArmPosition;
    }

    public double getCurrentArmPosition() {
        return (this.armEncoder.getCurrentPosition() * TICKS_TO_DEGREES) + 5.55;
//        return this.armEncoder.getCurrentPosition();
    }

    public void setTargetArmPosition(double position) {
        targetArmPosition = position;
    }

    public void deposit() {
        setTargetArmPosition(95);
    }

    public void preTake() {
        setTargetArmPosition(5);
    }

    public void preSubmerse() {
        setTargetArmPosition(10);
    }

    public void holdSpecimen() { setTargetArmPosition(40);}

    public void intake() {
        setTargetArmPosition(0);
    }

    public void specimenIntake() {
        setTargetArmPosition(3);
    }

    public void update() {
        armController.update();
    }

}
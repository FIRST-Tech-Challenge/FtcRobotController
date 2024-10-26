package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Usefuls.Math.M;
import org.firstinspires.ftc.teamcode.Usefuls.Motor.PID;


@Config
public class Arm {


    private static final double TICKS_TO_DEGREES = 0.1602564102564103;
    public static double targetArmPosition = 100;
    public static double sKp = 0.07, sKi = 0, sKd = 0.004;
    private double error;
    private final DcMotorEx a;
    private final DcMotorEx armEncoder;
    private PID armController;


    public Arm(HardwareMap hardwareMap, DcMotorEx armEncoder) {
        a = hardwareMap.get(DcMotorEx.class, "arm");
        this.armEncoder = armEncoder;
        this.armController = new PID(new PID.Coefficients(sKp, sKi, sKd),
                () -> (this.getCurrentArmPosition()) - targetArmPosition,
                factor -> {
                    this.a.setPower(M.clamp(factor, 0.7, -0.7)); //b is down
                });

    }

    public Arm(HardwareMap hardwareMap, DcMotorEx armEncoder, String armName) {
        a = hardwareMap.get(DcMotorEx.class, armName);
        this.armEncoder = armEncoder;

    }

    public void resetEncoder() {
        armEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getError() {
        return error;
    }

    public double getCurrentArmPosition() {
        return (this.armEncoder.getCurrentPosition() * TICKS_TO_DEGREES)+100;
    }


    /**
     * sets the position of the arm in degrees
     *
     * @param position the position in degrees
     */
    public void setTargetSlidesPosition(double position) {
        targetArmPosition = position;
    }

    public void update() {
        armController.update();
    }

}
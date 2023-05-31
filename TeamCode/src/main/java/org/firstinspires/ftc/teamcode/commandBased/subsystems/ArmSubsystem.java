package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class ArmSubsystem extends SubsystemBase {

    private Motor arm;
    private final DcMotor m_arm;

    private double armPos;
    private double armTarget;
    private double armPower;


    private double KP = 0;
    private double KI = 0;
    private double KD = 0;

    private final InterpLUT pCoefficients;
    private final InterpLUT iCoefficients;
    private final InterpLUT dCoefficients;
    private final PIDController pid;

    private final InterpLUT angleLUT;
    private double KF;
    private final PIDFController controller;


    public ArmSubsystem(final HardwareMap hwMap) {

        //motor setup
        arm = new Motor(hwMap, "arm", Constants.ARM_MOTOR);

        m_arm = arm.motor;

        m_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setRunMode(Motor.RunMode.RawPower);

        //interpLUT KF setup
        angleLUT = new InterpLUT();
        angleLUT.add(-100, -90);
        angleLUT.add(-50,-45);
        angleLUT.add(0, 0);
        angleLUT.add(50, 45);
        angleLUT.add(100, 90);
        angleLUT.createLUT();

        //pidf controller setup
        controller = new PIDFController(
                Constants.ARM_KP,
                Constants.ARM_KI,
                Constants.ARM_KD,
                KF
        );


        //interplut pid setup
        pCoefficients = new InterpLUT();
        iCoefficients = new InterpLUT();
        dCoefficients = new InterpLUT();

        pCoefficients.add(-Constants.ARM_MAX, -Constants.ARM_KP_90);
        pCoefficients.add(0, Constants.ARM_KP_0);
        pCoefficients.add(Constants.ARM_MAX, Constants.ARM_KP_90);

        iCoefficients.add(-Constants.ARM_MAX, -Constants.ARM_KI_90);
        iCoefficients.add(0, Constants.ARM_KI_0);
        iCoefficients.add(Constants.ARM_MAX, Constants.ARM_KI_90);

        dCoefficients.add(-Constants.ARM_MAX, -Constants.ARM_KD_90);
        dCoefficients.add(0, Constants.ARM_KD_0);
        dCoefficients.add(Constants.ARM_MAX, Constants.ARM_KD_90);

        pid = new PIDController(KP, KI, KD);
    }

    @Override
    public void periodic() {
        armPos = m_arm.getCurrentPosition();
        calculateKF();
        PIDF();
        //calculatePID();
        //PID();
    }

    public void PIDF() {
        armPower = controller.calculate(armTarget, armPos);
        arm.set(armPower);
    }

    public void calculateKF() {
        KF = Math.cos(angleLUT.get(m_arm.getCurrentPosition())) * Constants.ARM_KCOS;
    }

    public void PID() {
        armPower = pid.calculate(armTarget, armPos);
        arm.set(armPower);
    }

    public void calculatePID() {
        KP = pCoefficients.get(m_arm.getCurrentPosition());
        KI = iCoefficients.get(m_arm.getCurrentPosition());
        KD = dCoefficients.get(m_arm.getCurrentPosition());
    }

    public void setArmAngle(double target) {
        armTarget = angleLUT.get(target);
    }
}

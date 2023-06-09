package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class ArmSubsystem extends SubsystemBase {

    private Motor arm;
    private final DcMotor m_arm;

    private double armPos;
    private double armTarget = 0;
    private double armPower;
    private double armAngle;


    private static double KP;
    private static double KI;
    private static double KD;

    private double KF;
    private double KCOS;

    //private final PIDController pid;

    private final PIDFController controller;
    private MotionProfile profile;
    private final ElapsedTime timer;
    private MotionState state;
    private double correction;

    private final InterpLUT angleEncLUT;
    private final InterpLUT encAngleLUT;
    private final InterpLUT pCoefficients;
    private final InterpLUT iCoefficients;
    private final InterpLUT dCoefficients;


    public ArmSubsystem(final HardwareMap hwMap) {

        //motor setup
        arm = new Motor(hwMap, "arm", Constants.ARM_MOTOR);

        m_arm = arm.motor;

        m_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setRunMode(Motor.RunMode.RawPower);

        //angle setup
        angleEncLUT = new InterpLUT();
        angleEncLUT.add(-Constants.ARM_ANGLE_MAX, Constants.ARM_ENC_BACK_MAX);
        angleEncLUT.add(-90, Constants.ARM_ENC_BACK_PARALLEL);
        angleEncLUT.add(0,Constants.ARM_ENC_CENTER);
        angleEncLUT.add(90, Constants.ARM_ENC_FRONT_PARALLEL);
        angleEncLUT.add(Constants.ARM_ANGLE_MAX, Constants.ARM_ENC_FRONT_MAX);
        angleEncLUT.createLUT();

        encAngleLUT = new InterpLUT();
        encAngleLUT.add(Constants.ARM_ENC_BACK_MAX, -Constants.ARM_ANGLE_MAX);
        encAngleLUT.add(Constants.ARM_ENC_BACK_PARALLEL, -90);
        encAngleLUT.add(Constants.ARM_ENC_CENTER, 0);
        encAngleLUT.add(Constants.ARM_ENC_FRONT_PARALLEL, 90);
        encAngleLUT.add(Constants.ARM_ENC_FRONT_MAX, Constants.ARM_ANGLE_MAX);
        encAngleLUT.createLUT();


        //interplut pid setup
        pCoefficients = new InterpLUT();
        iCoefficients = new InterpLUT();
        dCoefficients = new InterpLUT();

        pCoefficients.add(Constants.ARM_ENC_BACK_MAX - Constants.ARM_ENC_SAFETY, Constants.ARM_KP_MAX);
        pCoefficients.add(Constants.ARM_ENC_BACK_PARALLEL, Constants.ARM_KP_90);
        pCoefficients.add(Constants.ARM_ENC_CENTER, Constants.ARM_KP_0);
        pCoefficients.add(Constants.ARM_ENC_FRONT_PARALLEL, Constants.ARM_KP_90);
        pCoefficients.add(Constants.ARM_ENC_FRONT_MAX + Constants.ARM_ENC_SAFETY, Constants.ARM_KP_MAX);
        pCoefficients.createLUT();

        iCoefficients.add(Constants.ARM_ENC_BACK_MAX - Constants.ARM_ENC_SAFETY, Constants.ARM_KI_MAX);
        iCoefficients.add(Constants.ARM_ENC_BACK_PARALLEL, Constants.ARM_KI_90);
        iCoefficients.add(Constants.ARM_ENC_CENTER, Constants.ARM_KI_0);
        iCoefficients.add(Constants.ARM_ENC_FRONT_PARALLEL, Constants.ARM_KI_90);
        iCoefficients.add(Constants.ARM_ENC_FRONT_MAX + Constants.ARM_ENC_SAFETY, Constants.ARM_KI_MAX);
        iCoefficients.createLUT();

        dCoefficients.add(Constants.ARM_ENC_BACK_MAX - Constants.ARM_ENC_SAFETY, Constants.ARM_KD_MAX);
        dCoefficients.add(Constants.ARM_ENC_BACK_PARALLEL, Constants.ARM_KD_90);
        dCoefficients.add(Constants.ARM_ENC_CENTER, Constants.ARM_KD_0);
        dCoefficients.add(Constants.ARM_ENC_FRONT_PARALLEL, Constants.ARM_KD_90);
        dCoefficients.add(Constants.ARM_ENC_FRONT_MAX + Constants.ARM_ENC_SAFETY, Constants.ARM_KD_MAX);
        dCoefficients.createLUT();

        //motion profile setup
        controller = new PIDFController(
                Constants.ARM_COEFFS,
                Constants.ARM_KV,
                Constants.ARM_KA,
                Constants.ARM_KS
        );

        timer = new ElapsedTime();

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(0, 0, 0),
                Constants.ARM_MAX_VEL,
                Constants.ARM_MAX_ACCEL
        );

        state = profile.get(timer.seconds());

        //pid = new PIDController(KP, KI, KD);
    }

    @Override
    public void periodic() {
        setArmPos();
        motionProfiling();
        calculateKF();
        PIDF();
    }

//    public void calculatePID() {
//        KP = pCoefficients.get(armPos);
//        KI = iCoefficients.get(armPos);
//        KD = dCoefficients.get(armPos);
//        pid.setPID(KP, KI, KD);
//    }

    public void motionProfiling() {
        state = profile.get(timer.seconds());
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
    }

    public void calculateKF() {
        armAngle = encAngleLUT.get(armPos);
        KF = getSIN(armAngle) * Constants.ARM_KSIN;
    }

    public void PIDF() {
        correction = -controller.update(armPos);
        arm.set(correction - KF);
    }

    private double getSIN(double angle) {
        if (angle >= 90) {
            angle -= 90;
        } else if (angle <= -90) {
            angle += 90;
        }
        return Math.sin(Math.toRadians(angle));
    }

    public void setArmAngle(double target) {
        armTarget = angleEncLUT.get(target);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(state.getX(), 0, 0),
                new MotionState(armTarget, 0, 0),
                Constants.ARM_MAX_VEL,
                Constants.ARM_MAX_ACCEL
        );
        timer.reset();
    }

    public double getArmAngle() {
        return armAngle;
    }

    public double getArmTarget() {
        return state.getX();
    }

    public void setArmPos() {
        armPos = m_arm.getCurrentPosition();
        armPos = -armPos;
    }

    public double getArmPos() {
        return armPos;
    }

    public double getArmPower() {
        return correction - KF;
    }

    public double[] getCoeffs() {
        return new double[]{KP, KI, KD, KF};
    }
}

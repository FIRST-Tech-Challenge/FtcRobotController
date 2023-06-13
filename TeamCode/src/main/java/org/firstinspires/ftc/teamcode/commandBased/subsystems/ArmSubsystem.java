package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBased.Constants;

public class ArmSubsystem extends SubsystemBase {

    private final Motor arm;
    private final DcMotor m_arm;

    private double armPos;
    private double armTarget = 0;
    private double armAngle;

    private double KF;

    private final PIDFController controller;
    private MotionProfile profile;
    private final ElapsedTime timer;
    private MotionState state;
    private double correction;

    private final InterpLUT angleEncLUT;
    private final InterpLUT encAngleLUT;

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
    }

    @Override
    public void periodic() {
        setArmPos();
        motionProfiling();
        calculateKF();
        PIDF();
    }

    private void motionProfiling() {
        state = profile.get(timer.seconds());
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
    }

    private void calculateKF() {
        armAngle = encAngleLUT.get(armPos);
        KF = getSIN(armAngle) * Constants.ARM_KSIN;
    }

    private void PIDF() {
        correction = -controller.update(armPos);
        arm.set(correction - KF);
    }

    private double getSIN(double angle) {
        if (angle >= 90) {
            angle = 180 - angle;
        } else if (angle <= -90) {
            angle = -180 - angle;
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

    public void setArmPos() {
        armPos = m_arm.getCurrentPosition();
        armPos = -armPos;
    }

    //getters
    public double getArmAngle() {
        return armAngle;
    }

    public double getArmTarget() {
        return state.getX();
    }

    public double getArmPos() {
        return armPos;
    }

    public double getArmPower() {
        return correction - KF;
    }

    public double getArmVelocity() {
        return state.getV();
    }

    public double getArmAcceleration() {
        return state.getA();
    }

    public double[] getCoeffs() {
        return new double[]{
                Constants.ARM_COEFFS.kP,
                Constants.ARM_COEFFS.kI,
                Constants.ARM_COEFFS.kD,
                Constants.ARM_KV,
                Constants.ARM_KA,
                Constants.ARM_KS,
                Constants.ARM_KSIN
        };
    }
}

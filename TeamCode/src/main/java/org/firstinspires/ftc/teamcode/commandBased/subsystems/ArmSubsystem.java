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
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.Positions;

public class ArmSubsystem extends SubsystemBase {

    private final Motor arm;
    private final DcMotor m_arm;

    private double armPos;
    private double armTargetEnc = 20;
    private double armTargetAngle = Constants.ARM_ANGLE_IDLE;
    private double armAngle;
    private double armEncOffset;

    private double KF;

    private boolean disabled = false;

    private PIDFController controller;
    private MotionProfile profile;
    private final ElapsedTime timer;
    private MotionState state;
    private double correction;

    private final InterpLUT angleEncLUT;
    private final InterpLUT encAngleLUT;

    public ArmSubsystem(final HardwareMap hwMap) {

        VoltageSensor batteryVoltageSensor = hwMap.voltageSensor.iterator().next();

        //motor setup
        arm = new Motor(hwMap, "arm", Constants.ARM_MOTOR);

        m_arm = arm.motor;

        m_arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_arm.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setRunMode(Motor.RunMode.RawPower);

        armEncOffset = Positions.armPosition;

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
                new MotionState(angleEncLUT.get(Constants.ARM_ANGLE_IDLE), 0, 0),
                100,
                200
        );

        state = profile.get(timer.seconds());
    }

    public boolean isFinished() {
        return (armTargetAngle >= armAngle - Constants.ARM_ANGLE_DEADZONE &&
                armTargetAngle <= armAngle + Constants.ARM_ANGLE_DEADZONE);
    }

    @Override
    public void periodic() {
        setArmPos();
        motionProfiling();
        calculateKF();
        armPower();
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
        correction = controller.update(armPos);
        if (!disabled) {
            arm.set(correction + KF);
        } else {
            arm.set(0);
        }
    }

    private void armPower() {
        if (disabled) {
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        } else {
            arm.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void addOffset(double offset) {
        Positions.armPosition += offset;
    }

    public void resetEncoder() {
        Positions.armPosition = -(getArmPos());
    }

    private double getSIN(double angle) {
        if (angle >= 90) {
            angle = 180 - angle;
        } else if (angle <= -90) {
            angle = -180 - angle;
        }
        return Math.sin(Math.toRadians(angle));
    }

    public void createNewController() {
        controller = new PIDFController(
                Constants.ARM_COEFFS,
                Constants.ARM_KV,
                Constants.ARM_KA,
                Constants.ARM_KS
        );
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
    }

    public void setArmAngle(double target, double velo, double accel) {
        armTargetEnc = angleEncLUT.get(target);
        armTargetAngle = target;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(state.getX(), 0, 0),
                new MotionState(armTargetEnc, 0, 0),
                velo,
                accel
        );
        timer.reset();
    }

    public void setArmPos() {
        armPos = getOffsetPos(-m_arm.getCurrentPosition());
    }

    private double getOffsetPos(double pos) {
        return pos + Positions.armPosition;
    }

    //getters
    public double getArmAngle() {
        return armAngle;
    }

    public double getArmProfileTarget() {
        return state.getX();
    }

    public double getArmTargetEnc() {
        return armTargetEnc;
    }

    public double getArmTargetAngle() {
        return armTargetAngle;
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

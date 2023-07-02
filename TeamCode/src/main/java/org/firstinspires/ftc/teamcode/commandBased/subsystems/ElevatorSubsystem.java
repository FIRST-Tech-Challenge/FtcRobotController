package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.opmodes.Positions;


public class ElevatorSubsystem extends SubsystemBase {

    private final Motor eleL;
    private final Motor eleR;
    private final DcMotor m_eleL;
    private final DcMotor m_eleR;
    private final MotorGroup ele;

    //ELEVATOR VARIABLES
    private double elePos;
    private double eleTarget;
    private double eleEncOffset;

    private PIDFController controller;
    private MotionProfile profile;
    private final ElapsedTime timer;
    private MotionState state;
    private double correction;

    public ElevatorSubsystem(final HardwareMap hwmap){

        //motor setup
        eleL = new Motor(hwmap, "eleL", Constants.ELE_MOTOR);
        eleR = new Motor(hwmap, "eleR", Constants.ELE_MOTOR);

        m_eleL = eleL.motor;
        m_eleR = eleR.motor;

        m_eleL.setDirection(DcMotorSimple.Direction.REVERSE);

        m_eleL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_eleR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_eleL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_eleR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_eleL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_eleR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ele = new MotorGroup(eleL, eleR);
        ele.setRunMode(Motor.RunMode.RawPower);

        eleEncOffset = Positions.elePosition;

        //pid controller and motion profile setup
        controller = new PIDFController(
                Constants.ELE_COEFFS,
                Constants.ELE_KV,
                Constants.ELE_KA,
                Constants.ELE_KS
        );
        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(0, 0, 0),
                inchesToTicks(Constants.ELE_MAX_VEL),
                inchesToTicks(Constants.ELE_MAX_ACCEL)
        );
        state = profile.get(timer.seconds());
    }

    @Override
    public void periodic() {
        elePos = getOffsetPos((m_eleL.getCurrentPosition() + m_eleR.getCurrentPosition()) / 2.0);

        //motion profiling
        state = profile.get(timer.seconds());

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        correction = controller.update(elePos);
        ele.set(correction + Constants.ELE_KG);
    }

    private double getOffsetPos(double pos) {
        return pos + eleEncOffset;
    }

    public void createNewController() {
        controller = new PIDFController(
                Constants.ELE_COEFFS,
                Constants.ELE_KV,
                Constants.ELE_KA,
                Constants.ELE_KS
        );
        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());
    }

    public void setProfileTarget(double target) {
        eleTarget = inchesToTicks(target);
        target = eleTarget;
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(state.getX(), 0, 0),
                new MotionState(target, 0, 0),
                inchesToTicks(Constants.ELE_MAX_VEL),
                inchesToTicks(Constants.ELE_MAX_ACCEL)
        );
        timer.reset();
    }

    public void moveProfileTarget(double amount) {
        setProfileTarget(getElePos() + amount);
    }

    public boolean isFinished() {
        return eleTarget >= elePos - inchesToTicks(Constants.ELE_DONE_DEADZONE)
            && eleTarget <= elePos + inchesToTicks(Constants.ELE_DONE_DEADZONE);
    }

    public double getElePower() {
        return (correction + Constants.ELE_KG);
    }

    public double getEleTarget() {
        return ticksToInches(eleTarget);
    }

    public double getEleProfileTarget() {
        return ticksToInches(state.getX());
    }

    public double getElePos() {
        return ticksToInches(elePos);
    }

    public double inchesToTicks(double inches) {
        return inches * 79.0426072601;
    }

    public double ticksToInches(double ticks) {
        return ticks / 79.0426072601;
    }
}

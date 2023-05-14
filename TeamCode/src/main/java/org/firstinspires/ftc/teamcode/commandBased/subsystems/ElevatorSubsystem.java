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

import org.firstinspires.ftc.teamcode.commandBased.Constants;


public class ElevatorSubsystem extends SubsystemBase {

    private Motor eleL;
    private Motor eleR;
    private final DcMotor m_eleL;
    private final DcMotor m_eleR;
    private final MotorGroup ele;

    //ELEVATOR VARIABLES
    private double elePos = 0;

    private final PIDFController controller;
    private MotionProfile profile;
    private final ElapsedTime timer;
    private MotionState state;
    private double correction;

    public ElevatorSubsystem(final HardwareMap hwmap){

        //motor setup
        eleL = new Motor(hwmap, "eleL", Motor.GoBILDA.RPM_312);
        eleR = new Motor(hwmap, "eleR", Motor.GoBILDA.RPM_312);

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
        elePos = (m_eleL.getCurrentPosition() + m_eleR.getCurrentPosition()) / 2.0;

        //motion profiling
        state = profile.get(timer.seconds());

        controller.setTargetPosition(state.getX());
        controller.setTargetVelocity(state.getV());
        controller.setTargetAcceleration(state.getA());

        correction = controller.update(elePos);
        ele.set(correction + Constants.ELE_KG);
    }

    public void setProfileTarget(double target) {
        target = inchesToTicks(target);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(state.getX(), 0, 0),
                new MotionState(target, 0, 0),
                inchesToTicks(Constants.ELE_MAX_VEL),
                inchesToTicks(Constants.ELE_MAX_ACCEL)
        );
        timer.reset();
    }

    public double getElePower() {
        return (correction + Constants.ELE_KG);
    }

    public double getEleTarget() {
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

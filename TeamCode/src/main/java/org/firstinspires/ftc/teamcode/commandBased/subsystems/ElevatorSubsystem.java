package org.firstinspires.ftc.teamcode.commandBased.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ElevatorSubsystem extends SubsystemBase {

    private Motor eleL;
    private Motor eleR;
    private final DcMotor m_eleL;
    private final DcMotor m_eleR;
    private final MotorGroup ele;

    //ELEVATOR VARIABLES
    private PIDCoefficientsEx elevatorCoeffs;
    private PIDEx elevatorPID = null;
    private double eleTarget= 0;
    private double elePos = 0;
    private double elePower = 0;

    public ElevatorSubsystem(final HardwareMap hwmap){
        //ELEVATOR
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

        elevatorCoeffs = new PIDCoefficientsEx(0.01, 0, 0, 0.25, 2, 0.5);
        elevatorPID = new PIDEx(elevatorCoeffs);

        Vector K = new Vector(new double[] {1.1, 0.3});
        FullStateFeedback controller = new FullStateFeedback(K);

        MotionProfile motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0),
                new MotionState(60, 0, 0),
                25,
                40,
                100
        );

        //MotionState state = motionProfile.get(elapsedTime);
    }

    @Override
    public void periodic() {
        elePos = (m_eleL.getCurrentPosition() + m_eleR.getCurrentPosition()) / 2.0;
        pidEX();
    }

    public void pidEX() {
        elePower = elevatorPID.calculate(eleTarget, elePos);
        ele.set(elePower);
    }

    public void fullStateFeedback() {

    }

    public void setHeight(double target) {
        this.eleTarget = target;
    }

    public double getEleTarget() {
        return eleTarget;
    }

    public double getElePos() {
        return elePos;
    }

}

package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase {
    public DoubleSupplier armLength;
    public DoubleSupplier armCOM;
    public double currentArmCOM;
    public double currentArmAngle;
    HardwareMap map;
    public MotorEx pivotLeft;
    public MotorEx pivotRight;
    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public ProfiledPIDController m_pivotPID;
    public boolean isPickup = true;
    public DoubleSupplier robotAcc;


    public PivotSubsystem(HardwareMap map, DoubleSupplier armLength, DoubleSupplier robotAcc){
        this.map = map;
        pivotLeft = new MotorEx(map,"pivotLeft");//tbd
        pivotLeft.setInverted(true);
        pivotLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pivotRight = new MotorEx(map,"pivotRight");//tbd
        pivotRight.setInverted(false);
        pivotRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.armLength = armLength;
        this.robotAcc = robotAcc;
        m_pivotPID = new ProfiledPIDController(pKP,pKI,pKD,new TrapezoidProfile.Constraints(vConstraint,aConstraint));
        m_pivotPID.m_controller.setMaximumAbsOutput(pMaxOutput);
        leftEncoder = pivotLeft.encoder;
        rightEncoder = pivotRight.encoder;
        leftEncoder.reset();
        rightEncoder.reset();

    }

    @Override
    public void periodic() {
        updateValues();
        updateTelemetry();
//        if (m_pivotPID.atGoal()) {
        setMotors(calculateFeedForward());
//        } else {
//            setMotors(-m_pivotPID.calculate(currentArmAngle) + calculateFeedForward());
//        }
    }

    
    private void updateValues() {
        calcArmAngle();
        m_pivotPID.setPID(pKP,pKI,pKD);
        m_pivotPID.setTolerance(pGoalTolerance);
        m_pivotPID.setGoalTolerance(pGoalTolerance, pGoalVelocityTolerance);
        m_pivotPID.m_controller.setAccumilatorResetTolerance(pGoalTolerance);//TODO:look at this
        m_pivotPID.setIzone(pIzone);
        m_pivotPID.m_controller.setMaximumAbsOutput(pMaxOutput);

    }



    private void updateTelemetry() {
        dashboardTelemetry.addData("armAngle", currentArmAngle);
        dashboardTelemetry.addData("accel",robotAcc);
//        dashboardTelemetry.addData("COMAngle", aCOMAngle());
//        dashboardTelemetry.addData("_pid+ff value", m_pivotPID.calculate(currentArmAngle)+calculateFeedForward());
        dashboardTelemetry.addData("_pid value", -m_pivotPID.calculate(currentArmAngle));
        dashboardTelemetry.addData("_FF", calculateFeedForward());
        dashboardTelemetry.addData("balanceAngle", aBalanceAngle(armLength));
//        dashboardTelemetry.addData("kG", akG(armLength));
        dashboardTelemetry.addData("rightEncoder", rightEncoder.getPosition());
        dashboardTelemetry.addData("rightEncoder rev", rightEncoder.getRevolutions());
        dashboardTelemetry.addData("leftEncoder rev", leftEncoder.getRevolutions());
        dashboardTelemetry.update();

    }

    public double calculateFeedForward(){
        return -akG(armLength)* Math.cos(Math.toRadians(aCOMAngle()));
    }

    private double aCOMAngle(){
        return currentArmAngle+90 - aBalanceAngle(armLength);
    }

    private double aBalanceAngle(DoubleSupplier x){
        return 117 + 7.84*x.getAsDouble() + 1.04*Math.pow(x.getAsDouble(),2);
    }
    private double akG(DoubleSupplier x) {
        return -0.0446*x.getAsDouble() + 0.162;
    }




    private void calcArmAngle() {
        currentArmAngle = 10.321*leftEncoder.getRevolutions();//linear equation (encoder to angle)
    }

    public void setMotors(double power){
        if(power>0.001)
        {
            pivotRight.set(power+kS);
            pivotLeft.set(power+kS);
        }
        else if(power<-0.001)
        {
            pivotRight.set(power-kS);
            pivotLeft.set(power-kS);

        }
        else{
            pivotRight.set(0);
            pivotLeft.set(0);
        }
    }
    private Command set(double setpoint){
        return new InstantCommand(()->m_pivotPID.setGoal(setpoint));
    }
    public Command setWithProfile(double setpoint, double maxA, double maxV){
        return new ParallelCommandGroup(new InstantCommand(()->m_pivotPID.setConstraints(new TrapezoidProfile.Constraints(maxV,maxA))),set(setpoint));
    }
    public Command disablePID(){
        return new InstantCommand(()->m_pivotPID.disable());
    }
    public Command enablePID(){
        return new InstantCommand(()->m_pivotPID.enable(currentArmAngle));
    }


    public Command toggle() {
        return new InstantCommand(()->isPickup = !isPickup);
    }
}

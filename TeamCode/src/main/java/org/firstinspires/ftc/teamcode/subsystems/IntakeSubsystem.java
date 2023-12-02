package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.sun.source.doctree.StartElementTree;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    DcMotorEx flyWheel;
    DcMotorEx flyWheelOutside;
    HardwareMap m_hardwareMap;
    Telemetry m_telemetry;
    public static double P =9;
    public static double I = 0;
    public static double D = 0;
    public static double F = 0;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        m_hardwareMap = hardwareMap;
        m_telemetry = telemetry;
        flyWheel = m_hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheelOutside = m_hardwareMap.get(DcMotorEx.class, "pe");

        flyWheelOutside.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flyWheelOutside.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorConfigurationType motorConfigurationType = flyWheelOutside.getMotorType();
        motorConfigurationType.setAchieveableMaxRPMFraction(0.99);
        motorConfigurationType.setMaxRPM(435);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorConfigurationType motorConfigurationType2 = flyWheel.getMotorType();
        motorConfigurationType2.setAchieveableMaxRPMFraction(0.99);
        motorConfigurationType2.setMaxRPM(435);
    }

    public void stop(){
        flyWheel.setPower(0);
        flyWheelOutside.setPower(0);
        }


    public void intake(double speed){
        flyWheel.setPower(speed);
        flyWheelOutside.setPower(speed);

    }
    public void outake(){
        flyWheel.setPower(-0.7);
        flyWheelOutside.setPower(-0.7);

    }

    public Command intakeCommand(){
        return new InstantCommand(()-> this.intake(1.0));
    }

    public Command outakeCommand(){
        return new InstantCommand(()-> this.outake());
    }

    public Command stopCommand(){return new InstantCommand(()-> this.stop());}
    public void periodic(){
        flyWheel.setVelocityPIDFCoefficients(P, I, D, F);
        m_telemetry.addData("desired velocity", flyWheel.getVelocity());

    }

}

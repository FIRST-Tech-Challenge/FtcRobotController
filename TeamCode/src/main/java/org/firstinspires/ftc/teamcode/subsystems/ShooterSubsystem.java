package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.archive.Shorty3Worlds.p;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ShooterSubsystem {

    // public static double shooterSpeed = 1320;
    public static double shooterSpeed = 1340;


    public static double realShooterSpeed = 0;

    public static double P = 150;
    public static double I = 0;
    public static double D = 0;

    public static double F = 13.65;

    public static boolean UPDATE_FLYWHEEL_PID = false;


    private DcMotorEx shooter;

    private Telemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap,Telemetry telemetry) {

        this.telemetry = telemetry;
        shooter = hardwareMap.get(DcMotorEx.class,"Shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void initShooter(){
        PIDFCoefficients pidfNew = new PIDFCoefficients (P, I, D, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

    }

    public void runShooter(){
       shooter.setVelocity(shooterSpeed);
        telemetry.addData("shooter speed",shooter.getVelocity());
        telemetry.addData("shooter target",shooterSpeed);
        PIDFCoefficients currentPIDF = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Flywheel P", currentPIDF.p);
        telemetry.addData("Flywheel F", currentPIDF.f);
    }

    public void stopShooter(){
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setPower(0);
    }

    public void pidfTuning(){
        if (UPDATE_FLYWHEEL_PID){
            PIDFCoefficients pidfNew = new PIDFCoefficients (P, I, D, F);
            shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);
            UPDATE_FLYWHEEL_PID = false;
        }
        realShooterSpeed = shooter.getVelocity();
    }
}

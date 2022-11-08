package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
/**
 * Example class created by Tiernan demonstrating better OOP and PIDF usage
 */
public class Arm {
    private PIDFController armPIDF;
    private Motor armMotor;

    public static double armKp = 0.0085;
    public static double armKi = 0.001;
    public static double armKd = 0.0001;
    public static double armKf = 0.000;

    private double correction;

    public static double EXTAKE_POS = 1200; // Actual position based on encoder readings
    public static double INTAKE_POS = 25;
    public double targetPos = 0;

    private final double[] PIDF_COEFF = {armKp, armKi, armKd, armKf};

    public Arm(HardwareMap hardwareMap){
        armMotor = new Motor(hardwareMap, "ARM", Motor.GoBILDA.RPM_60);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setRunMode(Motor.RunMode.VelocityControl);
        armMotor.resetEncoder(); // We want the arm to start at a position of 0
        armPIDF = new PIDFController(PIDF_COEFF[0], PIDF_COEFF[1], PIDF_COEFF[2], PIDF_COEFF[3]);
    }

    public void update(Telemetry telemetry){
        armPIDF.setPIDF(armKp, armKi, armKd, armKf);
        correction = armPIDF.calculate(armMotor.getCurrentPosition(), targetPos);

        telemetry.addData("Correction: ", correction);
        telemetry.addData("Target Position: ", targetPos);
        telemetry.addData("Motor Position: ", armMotor.getCurrentPosition());
        telemetry.update();

        armMotor.set(correction);
    }

    public void setExtake(){
        targetPos = EXTAKE_POS;
    }

    public void setIntake(){
        targetPos = INTAKE_POS;
    }
}

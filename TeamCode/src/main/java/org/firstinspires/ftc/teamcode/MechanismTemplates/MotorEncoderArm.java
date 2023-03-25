package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorEncoderArm {

    private PIDFController armPIDF;
    private Motor armMotor;

    public static double armKpUp = 0.006; // old is 0.012
    public static double armKpDown = 0.00109;
    public static double armKi = 0.00001;
    public static double armKd = 0.00007;
    public static double armKf = 0;
    public static double EXTAKE_POS = 1200; // 180 old val; in degrees of absolute encoder
    public static double INTAKE_POS = 100; // 65 old val
    public static double AUTOEXTAKE_POS = 1150;

    private double targetPos = 65;

    public MotorEncoderArm(HardwareMap hardwareMap){
        armMotor = new Motor(hardwareMap, "ARM", Motor.GoBILDA.RPM_84); // pin 1 control hub
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setRunMode(Motor.RunMode.VelocityControl);
        armPIDF = new PIDFController(armKpUp, armKi, armKd, armKf);
    }

    public void update(Telemetry telemetry){
        double correction = armPIDF.calculate(getArmPosition(), targetPos);
        telemetry.addData("Correction: ", correction);
        telemetry.addData("Target Position: ", targetPos);
        telemetry.addData("Motor Position: ", getArmPosition());
        telemetry.update();
        armMotor.set(correction); // sets a PID-tuned voltage for the arm motor
    }

    private int getArmPosition(){
        return armMotor.getCurrentPosition();
    }

    public void setExtake(){
        armPIDF.setPIDF(armKpUp, armKi, armKd, armKf);
        targetPos = EXTAKE_POS;
    }

    public void setAutoExtake(){
        armPIDF.setPIDF(armKpUp, armKi, armKd, armKf);
        targetPos = AUTOEXTAKE_POS;
    }

    public void setIntake(){
        armPIDF.setPIDF(armKpDown, armKi, armKd, armKf);
        targetPos = INTAKE_POS;
    }

    public void setCustom(int custom){
        armPIDF.setPIDF(armKpUp, armKi, armKd, armKf);
        targetPos=custom;
    }


}

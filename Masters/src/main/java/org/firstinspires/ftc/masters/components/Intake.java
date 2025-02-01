package org.firstinspires.ftc.masters.components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    PIDController pidController;

    public static double p = 0.0024, i = 0.0002, d = 0.00001;
    public static double f = 0.09;

    Init init;

    Telemetry telemetry;
    Servo intakeLeft;
    Servo intakeRight;
    DcMotor intakeMotor;
    DcMotor extendo;

    public static double RETRACT_POWER = -0.6;
    public static double EXTEND_POWER = 0.6;
    public static double INTAKE_POWER = 0.9;

    private int target;

    public Intake(Init init, Telemetry telemetry){
        this.telemetry = telemetry;
        this.init = init;

        intakeLeft = init.getIntakeLeft();
        intakeRight= init.getIntakeRight();
        intakeMotor = init.getIntake();
        extendo = init.getIntakeExtendo();
        initializeHardware();

    }

    public void initializeHardware() {

        intakeRight.setPosition(ITDCons.intakeInitRight);
        intakeLeft.setPosition(ITDCons.intakeInitLeft);

        pidController = new PIDController(p,i,d);
        target =0;

    }

    public void startIntake (){
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void reverseIntake(){
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
    }


    public void retractSlide() {

        if (target>0){
            target = target -50;
        }
//        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      extendo.setPower(RETRACT_POWER);
    }

    public void extendSlide() {

        if (target < ITDCons.MaxExtension){
            target = target - 50;
        }
//        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendo.setPower(EXTEND_POWER);
    }

    public void stopExtendo(){
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendo.setPower(0);
    }

    public void extendSlide(int position){
            extendo.setTargetPosition(position);
            extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendo.setPower(EXTEND_POWER);
    }

    public boolean extendoSlideIsBusy(){
        return extendo.isBusy();
    }


    public double getExtensionPosition(){
        return extendo.getCurrentPosition();
    }


    public void dropIntake(){
        intakeLeft.setPosition(ITDCons.dropLeft);
        intakeRight.setPosition(ITDCons.dropRight);
    }

    public void liftIntake(){
        intakeLeft.setPosition(ITDCons.liftIntakeLeft);
        intakeRight.setPosition(ITDCons.liftIntakeRight);
    }

    public void update(){

//frontRight
            int extendoPos = extendo.getCurrentPosition();

            telemetry.addData("extendoPos",extendoPos);
            double pid = pidController.calculate(extendoPos, target);

            telemetry.addData("extendo PID",pid);

            extendo.setPower(pid);

        }


}

package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Slide {
    private PIDFController slidePIDF;
    private Motor slideLeft, slideRight;

    //TODO: change values into actual tested positions instead of placeholders
    private final static double HIGH_JUNCTION= 2300; //<-- 12.90, 2150 ;
    private final static double MID_JUNCTION = 1500; //<-- 12.90, 1550 ;
    private final static double LOW_JUNCTION = 325;  //<-- 12.90, 500 ;
    public static double ZERO_POSITION = 10;//5;
    public final double MAX = 2400;

    public static double slideKp = 0.003; //0.00326; //0.0039;
    public static double slideKi = 0.000000325; //0.00000325;
    public static double slideKd = 0.000001; //0.000001;
    public static double slideKf = 0.000069; //0.000069;

    private final double[] PIDF_COFFECIENTS = {slideKp, slideKi, slideKd, slideKf};

    private double targetPos;
    double correctionLeft;
    double correctionRight;

    public Slide(HardwareMap hardwareMap){
        slideLeft = new Motor(hardwareMap, "SL", Motor.GoBILDA.RPM_312); // Pin 3?
        slideRight = new Motor(hardwareMap,"SR", Motor.GoBILDA.RPM_312); // Pin ___

        slideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideLeft.setRunMode(Motor.RunMode.VelocityControl);
        slideRight.setRunMode(Motor.RunMode.VelocityControl);

        slideLeft.setInverted(true);


        slidePIDF = new PIDFController(PIDF_COFFECIENTS[0],PIDF_COFFECIENTS[1],PIDF_COFFECIENTS[2],PIDF_COFFECIENTS[3]);

        targetPos = ZERO_POSITION; // target position is 0 by default
        slideRight.resetEncoder();
        slideLeft.resetEncoder();
    }

    public void update(Telemetry telemetry){
        slidePIDF.setPIDF(slideKp, slideKi, slideKd, slideKf);

        correctionLeft = slidePIDF.calculate(slideLeft.getCurrentPosition(), targetPos);
        correctionRight = slidePIDF.calculate(slideRight.getCurrentPosition(), targetPos);

/*
        telemetry.addData("targetPosition: ", targetPos);
        telemetry.addData("Right motor position: ", slideRight.getCurrentPosition());
        telemetry.addData("Left motor position: ", slideLeft.getCurrentPosition());
        telemetry.addData("Left correction: ", correctionLeft);
        telemetry.addData("Right correction: ", correctionRight);
        telemetry.update();

 */


        // sets the output power of the motor
        slideLeft.set(correctionLeft);
        slideRight.set(correctionRight);
    }

    public void manualSlides(int slideIncrement){
        if((targetPos + slideIncrement) <= 100 && (targetPos - slideIncrement) >= 0) {
            targetPos += slideIncrement;
        }
    }

    public void setIntakeOrGround(){
        targetPos = ZERO_POSITION;
    }

    public void setLowJunction(){
        targetPos = LOW_JUNCTION;
    }

    public void setMidJunction(){
        targetPos = MID_JUNCTION;
    }

    public void setHighJunction(){
        targetPos = HIGH_JUNCTION;
    }

    public void setManualSlide(int increment){
        double targetManual = (slideRight.getCurrentPosition() + increment + slideLeft.getCurrentPosition() + increment)/2;
        if(((targetManual + increment) <= MAX) && ((targetManual + increment) >= ZERO_POSITION))
            targetPos = targetManual;

    }

}

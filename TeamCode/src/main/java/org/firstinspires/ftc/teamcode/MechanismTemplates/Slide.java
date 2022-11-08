package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class Slide {
    private PIDFController slidePIDF;
    private Motor slideLeft, slideRight;

    private final static double HIGH_JUNCTION= 1000; //TODO: change values into actual tested positions instead of placeholders
    private final static double MID_JUNCTION = 800;//TODO: change values into actual tested positions instead of placeholders
    private final static double LOW_JUNCTION = 500;//TODO: change values into actual tested positions instead of placeholders
    private final static double ZERO_POSITION = 0;//TODO: change values into actual tested positions instead of placeholders

    private final double[] PIDF_COFFECIENTS = {0.001,0.001,0.001,0.001};//TODO: will have to tune to proper values later

    private double targetPosition;

    public Slide(HardwareMap hardwareMap){
        slideLeft = new Motor(hardwareMap, "SL", Motor.GoBILDA.RPM_312);
        slideRight = new Motor(hardwareMap,"SR", Motor.GoBILDA.RPM_312);
        slideLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideLeft.setRunMode(Motor.RunMode.VelocityControl);
        slideRight.setRunMode(Motor.RunMode.VelocityControl);

        slideLeft.setInverted(true);

        slidePIDF = new PIDFController(PIDF_COFFECIENTS[0],PIDF_COFFECIENTS[1],PIDF_COFFECIENTS[2],PIDF_COFFECIENTS[3]);

        targetPosition = ZERO_POSITION;

    }

    public void Update(){
        double correctionLeft = slidePIDF.calculate(slideLeft.getCurrentPosition(),targetPosition);
        double correctionRight = slidePIDF.calculate(slideRight.getCurrentPosition(),targetPosition);

        slideLeft.set(correctionLeft);
        slideRight.set(correctionRight);
    }

    public void setIntakeOrGround(){
        targetPosition = ZERO_POSITION;
    }

    public void setLowJunction(){
        targetPosition = LOW_JUNCTION;
    }

    public void setMidJunction(){
        targetPosition = MID_JUNCTION;
    }

    public void setHighJunction(){
        targetPosition = HIGH_JUNCTION;
    }

    public void manualSlides(int slideIncrement){
        if((targetPosition+slideIncrement)<=100 && (targetPosition-slideIncrement)>=0)
            targetPosition -= slideIncrement;
    }

}

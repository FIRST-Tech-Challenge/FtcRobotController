package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slide {
    private PIDFController slidePIDF;
    private Motor slideLeft, slideRight;

    //TODO: change values into actual tested positions instead of placeholders
    private final static double HIGH_JUNCTION= 1000;
    private final static double MID_JUNCTION = 800;
    private final static double LOW_JUNCTION = 500;
    private final static double ZERO_POSITION = 0;

    public static double slideKp = 0.0001;
    public static double slideKi = 0.001;
    public static double slideKd = 0.001;
    public static double slideKf = 0.001;

    private final double[] PIDF_COFFECIENTS = {slideKp, slideKi, slideKd, slideKf};//TODO: will have to tune to proper values later

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

        targetPosition = ZERO_POSITION; // target position is 0 by default

    }

    public void Update(){ // updates the position of the motor
        double correctionLeft = slidePIDF.calculate(slideLeft.getCurrentPosition(),targetPosition);
        double correctionRight = slidePIDF.calculate(slideRight.getCurrentPosition(),targetPosition);

        // Not sure if you would want to add a conditional to stop the motors at some point, but idt adding another while loop would work
        slideLeft.set(correctionLeft); // sets the output power of the motor
        slideRight.set(correctionRight);
    }

    public void manualSlides(int slideIncrement){
        if((targetPosition + slideIncrement) <= 100 && (targetPosition - slideIncrement) >= 0) {
            targetPosition -= slideIncrement;
        }
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

}

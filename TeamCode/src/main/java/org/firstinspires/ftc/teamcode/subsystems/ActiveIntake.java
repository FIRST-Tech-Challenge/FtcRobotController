package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActiveIntake {

    //Adjustable constraints/constants
    private double MAX_POWER = 1;

    //Internal Variables
    private CRServo intake;
    private boolean isRunning;

    /**
     * Quick constructor used to create an Active Intake Subsystem Class
     * @param hw [HardwareMap] Hardware map used to initialize servos.
     */
    public ActiveIntake(HardwareMap hw){
        this(hw, "intake");
        isRunning=false;
    }

    /**
     * Primary constructor used to create an Active Intake Subsystem Class
     * @param hw [HardwareMap] Hardware map used to initialize servos.
     * @param name [String] Name of the intake servo set in configuration.
     */
    public ActiveIntake(HardwareMap hw, String name){
        intake = hw.get(CRServo.class, name);
        isRunning=false;
    }

    /**
     * Spins intake servo to be able to intake samples
     */
    public void activateIntake(){
        intake.setPower(MAX_POWER);
        isRunning=true;
    }

    /**
     * Spins intake servo to be able to deposit samples
     */
    public void reverseIntake(){
        intake.setPower(-MAX_POWER);
        isRunning=true;

    }

    /**
     * Turns off all power supplied to intake servo.
     */
    public void deactivateIntake(){
        intake.setPower(0);
        isRunning = false;
    }
    public boolean isRunning() {
        return isRunning;

    }
}

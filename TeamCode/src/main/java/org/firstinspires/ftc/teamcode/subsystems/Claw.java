package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    //Adjustable Constraints
    public double OPEN_POS = 1, CLOSE_POS = 0.5;

    //Internal variables
    private Servo claw;
    private boolean isOpen;

    /**
     * Quick constructor for Claw Subsystem Class
     * @param hw [HardwareMap] The hardware map used to initialize servos
     */
    public Claw(HardwareMap hw){
        this(hw,"claw");
    }

    /**
     * Primary constructor for Claw Subsystem Class
     * @param hw [HardwareMap] The hardware map used to initialize servos
     * @param servoName [String] The servo's assigned name in the configuration
     */
    public Claw(HardwareMap hw, String servoName){
        claw = hw.get(Servo.class,servoName);
        isOpen = false;
    }

    /**
     * A tester funciton to assess the optimal position for the claw.
     * @param power [double] Can be paired with a trigger or joystick for dynamic speed changes
     * @return Returns the new position of the claw.
     */
    public double changePosition(double power){
        double CLAW_SPEED = 0.05;
        double newPos = claw.getPosition() + CLAW_SPEED * power;
        claw.setPosition(newPos);
        return newPos;
    }

    /**
     * @return [double] Returns the current position of the servo
     */
    public double getPosition(){
        return claw.getPosition();
    }

    /**
     * Opens the claw
     */
    public void openClaw(){
        claw.setPosition(OPEN_POS);
        isOpen = true;
    }

    /**
     * Closes the claw
     */
    public void closeClaw(){
        claw.setPosition(CLOSE_POS);
        isOpen = false;
    }

    /**
     * Toggles the claw between being opened or closed
     */
    public void toggleClaw(){
        if (isOpen) {
            closeClaw();
        }
        else{
            openClaw();
        }
    }

    public boolean getIsOpen(){
        return isOpen;
    }

    @Override
    public String toString(){
        return String.format(
                "Claw Open: %b\n",
                isOpen);
    }
}

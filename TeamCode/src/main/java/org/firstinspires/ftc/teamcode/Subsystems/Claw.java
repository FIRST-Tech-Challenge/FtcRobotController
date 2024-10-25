package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class Claw extends SubsystemBase {

    // Create wrist Servo
    private final Servo clawServo;

    /** Place code here to initialize subsystem */
    public Claw() {

        // Creates a Servo using the hardware map
        clawServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "clawServo");

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // Open and close the claw using the enum ClawState
    public void ControlClaw(ClawState state){
        clawServo.setPosition(state.getValue());
    }


}
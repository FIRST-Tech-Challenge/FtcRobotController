package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class ClawTouchSensor extends SubsystemBase {
    // Local objects and variables here
    private final TouchSensor clawTouch;

    /** Place code here to initialize subsystem */
    public ClawTouchSensor() {
        clawTouch =  RobotContainer.ActiveOpMode.hardwareMap.get(TouchSensor.class, "clawTouch");
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    // place special subsystem methods here
    public boolean ControlTouchSenser(){return clawTouch.isPressed();}
}
package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class ElbowJoint extends SubsystemBase {

    // Create wrist Servo
    private final Servo ElbowServo;

    /** Place code here to initialize subsystem */
    public ElbowJoint() {

        // Creates a Servo using the hardware map
        ElbowServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "elbowServo");

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }


    // Turns the Servo a set amount of degrees
    public void RotateTo(int degrees){

        // Converts degrees into 0-1 float
        double servoPos = degrees/180.0;

        // Set the Servo to ServoPos
        ElbowServo.setPosition(servoPos);

    }

    // Sets the Elbow to fixed positions
    public void setPos(ElbowPosition pos) {RotateTo(pos.getValue());}

}
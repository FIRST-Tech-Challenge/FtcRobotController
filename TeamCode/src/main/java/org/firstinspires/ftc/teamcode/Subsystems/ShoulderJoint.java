package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class ShoulderJoint extends SubsystemBase {

    // Create the shoulder motor
    private final Servo ShoulderServo;

    /** Place code here to initialize subsystem */
    public ShoulderJoint() {

        // Creates a Servo using the hardware map
        ShoulderServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "shoulderServo");

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
        ShoulderServo.setPosition(servoPos);

    }

    // Sets the Elbow to fixed positions
    public void setPos(ShoulderPosition pos) {RotateTo(pos.getValue());}


}
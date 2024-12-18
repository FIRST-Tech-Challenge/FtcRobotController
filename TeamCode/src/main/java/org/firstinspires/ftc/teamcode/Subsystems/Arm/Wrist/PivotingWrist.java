package org.firstinspires.ftc.teamcode.Subsystems.Arm.Wrist;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class PivotingWrist extends SubsystemBase {

    // Create wrist Servo
    /**0° is counterclockwise looking from behind*/
    private final Servo wristServo;

    /** Place code here to initialize subsystem */
    public PivotingWrist() {

        // Creates a Servo using the hardware map
        wristServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "wristRotateServo");

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
        wristServo.setPosition(servoPos);

    }

    private double servo_degrees;

    public void RotateTo(int degrees, int adjustment){

        servo_degrees = (degrees + adjustment)/180.0;
        while (servo_degrees>1){
            servo_degrees-=1;
        }

        // Converts degrees into 0-1 float
        double servoPos = (servo_degrees);

        // Set the Servo to ServoPos
        wristServo.setPosition(servoPos);


    }

}
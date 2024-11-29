package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class Climb extends SubsystemBase {
    // Local objects and variables here
    private final Servo climbServo;

    private final TouchSensor climbLowLimit;

    private final TouchSensor climbHighLimit;

    /** Place code here to initialize subsystem */
    public Climb() {

        climbServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "climbServo");

        climbLowLimit = RobotContainer.ActiveOpMode.hardwareMap.get(TouchSensor.class,"climbLowLimit");

        climbHighLimit = RobotContainer.ActiveOpMode.hardwareMap.get(TouchSensor.class,"climbHighLimit");

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        RobotContainer.DBTelemetry.addData("highLimit",GetHighLimit());
        RobotContainer.DBTelemetry.addData("lowLimit",GetLowLimit());
        RobotContainer.DBTelemetry.update();


    }

    // place special subsystem methods here
    public void ClimbServoSpeed(double speed){

        climbServo.setPosition(speed);
    }

    public boolean GetHighLimit (){

        return climbHighLimit.isPressed();
    }

    public boolean GetLowLimit (){

        return climbLowLimit.isPressed();
    }


}
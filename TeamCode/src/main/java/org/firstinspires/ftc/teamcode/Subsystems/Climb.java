package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Linear Slide Subsystem
 */
public class Climb extends SubsystemBase {

    // Initialize both motors
    private final DcMotorEx climb;
    ;


    /**
     * Place code here to initialize subsystem
     */
    public Climb() {

        // Creates the motors using the hardware map
        climb = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "climbRackJack");


        // Resets the encoders for both motors
        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Turn the left motor in reverse to move the slide upwards
        climb.setDirection(DcMotorSimple.Direction.REVERSE);


        // Sets the motors PIDF values
        climb.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);


        // Setting target to zero upon initialization
        climb.setTargetPosition(0);

        // Puts the motors into position control mode
        climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

    }

    // Using the var ticks sets the motor encoder ticks to a set position
    public void moveClimb(int ticks) {

        // Sets both motors to the ticks target position
        climb.setTargetPosition(ticks);


        // Sets the power VERY IMPORTANT
        climb.setPower(1);
        
    }

}
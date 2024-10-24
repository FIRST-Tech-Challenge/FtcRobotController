package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Subsystem */
public class ShoulderJoint extends SubsystemBase {

    // Create the shoulder motor
    private final DcMotorEx shoulderMotor;

    // Tracks the offset of the shoulder from it's zero pos
    private static final int KEYOFFSET = 0;

    /** Place code here to initialize subsystem */
    public ShoulderJoint() {

        // Creates the motor using the hardware map
        shoulderMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "shoulderMotor");

        // Resets the encoders for both motors
        shoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setting target to zero upon initialization
        shoulderMotor.setTargetPosition(0);

        // Puts the motors into position control mode
        shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    public void moveTo(ShoulderPosition pos) {

        shoulderMotor.setTargetPosition(pos.getValue());

        // Sets the power
        shoulderMotor.setPower(1);
    }



}
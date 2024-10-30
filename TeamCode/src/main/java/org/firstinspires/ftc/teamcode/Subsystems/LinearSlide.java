package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.RobotContainer;


/**
 * Linear Slide Subsystem
 */
public class LinearSlide extends SubsystemBase {

    // Initialize both motors
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;


    /**
     * Place code here to initialize subsystem
     */
    public LinearSlide() {

        // Creates the motors using the hardware map
        leftMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "left_linear_slide");
        rightMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "right_linear_slide");

        // Resets the encoders for both motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn the left motor in reverse to move the slide upwards
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Sets the motors PIDF values
        leftMotor.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);
        rightMotor.setVelocityPIDFCoefficients(10.0, 0.2, 0.001, 10.0);

        // Setting target to zero upon initialization
        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);

        // Puts the motors into position control mode
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Method called periodically by the scheduler
     * Place any code here you wish to have run periodically
     */
    @Override
    public void periodic() {

    }

    // Using the var ticks sets the motor encoder ticks to a set position
    public void moveLinearSlide(int ticks) {

        // Sets both motors to the ticks target position
        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);

        // Sets the power VERY IMPORTANT
        leftMotor.setPower(1);
        rightMotor.setPower(1);
    }

    // Used to move the slides to fixed levels
    public void moveTo(SlideTargetHeight target) {
        moveLinearSlide(target.getValue());
    }

}

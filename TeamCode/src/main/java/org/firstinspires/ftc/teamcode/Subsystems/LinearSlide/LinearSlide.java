package org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
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
        leftMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "leftLinearSlide");
        rightMotor = RobotContainer.ActiveOpMode.hardwareMap.get(DcMotorEx.class, "rightLinearSlide");


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

    /**Moves the linear slide motors to a set position.
     * DO NOT USE OVERRIDE OUTSIDE OF AUTOMATED MOVEMENTS
     *
     * @param ticks - Amount to move slides (int)
     * @param override - When true ignores safety conditions (boolean)
     * */
    public void moveLinearSlide(int ticks, boolean override) {
        Pose2d currentPOS = RobotContainer.odometry.getCurrentPos();
        if (!(Math.abs(currentPOS.getX())<0.9 && Math.abs(currentPOS.getY())<1.15) || override){
        // Sets both motors to the ticks target position
        leftMotor.setTargetPosition(ticks);
        rightMotor.setTargetPosition(ticks);

        // Sets the power VERY IMPORTANT
        leftMotor.setPower(1);
        rightMotor.setPower(1);
        }
    }

    /**Moves the slides to fixed levels. Use for driver controlled movements.
     *
     * @param target - slide target position (SlideTargetHeight)
     * */
    public void moveTo(SlideTargetHeight target) {
        moveLinearSlide(target.getValue(), false);
    }

    /**Moves the slides to fixed levels.
     * DO NOT USE OVERRIDE OUTSIDE OF AUTOMATED MOVEMENTS
     *
     * @param target - slide target position (SlideTargetHeight)
     * @param override - When true ignores safety conditions (boolean)
     * */
    public void moveTo(SlideTargetHeight target, boolean override) {
        moveLinearSlide(target.getValue(), override);
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDF;


public class Arm {

    private DcMotor rotationMotor;
    private DcMotor armMotor;
    private PIDF pidController;

    // PID coefficients (tune these values)
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.01;
    private final double kF = 0.0;

    // Constructor
    public Arm(HardwareMap hardwareMap) {
        // Initialize motor
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize motor
        rotationMotor = hardwareMap.get(DcMotor.class, "rotationMotor");
        rotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize PIDF controller with tuned values
        pidController = new PIDF(kP, kI, kD, kF);

        // Set an initial target position for the arm
        pidController.setSetPoint(0);  // Start at encoder position 0
    }

    // Method to set the arm position
    public void setArmPosition(int targetPosition) {
        // Update the setpoint in the PID controller
        pidController.setSetPoint(targetPosition);

        // Get the current position from the encoder
        int currentPosition = armMotor.getCurrentPosition();

        // Calculate the PID output
        double power = pidController.calculate(currentPosition);

        // Set the motor power based on the PID output
        armMotor.setPower(power);
    }

    public void rotateArm(double power) {
        rotationMotor.setPower(power);
    }
    // Check if the arm is at the target position
    public boolean atTargetPosition() {
        return pidController.atSetPoint();
    }
}

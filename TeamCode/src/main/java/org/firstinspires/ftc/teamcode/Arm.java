package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFC;


public class Arm {

    public DcMotor rotationMotor;
    public DcMotor armMotor;
    public PIDFC pidf;


    // PIDF coefficients (tune these values)
    public static double kp = 0.01;
    public static double ki = 0.0;
    public static double kd = 0.01;
    public static double kf = 0.0;

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
        pidf = new PIDFC(kp, ki, kd, kf);

        // Set an initial target position for the arm
        pidf.setSetPoint(0);  // Start at encoder position 0
    }
        // Method to set the arm position
        public void drive(int targetPosition){
            // Update the setpoint in the PID controller
            pidf.setSetPoint(targetPosition);

            // Get the current position from the encoder
            int currentPosition = armMotor.getCurrentPosition();

            // Calculate the PID output
            double power = pidf.calculate(currentPosition);

            // Set the motor power based on the PID output
            armMotor.setPower(power);
        }
        public void rotateArm (double power){
            rotationMotor.setPower(power);
        }
        //Check if the arm is at the target position
        public boolean atTargetPosition () {
            return pidf.atSetPoint();

        }
    }
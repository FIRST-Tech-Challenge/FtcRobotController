/*package org.firstinspires.ftc.teamcode.opmodes.Subsystems;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ArmCode {
    private DcMotor armMotor;
    private DcMotor liftMotor;

    // Motor power settings
    private static final double ARM_POWER = 0.8; // Adjust based on required speed
    private static final double LIFT_POWER = 0.8;

    public ArmCode(DcMotor armMotor, DcMotor liftMotor) {
        this.armMotor = armMotor;
        this.liftMotor = liftMotor;

        // Set zero power behavior to brake
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void controlArm(Gamepad gamepad) {
        // Linear Slide Motor Control (armMotor)
        if (gamepad.dpad_up) {
            armMotor.setPower(ARM_POWER);
        } else if (gamepad.dpad_down) {
            armMotor.setPower(-ARM_POWER);
        } else {
            armMotor.setPower(0);    // Stop the linear slide
        }

        // Lift Motor Control (liftMotor)
        if (gamepad.triangle) {
            liftMotor.setPower(LIFT_POWER);
        } else if (gamepad.cross) {
            liftMotor.setPower(-LIFT_POWER);
        } else {
            liftMotor.setPower(0);    // Stop the arm
        }
    }
}

 */

package org.firstinspires.ftc.teamcode.opmodes.test.DriveTests.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ArmCode {
    private DcMotor armMotor;
    private DcMotor liftMotor;

    // Motor power settings
    private static final double ARM_POWER = 0.8;
    private static final double LIFT_POWER = 0.8;

    // Target position for the lift
    private int liftTargetPosition = 0;

    public ArmCode(DcMotor armMotor, DcMotor liftMotor) {
        this.armMotor = armMotor;
        this.liftMotor = liftMotor;

        // Configure the lift motor with encoders
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm Brakes
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void controlArm(Gamepad gamepad) {
        // Arm Motor Control
        //TODO: Set LinearSlide Positions to the joystick
        if (gamepad.square) {
            armMotor.setPower(ARM_POWER);
        } else if (gamepad.circle) {
            armMotor.setPower(-ARM_POWER);
        } else {
            //TODO: Test
            armMotor.setTargetPosition(armMotor.getCurrentPosition()); //Sets current position as 'position'
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Runs to set Position
            armMotor.setPower(0.1); //To stop jerking (maybe?)
        }

        // Lift Motor Control
        //TODO: Set Arm Motor to specific positions with encoder
        if (gamepad.triangle) {
            liftTargetPosition += 50; // Move up
        } else if (gamepad.cross) {
            liftTargetPosition -= 50; // Move down
        }

        // Ensure the lift position is within safe bounds
        //Simpler, limits jerking and weird movements
        liftTargetPosition = Math.max(0, liftTargetPosition); // Replace 0 with the lower limit if needed

        // Set the lift motor target position
        liftMotor.setTargetPosition(liftTargetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power to hold the position
        if (liftMotor.isBusy()) {
            liftMotor.setPower(LIFT_POWER);
        } else {
            liftMotor.setPower(0.1); //or 0, whatever stops jerking
        }


    }
}

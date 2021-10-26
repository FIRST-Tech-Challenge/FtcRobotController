package org.firstinspires.ftc.teamcode.Subsystems;

// Mecanum Drivetrain

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Generic_Lift {
    // Instantiate the drivetrain motor variables
    private DcMotorEx lift;


    public Generic_Lift(HardwareMap hardwareMap){                 // Motor Mapping
    lift = hardwareMap.get(DcMotorEx.class, "lf");      //Sets the names of the hardware on the hardware map
// "DeviceName" must match the Config EXACTLY

    // Set motor direction based on which side of the robot the motors are on
    lift.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void Update(Gamepad gamepad2){ //Code to be run in Op Mode void Loop at top level
        if (gamepad2.y) {
            lift.setPower(0.3);        //Sets the power for the Long arm
            lift.setTargetPosition(100);        //Tell the motor to go to 90 degrees when told to
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad2.a) {
            lift.setPower(0.3);        //Sets the power for the Long arm
            lift.setTargetPosition(100);        //Tell the motor to go to 90 degrees when told to
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }
}

package org.firstinspires.ftc.teamcode.Subsystems;

// Intake

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//ToDo: Add some kind of sensor (maybe beam break) to confirm only 1 game piece at any time
//ToDo: May use servo to open/close or drop the team element
//ToDo: May enable/disable electromagnet?
//ToDo: May need to run in different directions depending where the arm is, i.e. when arm is on front of robot the intake will be reversed compared to when it is by the back of the robot

public class Intake {
    // Instantiate the motor variables
    private DcMotorEx intake;
    int pos= 0 ;

    public Intake(HardwareMap hardwareMap){                 // Motor Mapping
        intake = hardwareMap.get(DcMotorEx.class, "intake_m");      //Sets the names of the hardware on the hardware map
        // "DeviceName" must match the Config EXACTLY

        // Set motor direction based on which side of the robot the motors are on
        intake.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void Update(Gamepad gamepad2, Generic_Lift lift){ //Code to be run in Op Mode void Loop at top level
        pos = lift.getArmPosition();                    // gets the current arm position
        if (gamepad2.right_trigger > 0 && pos>0 ) {       //runs the intake forward based on arm position
            intake.setPower(1);
        }else if (gamepad2.right_trigger > 0 && pos<0){
            intake.setPower(-1);                           // runs the intake forward, but the arm is backwards so it is negative
        }


        if (gamepad2.left_trigger > 0) {        //runs the intake backwards
            intake.setPower(-1);
        }

    }

}

package org.firstinspires.ftc.teamcode.Subsystems;

// Turntable

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//ToDo: May need multiple turntable spinners
//ToDo: Make red-side and blue-side set from beginning of match, ideally based on what auto routine was run. In other words, the same button will cause the spinner to rotate in different directions based on what side of the field we started on
//ToDo: Make pre-set program that auto goes the right distance, stops to place a new one, and then starts again (faster than a human can)

public class Turn_Table {
    // Instantiate the  motor variables
    private DcMotorEx spin_table;


    public Turn_Table(HardwareMap hardwareMap){                 // Motor Mapping
        spin_table = hardwareMap.get(DcMotorEx.class, "spin_table");      //Sets the names of the hardware on the hardware map
// "DeviceName" must match the Config EXACTLY

    // Set motor direction based on which side of the robot the motors are on
        spin_table.setDirection(DcMotorEx.Direction.FORWARD);

    }

    public void Update(Gamepad gamepad2){ //Code to be run in Op Mode void Loop at top level
        if (gamepad2.x) {        //runs the intake backwards for the BLUE side
            spin_table.setPower(-0.25); // THIS WILL BE TUNED FOR PERFECTIIIIIOOOOON
        }
        if (gamepad2.b) {        //runs the intake backwards for the RED side
            spin_table.setPower(0.25); // THIS WILL BE TUNED FOR PERFECTIIIIIOOOOON
        }
    }
}

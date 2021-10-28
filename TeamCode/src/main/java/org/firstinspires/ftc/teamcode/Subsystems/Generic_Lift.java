package org.firstinspires.ftc.teamcode.Subsystems;

// Generic Lift

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Generic_Lift { //ToDo: Rename to specific lift type once we make decision (Elevator, arm, etc.)
    // Instantiate the drivetrain motor variables
    private DcMotorEx lift;
    public int position = 0; // Integer position of the arm

    //ToDo: Add encoder count amount (still undetermined, just put a number in now)
    //ToDo: Add stop and reset encoder in init(), and run using encoder. See FTC_14133_2021_Auto.java
    //ToDo: Add limit switch sensing for either extreme location and to reset to 0 on one side (Homing) or in the middle
    //ToDo: Create set-point positions for different arm positions (Intake, place low, place mid, place high, different sides)
        //This can likely be done with looking at different states and using the arrows, bumpers, or something else to move up the integer positions
        //Pseudo code:
            // integer position == 0; //initial position of arm
            // if (gamepad.dpadup==1){
            // position = position + 1
            // else if(gamepad.dpaddown==1)
            // position = position - 1}

            // if (position == 0){
            // set encoder position to x
            // else if(position==1){
            // set encoder position to y //etc}
            // else if(position==-1){
            // set encoder position to -y //etc}
            //}

    public Generic_Lift(HardwareMap hardwareMap){                 // Motor Mapping
    lift = hardwareMap.get(DcMotorEx.class, "lift");      //Sets the names of the hardware on the hardware map
// "DeviceName" must match the Config EXACTLY

    // Set motor direction based on which side of the robot the motors are on
    lift.setDirection(DcMotorEx.Direction.FORWARD);
    position=0; //initial arm position
    }

    public void Update(Gamepad gamepad2){ //Code to be run in Op Mode void Loop at top level

        // if (gamepad.dpadup==1){   this needs to be a toggle methinks rather than simply a check
        // position = position + 1
        // else if(gamepad.dpaddown==1)
        // position = position - 1}
        //Limit this to however many positions we want (maybe 3 per side?

        // if (position == 0){
        // set encoder position to x
        // else if(position==1){
        // set encoder position to y //etc}
        // else if(position==-1){
        // set encoder position to -y //etc}
        //}

    }
    public int getArmPosition(){
        return position;
    }
}

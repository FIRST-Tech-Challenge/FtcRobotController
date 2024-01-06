package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.ExportToBlocks;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Gripper extends BlocksOpModeCompanion {
    
    // Define features of the gripper    
    static private String servoName = null;
    static private Servo servo        = null;
    static private double currentPosition  = 0.0;
    static private double targetPosition  = currentPosition;
    
    static private double maxPos    = 1.0;
    static private double midPos    = 0.5;    
    static private double minPos    = 0.0;
    
    // Put telemetry data into the queue 
    public void addTelemetry() {
        
        // Queue up the servo values
        telemetry.addData("Servo: ", servoName);        
        telemetry.addData("Target, Position, Two, One, None: ", "%5.2f  %5.2f  %5.2f  %5.2f  %5.2f", targetPosition, currentPosition, minPos, midPos, maxPos);

    }
    
    @ExportToBlocks (
        heading = "Gripper: Initialize",
        color = 255,
        comment = "Initialize gripper",
        tooltip = "Initialize gripper",
        parameterLabels = {"Gripper Motor Name",
                           "Initial Position",
                           "GripTwo Position",
                           "GripOne Position",
                           "GripNone Position"
        }
    )
    /** Initialize gripper
     */
    // Set the servo handle, connect to the servo and set its initial position.
    public static void init(String _servoName, double initPosition, double two, double one, double none) {

        // Setting initial values
        servoName = _servoName;
        servo        = null;
        
        maxPos    = two;
        midPos    = one;        
        minPos = none;
        
        if (initPosition > maxPos)  
            initPosition = maxPos;
            
        if (initPosition < minPos)
            initPosition = minPos;
        
        currentPosition  = initPosition;
                
        // Connect to servo
        servo = hardwareMap.get(Servo.class, servoName);    

        // Set the servo position to the initial position provided    
        servo.setPosition(currentPosition);    
        
        // Initialize our target position to the current position
        targetPosition = currentPosition;
        
    }

    @ExportToBlocks (
        heading = "Gripper: Grip None",
        color = 255,
        comment = "Opens gripper",
        tooltip = "At end of setting servo the gripper should be open.",
        parameterLabels = {
        }
    )
    public static void GripNone() {
        
        // Let's refresh and get our current position
        currentPosition = servo.getPosition();    
                
        // Setting our target to be our minimum, open position
        targetPosition = minPos;
        
        servo.setPosition(targetPosition);

    }

    @ExportToBlocks (
        heading = "Gripper: Grip One",
        color = 255,
        comment = "Grips one only",
        tooltip = "At end of setting servo the gripper should be holding one only.",
        parameterLabels = {
        }
    )
    public static void GripOne() {
        
        // Let's refresh and get our current position
        currentPosition = servo.getPosition();    
                
        // Setting our target to be our middle position for one pixel grip.
        targetPosition = midPos;
        
        servo.setPosition(targetPosition);

    }


    @ExportToBlocks (
        heading = "Gripper: Grip Two",
        color = 255,
        comment = "Tightens gripper to hold two pixels",
        tooltip = "At end of setting servo the gripper should be able to hold two pixels.",
        parameterLabels = {
        }
    )
    public static void GripTwo() {
        
        // Let's refresh and get our current position
        currentPosition = servo.getPosition();    
                
        // Setting our target to be our maximum, closed position
        targetPosition = maxPos;
        
        servo.setPosition(targetPosition);

    }

    // Returns the current servo position
    public static double getPosition() {
        // Read the current servo position.  
        // Assumes the servo was set to the initial position when initialized.
        currentPosition = servo.getPosition();    
        
        return currentPosition;
    }

    // Returns the minumum servo position allowed
    public static double getMin() {    
        return minPos;
    }

    // Returns the middle servo position
    public static double getMid() {    
        return midPos;
    }
    
    // Returns the maximum servo position allowed
    public static double getMax() {    
        return maxPos;
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ArmJoint {
    
    // Define features of the arm joint    
    private String servoName = null;
    private Servo servo    = null;
    private double currentPosition  = 0.0;
    private double targetPosition  = currentPosition;
    
    private double increment = 0.001;
    private double incrementDefault = increment;
    private double maxPos    = 1.0;
    private double minPos    = 0.0;
    
    private boolean movingToTarget = false;
  
    // Constructor and initialization of arm joint   
    public ArmJoint(String _servoName, Servo _servo, double _initPosition, double _Min, double _Max, double _increment) {
        // Setting initial values
        servoName =  new String (_servoName);
        servo     = _servo;
        
        if (_Max < maxPos && _Max > minPos) 
            maxPos = _Max;
            
        if (_Min < minPos && _Min < maxPos) 
            minPos    = _Min;
            
        incrementDefault = _increment;
        increment = incrementDefault;
        
        if (_initPosition > maxPos) 
            _initPosition = maxPos;
            
        if (_initPosition < minPos)
            _initPosition = minPos;
        
        currentPosition  = _initPosition;
        
    }
    
    
    // Set the servo handle, connect to the servo and set its initial position.
    public void init() {
        // Connect to servo
        //servo = HardwareMap.get(Servo.class, servoName);    
        //servo = HardwareMap.get(servoName);
        if (servo == null)
            return;
            
        // Set the servo position to the initial position provided    
        servo.setPosition(currentPosition);    
        
        // Initialize our target position to the current position
        targetPosition = currentPosition;
        
    }


    public void moveByIncrement(double INCREMENT) {
        
        movingToTarget = false;
        increment = INCREMENT;
        
        // Let's refresh and get our current position
        currentPosition = servo.getPosition(); 

        double newPosition = currentPosition + INCREMENT;
        
        if (newPosition >= maxPos)
            newPosition = maxPos;
        else if (newPosition <= minPos)
            newPosition = minPos;
        
        // Set the new position for the servo
        servo.setPosition(newPosition);
        
        // Set increment back to default
        increment = incrementDefault;

    }

    // Sets a new target position for servo and initiates first move by increment
    public void setTargetPosition(double newTarget, double numberSteps) {

        // Let's refresh and get our current position
        currentPosition = servo.getPosition();    
        
        if (newTarget > maxPos)
            targetPosition = maxPos;
        else if (newTarget < minPos)
            targetPosition = minPos;
        else
            targetPosition = newTarget;
        
        if (numberSteps != 0)
            increment = (targetPosition - currentPosition)/numberSteps;
            if (increment == 0)
                if (targetPosition > currentPosition)
                    increment = 0.0001;
                else
                    increment = -0.0001;
            
        movingToTarget = true;

    }
    
    // Move towards target position by increment
    // Returns TRUE if still moving to the set target position
    // Returns FALSE if current position is at the target position (within the increment range)
    public boolean movingtoTarget() {
        
        double position;
        
        // Let's refresh and get our current position
        currentPosition = servo.getPosition();    
        
        if (increment > 0 && targetPosition > currentPosition) {
            // Need to increase our current position
            position = currentPosition + increment;
            
            if (position > maxPos) {
                
                // We've reached the maximum position the joint can go.
                movingToTarget = false;
            } else {
                // We still need to move towards our target position
                servo.setPosition(position);
                
                movingToTarget = true;
            
            }
                
        } else if (increment > 0 && targetPosition <= currentPosition) {
            
            // We've reached the target position
            movingToTarget = false;
        
        } else if (increment < 0 && targetPosition < currentPosition) {
            
            // Need to decrease our current position            
            position = currentPosition + increment;
            
            if (position < minPos) {
                // We've reached the minimum position the joint can go.
                movingToTarget = false;
                
            } else {
                // We still need to move towards our target position
                servo.setPosition(position);
                
                movingToTarget = true;
            }
         } else if (increment < 0 && targetPosition >= currentPosition) {
            
             // We've reached the target position
            movingToTarget = false;
           
       // Oddball case where currentPosition is the target position
        } else {
            movingToTarget = false;
        }
        
        if (! movingToTarget) {
           // Set increment back to default
            increment = incrementDefault;
            targetPosition = currentPosition;
        }
        
        return movingToTarget;
        
    }

    // Returns the current servo position
    public double getCurrentPosition() {
        // Read the current servo position.  
        // Assumes the servo was set to the initial position when initialized.
        currentPosition = servo.getPosition();    
        
        return currentPosition;
    }

    // Returns the targeted servo position
    public double getTargetPosition() {
        
        return targetPosition;
    }

    // Returns the minumum servo position allowed
    public double getMin() {    
        return minPos;
    }

    // Returns the maximum servo position allowed
    public double getMax() {    
        return maxPos;
    }

    // Returns the current increment value for the joint
    public double getCurrentIncrement() {    
        return increment;
    }
    
    // Returns the name of the joint
    public String getName() {    
        return servoName;
    }

    
    // Releases the joint
    public void release() {    
        servo.getController().pwmDisable();
    }

    
}
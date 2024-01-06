package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Pose {
    
    // Define the name of the pose
    String namePose = null;
    
    // Define joint positions of the arm's pose
    private double wristPosition  = 0.0;
    private double elbowPosition  = 0.0;
    private double shoulderPosition  = 0.0;
  
    // Constructor and initialization of pose  
    public Pose(String _name, double _shoulder, double _elbow, double _wrist) {
        
        namePose = new String (_name);
        wristPosition  = _wrist;
        elbowPosition  = _elbow;
        shoulderPosition  = _shoulder;
        
    }
    

    // Returns the pose's shoulder target position
    public double getPoseShoulderTarget() {
        
        return shoulderPosition;
    }

    // Returns the pose's elbow target position
    public double getPoseElbowTarget() {
        
        return elbowPosition;
    }
    
    // Returns the pose's wrist target position
    public double getPoseWristTarget() {
        
        return wristPosition;
    }

}
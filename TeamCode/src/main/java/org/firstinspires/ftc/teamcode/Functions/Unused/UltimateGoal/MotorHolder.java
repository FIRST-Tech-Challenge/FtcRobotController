package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.robotcore.hardware.DcMotor;

// Scopul acestei clase e sa retina variabilele motoarelor pentru a fi mai usor sa initializezi clasele
// Move.cs sau Rotate.cs sau alte clase care folosesc motoarele de jos.
// Made by Vlad


// DEPRECATED USE Variables.MotorHolder instead
@Deprecated
public class MotorHolder {

    public DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;

    public MotorHolder(DcMotor _LMF, DcMotor _RMF, DcMotor _LMB, DcMotor _RMB){
        leftMotor = _LMF;
        rightMotor = _RMF;
        leftMotorBack = _LMB;
        rightMotorBack = _RMB;

    }

    // se poate initializa o clasa folosind clasa asta astfel:
    
    /*
    public NUME_CLASA(MotorHolder _MotorHolder){
        leftMotor = _MotorHolder.leftMotor;
        rightMotor = _MotorHolder.rightMotor;
        leftMotorBack = _MotorHolder.leftMotorBack;
        rightMotorBack = _MotorHolder.rightMotorBack;
        
    }
    */

}

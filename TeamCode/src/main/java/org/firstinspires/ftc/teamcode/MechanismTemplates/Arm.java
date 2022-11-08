package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Example class created by Tiernan demonstrating better OOP and PIDF usage
 */
public class Arm {
    private PIDFController armPIDF;
    private Motor armMotor;

    private final static double FORWARDS_POS = 100; // TODO: Change this to the actual position based on motor encoder readings
    private final static double BACKWARDS_POS = -100; // TODO: Change this to the actual position based on motor encoder readings
    private final double[] PIDF_COEFF = {0.0001, 0.001, 0.001, 0.001}; // TODO: Tune this PIDF

    private double targetPos;

    public Arm(HardwareMap hardwareMap){
        armMotor = new Motor(hardwareMap, "ARM", Motor.GoBILDA.RPM_60);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotor.setRunMode(Motor.RunMode.VelocityControl);
        //armMotor.setInverted(true); TODO: will have to check if we need to invert the motor

        armPIDF = new PIDFController(PIDF_COEFF[0], PIDF_COEFF[1], PIDF_COEFF[2], PIDF_COEFF[3]);
        targetPos = FORWARDS_POS;
    }

    public void update(){
        double correction = armPIDF.calculate(armMotor.getCurrentPosition(), targetPos);
        armMotor.set(correction);
    }

    public void setForwards(){
        targetPos = FORWARDS_POS;
    }

    public void setBackwards(){
        targetPos = BACKWARDS_POS;
    }

//    public void goToJunction(int target){
//        double currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average position
//
//        slideLeft.setTargetPosition(target);
//        slideRight.setTargetPosition(target);
//
//        if(Math.abs(currentPosition - target) > 5) {
//            slideLeft.setPower(motorPID.update(target, currentPosition));
//            slideRight.setPower(motorPID.update(target, currentPosition));
//
//            //currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average new position
//        }
//    }
//
//    // Auto
//    public void armPivot(int target){
//        double currentPosition = armMotor.getCurrentPosition(); // average motor position
//        armMotor.setTargetPosition(target);
//
//        if(Math.abs(currentPosition - target) > 5) {
//            armMotor.setPower(motorPID.update(target, currentPosition));
//           // currentPosition = armMotor.getCurrentPosition(); // average of new motor position
//        }
//    }
//
//    // TeleOp
//    public void manualArmPivot(int armIncrement){
//        double currentPosition = armMotor.getCurrentPosition();
//        int target = (int)currentPosition + armIncrement; // we want to set the target to just above/below the current position every time this runs
//        armMotor.setTargetPosition(target);
//
//        if(Math.abs(target - currentPosition) > 5) {
//            armMotor.setPower(motorPID.update(target, currentPosition));
//            //currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average new position
//        }
//    }
//
//    // TeleOp
//    public void manualClawRotate(double servoIncrement){
//        wristJoint.setPosition(wristJoint.getPosition() + servoIncrement);
//    }
//    // Auto
//    public void clawRotate(double servoTarget){
//        wristJoint.setPosition(servoTarget);
//    }
//
//    // TeleOp
//    public void manualSlides(int slideIncrement){
//        double currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average position
//        int target = (int)currentPosition + slideIncrement; // we want to set the target to just above/below the current position every time this loop runs
//
//        slideLeft.setTargetPosition(target);
//        slideRight.setTargetPosition(target);
//
//        if(Math.abs(target - currentPosition) > 5) {
//            slideLeft.setPower(motorPID.update(target, currentPosition));
//            slideRight.setPower(motorPID.update(target, currentPosition));
//
//            //currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; average new position
//        }
//    }
//
//    // Can be used in both teleOp and Autonomous (maybe)
//    public void claw(){
//        if(clawJoint.getPosition() == OPEN){
//            clawJoint.setPosition(CLOSE);
//        }else{
//            clawJoint.setPosition(OPEN);
//        }
//    }

}

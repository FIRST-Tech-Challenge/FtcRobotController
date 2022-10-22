package org.firstinspires.ftc.teamcode.MechanismTemplates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PIDs.PIDController;

public class ArmClass {

    PIDController motorPID;
    DcMotorEx slideLeft; DcMotorEx slideRight;
    DcMotorEx armMotor;
    Servo wristJoint; Servo clawJoint;
    double OPEN; double CLOSE;

    public ArmClass(PIDController mp, DcMotorEx sl, DcMotorEx sr, DcMotorEx a, Servo wj, Servo cj, double[] servoMinMax){
        this.motorPID = mp;
        this.slideLeft = sl;
        this.slideRight = sr;
        this.armMotor = a;
        this.wristJoint = wj;
        this.clawJoint = cj;
        this.OPEN = servoMinMax[0];
        this.CLOSE = servoMinMax[1];
    }

    // TeleOp and Auto
    public void goToJunction(int target){
        double currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average position

        slideLeft.setTargetPosition(target);
        slideRight.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            slideLeft.setPower(motorPID.update(target, currentPosition));
            slideRight.setPower(motorPID.update(target, currentPosition));

            currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average new position
        }
    }

    // Auto
    public void armPivot(int target){
        double currentPosition = armMotor.getCurrentPosition(); // average motor position
        armMotor.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            armMotor.setPower(motorPID.update(target, currentPosition));
            currentPosition = armMotor.getCurrentPosition(); // average of new motor position
        }
    }

    // TeleOp
    public void manualArmPivot(int armIncrement){
        double currentPosition = armMotor.getCurrentPosition();
        int target = (int)currentPosition + armIncrement; // we want to set the target to just above/below the current position every time this runs

        armMotor.setTargetPosition(target);

        while(Math.abs(target - currentPosition) > 5) {
            armMotor.setPower(motorPID.update(target, currentPosition));
            currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average new position
        }
    }

    // TeleOp
    public void manualClawRotate(double servoIncrement){
        wristJoint.setPosition(wristJoint.getPosition() + servoIncrement);
    }
    // Auto
    public void clawRotate(double servoTarget){
        wristJoint.setPosition(servoTarget);
    }

    // TeleOp
    public void manualSlides(int slideIncrement){
        double currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average position
        int target = (int)currentPosition + slideIncrement; // we want to set the target to just above/below the current position every time this loop runs

        slideLeft.setTargetPosition(target);
        slideRight.setTargetPosition(target);

        while(Math.abs(target - currentPosition) > 5) {
            slideLeft.setPower(motorPID.update(target, currentPosition));
            slideRight.setPower(motorPID.update(target, currentPosition));

            currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average new position
        }
    }

    // Can be used in both teleOp and Autonomous (maybe)
    public void claw(){
        if(clawJoint.getPosition() == OPEN){
            clawJoint.setPosition(CLOSE);
        }else{
            clawJoint.setPosition(OPEN);
        }
    }

}

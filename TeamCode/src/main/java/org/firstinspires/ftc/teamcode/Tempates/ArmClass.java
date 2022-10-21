package org.firstinspires.ftc.teamcode.Tempates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PIDs.PIDController;

public class ArmClass {

    public ArmClass(){

    }

    public static void goToJunction(PIDController motorPID, DcMotorEx slideLeft, DcMotorEx slideRight, int target){
        double currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average position

        slideLeft.setTargetPosition(target);
        slideRight.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            slideLeft.setPower(motorPID.update(target, currentPosition));
            slideRight.setPower(motorPID.update(target, currentPosition));

            currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average new position
        }

    }

    public static void armPivot(PIDController motorPID, DcMotorEx armMotor, int target){
        double currentPosition = armMotor.getCurrentPosition(); // average position
        armMotor.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            armMotor.setPower(motorPID.update(target, currentPosition));
            currentPosition = armMotor.getCurrentPosition(); // average new position
        }
    }

    public static void clawRotate(Servo wristJoint, double increment){
        wristJoint.setPosition(wristJoint.getPosition() + increment);
    }

    public static void slides(PIDController motorPID, DcMotorEx slideLeft, DcMotorEx slideRight,  int increment){
        double currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average position
        int target = (int)currentPosition + increment; // we want to set the target to just above the current position every time this loop runs

        slideLeft.setTargetPosition(target + increment);
        slideRight.setTargetPosition(target + increment);

        while(Math.abs(target - currentPosition) > 5) {
            slideLeft.setPower(motorPID.update(target, currentPosition));
            slideRight.setPower(motorPID.update(target, currentPosition));

            currentPosition = (slideLeft.getCurrentPosition() + slideRight.getCurrentPosition())/2.0; // average new position
        }
    }

    public static void claw(Servo clawJoint, double OPEN, double CLOSE){
        if(clawJoint.getPosition() == OPEN){
            clawJoint.setPosition(CLOSE);
        }else{
            clawJoint.setPosition(OPEN);
        }
    }

}

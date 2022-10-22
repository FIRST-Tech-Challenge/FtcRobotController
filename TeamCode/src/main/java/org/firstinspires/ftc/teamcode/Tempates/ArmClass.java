package org.firstinspires.ftc.teamcode.Tempates;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PIDs.PIDController;

public class ArmClass {

    PIDController motorPID;
    DcMotorEx slideLeft; DcMotorEx slideRight;
    DcMotorEx armMotor;
    Servo wristJoint; Servo clawJoint;
    double OPEN; double CLOSE;

    public ArmClass(PIDController mp, DcMotorEx sl, DcMotorEx sr, DcMotorEx a, Servo wj, Servo cj, double[] oc){
        this.motorPID = mp;
        this.slideLeft = sl;
        this.slideRight = sr;
        this.armMotor = a;
        this.wristJoint = wj;
        this.clawJoint = cj;
        this.OPEN = oc[0];
        this.CLOSE = oc[1];
    }

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

    public void armPivot(int target){
        double currentPosition = armMotor.getCurrentPosition(); // average position
        armMotor.setTargetPosition(target);

        while(Math.abs(currentPosition - target) > 5) {
            armMotor.setPower(motorPID.update(target, currentPosition));
            currentPosition = armMotor.getCurrentPosition(); // average new position
        }
    }

    public void clawRotate(double increment){
        wristJoint.setPosition(wristJoint.getPosition() + increment);
    }

    public void slides(int increment){
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

    public void claw(){
        if(clawJoint.getPosition() == OPEN){
            clawJoint.setPosition(CLOSE);
        }else{
            clawJoint.setPosition(OPEN);
        }
    }

}

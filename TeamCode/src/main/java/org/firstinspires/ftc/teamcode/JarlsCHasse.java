package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class JarlsCHasse {

    DcMotorEx RMFront = null;
    DcMotorEx LMFront = null;
    DcMotorEx RMBack = null;
    DcMotorEx LMBack = null;

    /*Variables used for Autonomus moving stats*/
    double Xmov = 0.0;
    double Ymov = 0.0;
    double rXmov = 0.0;

    double x;
    double y;

    double max;

    JarlsCHasse(HardwareMap hwMap){
        RMFront = hwMap.get(DcMotorEx.class, "rightFront");
        LMFront = hwMap.get(DcMotorEx.class, "leftFront");
        RMBack = hwMap.get(DcMotorEx.class, "rightRear");
        LMBack = hwMap.get(DcMotorEx.class, "leftRear");

        RMFront.setDirection(DcMotorEx.Direction.REVERSE);
        LMBack.setDirection(DcMotorEx.Direction.REVERSE);

        RMFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RMBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LMBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveY(boolean sign, double rotation){

        if(sign) {
            Xmov = (1 * Math.cos(rotation) - (0 * Math.sin(rotation)));
        }
        else{
            Xmov = -(0 * Math.cos(rotation) - (1 * Math.sin(rotation)));
        }

    }

    public void moveX(boolean sign, double rotation){

        if(sign) {
            Ymov = (0 * Math.sin(rotation) + (1 * Math.cos(rotation)));
        }
        else{
            Ymov = -(0 * Math.sin(rotation) + (1 * Math.cos(rotation)));
        }


    }

    public void turn0Clockwise(boolean sign){

        if(sign) {
            rXmov = 1;
        }
        else{
            rXmov = -1;
        }

    }

    public void GamepadInputs(double rotation, Gamepad gmpad){

        float xt = gmpad.left_stick_x;
        float yt = -gmpad.left_stick_y;
        float rx = gmpad.right_stick_x;

        x = (xt * Math.cos(rotation) - (yt * Math.sin(rotation)));
        y = (xt * Math.sin(rotation) + (yt * Math.cos(rotation)));


        double leftFrontPower = y+x+rx;
        double rightFrontPower = y-x-rx;
        double leftBackPower = y-x+rx;
        double rightBackPower = y+x-rx;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }


        LMFront.setPower(leftFrontPower);
        RMFront.setPower(rightFrontPower);
        LMBack.setPower(leftBackPower);
        RMBack.setPower(rightBackPower);

    }

    public void coordinateBasedState(){
        RMFront.setPower(-Xmov+Ymov-rXmov);
        LMFront.setPower(Xmov+Ymov+rXmov);
        RMBack.setPower(Xmov+Ymov-rXmov);
        LMBack.setPower(-Xmov+Ymov+rXmov);
    }

    public void HALT(){
        Xmov = 0;
        Ymov = 0;
        rXmov = 0;
    }


}

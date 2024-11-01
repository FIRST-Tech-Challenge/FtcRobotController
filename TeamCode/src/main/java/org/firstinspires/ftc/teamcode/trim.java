package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;

public class trim {
    private float totalTrim = 1f;
    private float leftTrim = 1f;
    private float rightTrim = 1f;
    private final float trimAmount = 0.01f;

    public float trimLeft() {
        return leftTrim;
    }

    public float trimRight() {
        return rightTrim;
    }

    private void Trim(float amount) {
        totalTrim += amount;
        if (totalTrim < 1) {
            rightTrim = 1;
            leftTrim=totalTrim;
        }else if(totalTrim>1){
            leftTrim=1;
            rightTrim=totalTrim-1;
        }else {
            leftTrim=1;
            rightTrim=1;
        }
        if (leftTrim<0){
            leftTrim=0;
            totalTrim=0;
        }
        if (rightTrim<0){
            rightTrim=0;
            totalTrim=2;
        }
    }
    public void update(){
        if (gamepad1.dpad_left){
            Trim(-trimAmount);
        }
        if (gamepad1.dpad_right){
            Trim(trimAmount);
        }
    }
}
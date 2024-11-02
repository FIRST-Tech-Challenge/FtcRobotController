package org.firstinspires.ftc.teamcode;
/*
To get the trim to work
1) multiply the left motor by the value of the left trim using the method trimLeft() wherever you want to trim it
2) multiply the right motor by the value of the right trim using the method trimRight() wherever you want to trim it
3) call the update() method before you first move your motors in your while (opModeIsActive) loop
*/

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

// Please work.
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveMethods {
    public void driveForDistance(double distanceMeters, double power) {
        motor01.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor02.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor03.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor04.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) (distanceMeters * encoders * rotationsPerMeter);
        motor01.setTargetPosition((targetPos));
        motor02.setTargetPosition((targetPos));
        motor03.setTargetPosition((targetPos));
        motor04.setTargetPosition((targetPos));


        motor01.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor02.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor03.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor04.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        motor01.setPower(power);
        motor02.setPower(power);
        motor03.setPower(power);
        motor04.setPower(power);
        targetPos = motor01.getTargetPosition();
        int currentPos = motor01.getCurrentPosition();
        boolean hasNotReachedTarget = true;
        while (hasNotReachedTarget) {
            if(currentPos == targetPos){
                hasNotReachedTarget = false;
                motor01.setPower(0);
                motor02.setPower(0);
                motor03.setPower(0);
                motor04.setPower(0);
            }
        }
    }
}

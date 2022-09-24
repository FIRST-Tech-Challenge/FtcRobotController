package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.Variables.*;


public class DriveMethods  {
    public static void driveForDistance(double distanceMeters, boolean doStrafe, double power) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double distanceTraveled = 0;
        int targetPos = (int) (distanceMeters * clicksPerRotation * rotationsPerMeter);
        motorFL.setTargetPosition((targetPos));
        motorBL.setTargetPosition((targetPos));
        motorFR.setTargetPosition((targetPos));
        motorBR.setTargetPosition((targetPos));



        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (doStrafe) {
            motorFL.setPower(power);
            motorBL.setPower(-power);
            motorFR.setPower(-power);
            motorBR.setPower(power);
        } else {
            motorFL.setPower(power);
            motorBL.setPower(power);
            motorFR.setPower(power);
            motorBR.setPower(power);
        }
        targetPos = motorFL.getTargetPosition();
        int currentPos = motorFL.getCurrentPosition();
        boolean hasNotReachedTarget = true;
        while (hasNotReachedTarget) {
            if(currentPos == targetPos){
                hasNotReachedTarget = false;
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }
        }
    }
}

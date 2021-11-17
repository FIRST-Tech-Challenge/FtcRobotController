package org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide {

    private final DcMotor linearSlide;


    private static final double motorPower = 0.5;
    private static final double ticksPerRevolution = 145.1;
    private static final double spoolRadius = 0.55 / 2; //in inches
    private static final double inchesPerRevolution = ticksPerRevolution * 2 * Math.PI * spoolRadius;
    private static final int linearSlideAngle = 68;
    private static final double stopPower = 0.13;

    public static final double level_one_height = 0;

    // TODO Use run to position with values for zero power break behavior

    public LinearSlide(HardwareMap hardwareMap, String dcMotorName) {
        linearSlide = hardwareMap.dcMotor.get(dcMotorName);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getEncoderCount(){
        return linearSlide.getCurrentPosition();
    }

    public void setMotorPower(double power) {
        linearSlide.setPower(power);
    }


    public void goUp() {
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setPower(motorPower);

    }

    public void goDown() {
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setPower(-motorPower);
    }

    public void stop() {
        linearSlide.setPower(stopPower);
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

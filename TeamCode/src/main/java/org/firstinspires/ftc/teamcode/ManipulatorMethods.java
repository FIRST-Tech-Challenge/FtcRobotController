package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ManipulatorMethods {
    Hardware robot;

    public ManipulatorMethods(Hardware r) {
        robot = r;
    }

    public void moveSlideEncoder(int height, double speed) {
        robot.rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rightSlides.setTargetPosition(height);
        robot.leftSlides.setTargetPosition(height);

        robot.rightSlides.setPower(speed);
        robot.leftSlides.setPower(speed);

        robot.rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveSlideEncoder(int leftHeight, int rightHeight, double speed) {
        robot.rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rightSlides.setTargetPosition(rightHeight);
        robot.leftSlides.setTargetPosition(leftHeight);


        robot.rightSlides.setPower(speed);
        robot.leftSlides.setPower(speed);

        robot.rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetSlides() {
        robot.rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void moveSlides(double speed) {
        robot.rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightSlides.setPower(speed);
        robot.leftSlides.setPower(speed);
    }
    public void stopSlides() {
        robot.rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.rightSlides.setPower(0);
        robot.leftSlides.setPower(0);
    }

    public void intake(){
        robot.rightIntake.setPower(-1);
        robot.leftIntake.setPower(1);

    }
    public void outtake(){
        robot.rightIntake.setPower(1);
        robot.leftIntake.setPower(-1);
    }
    public void stopIntake(){
        robot.rightIntake.setPower(0);
        robot.leftIntake.setPower(0);
    }
}

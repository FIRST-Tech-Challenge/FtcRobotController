package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ManipulatorMethods {
    public DcMotor rightSlides = null;
    public DcMotor leftSlides = null;

    public CRServo leftIntake = null;
    public CRServo rightIntake = null;

    public DistanceSensor distanceSensor = null;


    // Other variable names

    public ManipulatorMethods(HardwareMap hwMap) {
        rightSlides = hwMap.dcMotor.get("rightSlides");

        leftSlides = hwMap.dcMotor.get("leftSlides");


        // Define Servos
        leftIntake = hwMap.crservo.get("leftIntake");

        rightIntake = hwMap.crservo.get("rightIntake");

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing

        //Subsystem Motors & Servos
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setDirection(DcMotorSimple.Direction.FORWARD);

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //SENSORS
        distanceSensor = hwMap.get(DistanceSensor.class, "distance");

        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveSlideEncoder(int height, double speed) {
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlides.setTargetPosition(height);
        leftSlides.setTargetPosition(height);

        rightSlides.setPower(speed);
        leftSlides.setPower(speed);

        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveSlideEncoder(int leftHeight, int rightHeight, double speed) {
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlides.setTargetPosition(rightHeight);
        leftSlides.setTargetPosition(leftHeight);

        rightSlides.setPower(speed);
        leftSlides.setPower(speed);

        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetSlides() {
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void moveSlides(double speed) {
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlides.setPower(speed);
        leftSlides.setPower(speed);
    }
    public void stopSlides() {
        rightSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightSlides.setPower(0);
        leftSlides.setPower(0);
    }

    public void intake(){
        rightIntake.setPower(-1);
        leftIntake.setPower(1);

    }
    public void outtake(){
        rightIntake.setPower(1);
        leftIntake.setPower(-1);
    }
    public void stopIntake(){
        rightIntake.setPower(0);
        leftIntake.setPower(0);
    }
}

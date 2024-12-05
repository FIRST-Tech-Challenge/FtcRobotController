package org.firstinspires.ftc.teamcode.functions;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SlideFunctions {

    public DcMotor rightSlideMotor;
    public DcMotor leftSlideMotor;
    public TouchSensor slideSafety;

    public SlideFunctions(HardwareMap hardwareMap) {

        rightSlideMotor = hardwareMap.get(DcMotor.class, "right_slide_motor");
        leftSlideMotor = hardwareMap.get(DcMotor.class, "left_slide_motor");
        slideSafety = hardwareMap.get(TouchSensor.class,"slide_safety");
        rightSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        leftSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        //reset encoders
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //everything brakes at 0
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void SlideControl(Gamepad gamepad2, Telemetry telemetry){

        double slidePowerConst = 0.9; //max power of slide
        double slidePower = -gamepad2.left_stick_y;

        //get position of slides
        int rightSlidePosition = rightSlideMotor.getCurrentPosition();
        int leftSlidePosition = leftSlideMotor.getCurrentPosition();

        //slide safety
        // Gleb here, this works compeltely off of encoder values because the touch sensor is not setup properly yet.
        if ((rightSlidePosition <= -22850) && slidePower > 0 || (rightSlidePosition >= -100) && slidePower < 0) {
            slidePower = 0;
            telemetry.addData("Slide Safety is working", slideSafety);
        }

        rightSlideMotor.setPower(slidePower * slidePowerConst);
        leftSlideMotor.setPower(slidePower * slidePowerConst);
        //telemetry poo
        telemetry.addData("Slide power","%4.2f", slidePower);
        telemetry.addData("Right Slide Position", rightSlidePosition);
        telemetry.addData("Left Slide Position", leftSlidePosition);
    }

    public void ArmControl(Gamepad gamepad2, Telemetry telemetry) {
        double armPower = -gamepad2.right_stick_y;
        leftSlideMotor.setPower(armPower);

        int armPosition = leftSlideMotor.getCurrentPosition();

        telemetry.addData("Left Slide power","%4.2f", armPower);
        telemetry.addData("Left Slide Position", armPosition);
    }

}

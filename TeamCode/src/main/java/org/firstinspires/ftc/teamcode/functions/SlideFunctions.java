package org.firstinspires.ftc.teamcode.functions;

import android.text.method.Touch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SlideFunctions {

    public DcMotor slideMotor;
    public DcMotor armMotor;
    public TouchSensor slideSafety;

    public SlideFunctions(HardwareMap hardwareMap) {

        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        slideSafety = hardwareMap.get(TouchSensor.class,"slide_safety");
        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        //reset encoders
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //everything brakes at 0
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void SlideControl(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){

        double slidePowerConst = 0.9; //max power of slide
        double slidePower = -gamepad2.left_stick_y;

        //get position of slides
        int slidePosition= slideMotor.getCurrentPosition();

        //slide safety
        if ((slidePosition >= 1000) && (slidePower>0)|| slideSafety.isPressed() && slidePower<0){
            slideMotor.setPower(0);
        } else if (slideSafety.isPressed() && slidePower<0 ){
            slideMotor.setPower(0);
        } else {
            slideMotor.setPower(slidePower * slidePowerConst);
        }
        //telemetry poo
        telemetry.addData("Slide power","%4.2f", slidePower);
        telemetry.addData("Slide Position", slidePosition);
    }

    public void ArmControl(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        double armPower = gamepad2.right_stick_y;
        armMotor.setPower(armPower);

        int armPosition = armMotor.getCurrentPosition();

        telemetry.addData("Arm power","%4.2f", armPower);
        telemetry.addData("Arm Position", armPosition);
    }

}

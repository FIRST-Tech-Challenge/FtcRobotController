package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class SlideFunctions {

    public DcMotor slideMotor;
    public DcMotor armMotor;

    public SlideFunctions(HardwareMap hardwareMap) {

        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");

        slideMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD);
    }


    public void SlidePosition(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry){

        double slidePowerConst = 0.7; //max power of slide
        double slidePower = -gamepad2.left_stick_y;

        //insert slide safety stuff here >.<

        slideMotor.setPower(slidePower * slidePowerConst);

        //display of power and position of motor
        int slidePosition= slideMotor.getCurrentPosition();
        telemetry.addData("Slide power","%4.2f", slidePower);
        telemetry.addData("Slide Position", slidePosition);

    }

    public void ArmPosition(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        double armPower = gamepad2.right_stick_y;
        armMotor.setPower(armPower);

        int armPosition = armMotor.getCurrentPosition();

        telemetry.addData("Arm power","%4.2f", armPower);
        telemetry.addData("Arm Position", armPosition);
    }

}

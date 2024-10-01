package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Joshua Linear Slide")
public class linearSlidesTest extends OpMode {
//op mode -> real time control, linearOp -> preset commands (auto)
    private DcMotor linearSlide1;
    private DcMotor linearSlide2;
    public ElapsedTime runtime;

    @Override
    public void init(){
        linearSlide1 = hardwareMap.get(DcMotor.class, "slide1");
        linearSlide2 = hardwareMap.get(DcMotor.class, "slide2");

        linearSlide1.setDirection(DcMotor.Direction.REVERSE); //how it should determine positive/negative positions
        linearSlide2.setDirection(DcMotor.Direction.REVERSE);

        runtime = new ElapsedTime();
        runtime.reset(); //set the counter for runtime to be 0

    }

    @Override
    public void loop() {
        double currentPosition = 0.0;

        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Position", linearSlide1.getCurrentPosition() );
        telemetry.update();
        //print out the data of the linear slide position on the game hub


        double power = Math.abs(gamepad2.left_stick_y);

        // remember it is reversed -> init
        if (gamepad2.left_stick_y < -0.1 && linearSlide1.getCurrentPosition()< 3000) {

            linearSlide1.(power);
            linearSlide2.setPower(power);
        }

        else if (gamepad2.left_stick_y > 0.1 && linearSlide1.getCurrentPosition() > 3) {

            linearSlide1.setPower(-power);
            linearSlide2.setPower(-power);
        }
        else {
            linearSlide1.setPower(0.05); //try 0
            linearSlide2.setPower(0.05);
        }

    }
}

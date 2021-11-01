package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Slide Test", group="tests")
public class LinSlideTest extends OpMode {

    private LinearSlide slide;
    private float slowdownModifierp1;

    @Override
    public void init() {
        DcMotor slide = hardwareMap.dcMotor.get("arm");
        this.slide = new LinearSlide(slide, 0.0f, 360.0f);
    }

    @Override
    public void loop() {
        this.slowdownModifierp1 = 1 - (gamepad1.right_trigger * 0.85f);
        slide.MoveSlideUnrestricted(gamepad1.left_stick_y * slowdownModifierp1);
    }
}

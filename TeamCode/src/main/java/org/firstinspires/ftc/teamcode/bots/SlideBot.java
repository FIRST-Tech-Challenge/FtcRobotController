package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SlideBot extends BotBot{
    public DcMotor slide = null;
    public Servo dropper = null;

    public int slidePosition = 0;

    final double dropperUp = 0.65;
    final double dropperDown = 0.1;

    boolean isDrop = false;
    boolean isExtended = false;

    long timeSinceToggle = 0;
    long lastToggleDone = 0;
    long timeSinceToggle1 = 0;
    long lastToggleDone1 = 0;

    public SlideBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        slide = hwMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setPower(0);

        dropper = hwMap.get(Servo.class, "dropper");
        dropper.setPosition(dropperUp);
    }

    public void slideUp(boolean button) {
        if (button && slidePosition<23){
            slide.setPower(1);
            slidePosition++;
        } else {
            slide.setPower(0);
        }
    }
    public void slideDown(boolean button) {
        if (button && slidePosition>0){
            slide.setPower(-1);
            slidePosition--;
        } else {
            slide.setPower(0);
        }
    }
    public void slideControl(float down, float up) {
        if (up > 0){
            slide.setPower(up);
        } else if (down > 0){
            slide.setPower(-down);
        } else {
            slide.setPower(0);
        }
    }

    public void slideToggle(boolean button) {
        timeSinceToggle1 = System.currentTimeMillis() - lastToggleDone1;
        if (button && timeSinceToggle > 500) {
            if (isExtended) {
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.4);
                isExtended = false;
                lastToggleDone1 = System.currentTimeMillis();
            } else if (!isExtended) {
                slide.setTargetPosition(-1990);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.4);
                isExtended = true;
                lastToggleDone1 = System.currentTimeMillis();
            }
        } else if (!slide.isBusy()) {
            slide.setPower(0);
        }
    }

    public void toggleDropper(boolean toggle) {
        timeSinceToggle = System.currentTimeMillis() - lastToggleDone;
        if (toggle && timeSinceToggle > 300) {
            if (isDrop) {
                dropper.setPosition(dropperUp);
                isDrop = false;
                lastToggleDone = System.currentTimeMillis();
            } else if (!isDrop) {
                dropper.setPosition(dropperDown);
                isDrop = true;
                lastToggleDone = System.currentTimeMillis();
            }
        }
    }

    protected void onTick() {
        opMode.telemetry.addData("position:", slide.getCurrentPosition());
        opMode.telemetry.update();
        super.onTick();

    }
}

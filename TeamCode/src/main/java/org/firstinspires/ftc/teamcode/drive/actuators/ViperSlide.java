package org.firstinspires.ftc.teamcode.drive.actuators;

import static com.qualcomm.robotcore.hardware.Servo.MIN_POSITION;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
public class ViperSlide extends OpMode{
    DcMotor poliaright;
    DcMotor polialeft;
    double ticks = 2800.5;
    double newTarget;

    @Override
    public void init(){
        poliaright = hardwareMap.get(DcMotor.class, "poliaright");
        telemetry.addData("Hardware: ", "Initialized");
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        poliaright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        polialeft = hardwareMap.get(DcMotor.class, "polialeft");
        poliaright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        polialeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_up){
            viperslide1Up(-1);
            viperslide2Up(1);
        }
        if (gamepad1.dpad_left){
            viperslide1Up(-3);
            viperslide2Up(3);
        }
        if (gamepad2.dpad_down) {
            viperslide2Down();
            viperslide1Down();
        }
        int currentPosition1 = poliaright.getCurrentPosition();
        int currentPosition2 = polialeft.getCurrentPosition();

        if (currentPosition1 <= MIN_POSITION) {
            poliaright.setPower(0);
        }
        if (currentPosition2 <= MIN_POSITION){
            polialeft.setPower(0);
        }
    }
    public void viperslide1Up(int turnage) {
        newTarget = ticks / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Up(int turnage) {
        newTarget = ticks / turnage;
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide1Down() {
        poliaright.setTargetPosition(0);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslide2Down() {
        polialeft.setTargetPosition(0);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
package org.firstinspires.ftc.teamcode.drive.actuators;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
public class ViperSlide extends OpMode{
    DcMotor viperslide;
    double ticks = 2800.5;
    double newTarget;
    @Override
    public void init(){
        viperslide = hardwareMap.get(DcMotor.class, "viperslide");
        telemetry.addData("Hardware: ", "Initialized");
        viperslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_down){
            viperslideDown();
            sleep(2000);
            viperslide.setPower(0);
        }
        if (gamepad1.dpad_up){
            viperslideUp(1);
        }
    }
    public void viperslideUp(int turnage) {
        newTarget = ticks / turnage;
        viperslide.setTargetPosition((int) newTarget);
        viperslide.setPower(0.8);
        viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslideDown() {
        viperslide.setTargetPosition(0);
        viperslide.setPower(0.8);
        viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
package org.firstinspires.ftc.teamcode.drive.actuators;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
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
        if (gamepad1.dpad_down){
            viperslidesDown();
            sleep(2000);
            poliaright.setPower(0);
        }
        if (gamepad1.dpad_up){
            viperslidesUp(1);
        }
    }
    public void viperslidesUp(int turnage) {
        newTarget = ticks / turnage;
        poliaright.setTargetPosition((int) newTarget);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        polialeft.setTargetPosition((int) newTarget);
        polialeft.setPower(-1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void viperslidesDown() {
        poliaright.setTargetPosition(0);
        poliaright.setPower(1);
        poliaright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        polialeft.setTargetPosition(0);
        polialeft.setPower(-1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
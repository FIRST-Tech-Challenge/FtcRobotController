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
    private boolean isPoliaMoving = false;
    private long poliaStartTime = 0;
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
        if (gamepad1.dpad_right){
            viperslide1Down();
            viperslide2Down();
        }
        // HANDLING POLIA TIMING
        if (isPoliaMoving) {
            if (System.currentTimeMillis() - poliaStartTime > 300) {
            }
            if (System.currentTimeMillis() - poliaStartTime > 1550) {
                poliaright.setPower(0);
                polialeft.setPower(0);
                isPoliaMoving = false; // Reset flag when done
            }
        }
        // OTHER CONTROLS
        if (gamepad2.dpad_down && !isPoliaMoving) {
            viperslide2Down();
            viperslide1Down();
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

        isPoliaMoving = true; // Flag to indicate polia movement
        poliaStartTime = System.currentTimeMillis(); // Store start time
    }
    public void viperslide2Down() {
        polialeft.setTargetPosition(0);
        polialeft.setPower(1);
        polialeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        isPoliaMoving = true; // Flag to indicate polia movement
        poliaStartTime = System.currentTimeMillis(); // Store start time
    }
}
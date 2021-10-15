package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.EncoderMotor;

import kotlin.jvm.internal.MagicApiIntrinsics;

@TeleOp(name="ControllerTeleOp", group="Iterative")
public class ControllerTeleOp extends OpMode {

    private final ElapsedTime TIME = new ElapsedTime();
    private EncoderMotor spinner, motor0, motor1, motor2, motor3;
    private float vertical, horizontal, pivot;
//    private DcMotor motor0, motor1, motor2, motor3;

    /**
     * Code to run once when the OpMode is initialized.
     */
    @Override
    public void init() {
        spinner = new EncoderMotor(telemetry, hardwareMap, "Spinner", DcMotorSimple.Direction.REVERSE, 400, 1, 1);
        motor0 = new EncoderMotor(telemetry, hardwareMap, "DcMotor", DcMotorSimple.Direction.FORWARD, 400, 0, 0);
        motor1 = new EncoderMotor(telemetry, hardwareMap, "DcMotor 1", DcMotorSimple.Direction.FORWARD, 400, 0, 0);
        motor2 = new EncoderMotor(telemetry, hardwareMap, "DcMotor 2", DcMotorSimple.Direction.FORWARD, 400, 0, 0);
        motor3 = new EncoderMotor(telemetry, hardwareMap, "DcMotor 3", DcMotorSimple.Direction.FORWARD, 400, 0, 0);
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to loop between the end of init() and beginning of start().
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run after hitting play.
     */
    @Override
    public void start() {
        TIME.reset();
    }

    /*
     * Code to run after start() ends.
     */
    @Override
    public void loop() {
        if(gamepad1.x) {
            spinner.driveWithEncoder(50);
        }else{
            spinner.driveWithEncoder(0);
        }
        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;
        motor3.driveWithEncoder((int) (-pivot + (vertical - horizontal) * 100));
        motor2.driveWithEncoder((int) (-pivot + vertical + horizontal * 100));
        motor1.driveWithEncoder((int) (pivot + (vertical - horizontal) * 100));
        motor0.driveWithEncoder((int) (pivot + vertical + horizontal * 100));
    }

    /*
     * Code to run once stop is pressed, or once the time runs out.
     */
    @Override
    public void stop() {
        spinner.stop();
        motor0.stop();
        motor1.stop();
        motor2.stop();
        motor3.stop();
        telemetry.addData("Status", "Completed");
    }

}

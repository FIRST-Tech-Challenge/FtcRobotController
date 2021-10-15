package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Motor;

import kotlin.jvm.internal.MagicApiIntrinsics;

@TeleOp(name="TemplateOpMode", group="Iterative")
public class ControllerTeleOp extends OpMode {

    private final ElapsedTime TIME = new ElapsedTime();
    private Motor spinner;

    /**
     * Code to run once when the OpMode is initialized.
     */
    @Override
    public void init() {
        spinner = new Motor(telemetry, hardwareMap, "spinner", DcMotorSimple.Direction.FORWARD, 400, 1, 1);
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
    }

    /*
     * Code to run once stop is pressed, or once the time runs out.
     */
    @Override
    public void stop() {
        spinner.stop();
        telemetry.addData("Status", "Completed");
    }

}

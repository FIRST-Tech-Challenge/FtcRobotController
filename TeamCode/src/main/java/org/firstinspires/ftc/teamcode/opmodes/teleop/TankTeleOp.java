package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.ButtonPriority;
import org.firstinspires.ftc.teamcode.utils.GamepadExtended;
import org.firstinspires.ftc.teamcode.utils.Motor;
import org.firstinspires.ftc.teamcode.utils.Tank;

@TeleOp(name="TankTeleOp", group="Iterative")
public class TankTeleOp extends OpMode {

    private final ElapsedTime TIME = new ElapsedTime();
    private Motor rightTop, rightBottom, leftTop, leftBottom, spinner, elevator, grabber;
    private Tank tank;
    private ButtonPriority PRIORITY = new ButtonPriority();

    /**
     * Code to run once when the OpMode is initialized.
     */
    @Override
    public void init() {
        rightTop = new Motor(telemetry, hardwareMap, "rd1", DcMotorSimple.Direction.FORWARD);
        rightBottom = new Motor(telemetry, hardwareMap, "rd2", DcMotorSimple.Direction.FORWARD);
        leftTop = new Motor(telemetry, hardwareMap, "ld1", DcMotorSimple.Direction.REVERSE);
        leftBottom = new Motor(telemetry, hardwareMap, "ld2", DcMotorSimple.Direction.REVERSE);
        spinner = new Motor(telemetry, hardwareMap, "spinner", DcMotorSimple.Direction.FORWARD);
        elevator = new Motor(telemetry, hardwareMap, "elevator", DcMotorSimple.Direction.FORWARD, 1400, 1, 1);
        grabber = new Motor(telemetry, hardwareMap, "grabber", DcMotorSimple.Direction.FORWARD);
        tank = new Tank(telemetry, rightTop, rightBottom, leftTop, leftBottom);
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
        double left = gamepad1.left_stick_x + gamepad1.left_stick_y * 100.0;
        double right = gamepad1.left_stick_x - gamepad1.left_stick_y * 100.0;
        tank.driveWithEncoder((int) right, (int) left);
    }

    /*
     * Code to run once stop is pressed, or once the time runs out.
     */
    @Override
    public void stop() {
        tank.stop();
        spinner.stop();
        elevator.stop();
        grabber.stop();
    }

}

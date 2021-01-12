package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivecontrol.Position;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2D;

@TeleOp(name="test", group="pushBot")
public class TestOp extends OpMode {
    private static final boolean debug = false;

    private Robot robot;
    private boolean resetIMUOnStart = true;

    private boolean slowMode = false;

    private static final double DEAD_BAND_MAG_SLOW = 0.03;
    private static final double DEAD_BAND_MAG = 0.1;

    @Override
    public void init() {
        robot = new Robot(debug, new Position(0, 0), telemetry, hardwareMap);
    }

    public void init_loop() {
        if (gamepad1.y) {
            resetIMUOnStart = false;
        }
    }

    @Override
    public void start() {
        super.start();
        if (resetIMUOnStart) {
            robot.initIMU();
        }
    }

    @Override
    public void loop() {
        robot.updateDrivePositionTracking();
        slowMode = false;

        Vector2D leftStick1 = new Vector2D(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        Vector2D rightStick1 = new Vector2D(gamepad1.right_stick_x, -gamepad1.right_stick_y);

        if (gamepad1.left_trigger > 0.1) {
            leftStick1 = leftStick1.scale((1-Math.abs(gamepad1.left_trigger))*.75);
            rightStick1 = rightStick1.scale(1-Math.abs(gamepad1.left_trigger));
        }

        robot.updateUsingJoysticks(
                checkDeadBand(leftStick1).scale(Math.sqrt(2)),
                checkDeadBand(rightStick1).scale(Math.sqrt(2))
        );
    }

    private Vector2D checkDeadBand(Vector2D j) {
        return j.getMagnitude() > (slowMode ? DEAD_BAND_MAG_SLOW : DEAD_BAND_MAG)
                ? j : new Vector2D(0, 0);
    }

    @Override
    public void stop() {
        super.stop();
    }
}

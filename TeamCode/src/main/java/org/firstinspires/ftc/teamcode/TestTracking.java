package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Test Tracking", group = "TeleOp")
public class TestTracking extends OpMode {
    Robot robot;
    public double DEADBAND_MAG = 0.1;
    public Vector2d DEADBAND_VEC = new Vector2d(DEADBAND_MAG, DEADBAND_MAG);
    public boolean willResetIMU = true;

    double loopStartTime = 0;
    double loopEndTime = 0;

    public void init() {
        robot = new Robot(this, false, false);
    }

    public void init_loop() {
        if (gamepad1.y) {
            willResetIMU = false;
        }
    }

    public void start () {
        if (willResetIMU) robot.initIMU();
    }

    public void loop() {
        loopStartTime = System.currentTimeMillis();
        telemetry.addData("OS loop time: ", loopEndTime - loopStartTime);

        Vector2d joystick1 = new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y); //LEFT joystick
        Vector2d joystick2 = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y); //RIGHT joystick

        robot.driveController.updateUsingJoysticks(checkDeadband(joystick1).scale(Math.sqrt(2)), checkDeadband(joystick2).scale(Math.sqrt(2)), true);

        robot.updateBulkData(); //read data once per loop, access it through robot class variable
        robot.driveController.updatePositionTracking(telemetry);

        loopEndTime = System.currentTimeMillis();
        telemetry.addData("Our loop time: ", loopEndTime - loopStartTime);

        telemetry.update();
    }

    public Vector2d checkDeadband(Vector2d joystick) {
        if (Math.abs(joystick.getX()) > DEADBAND_VEC.getX() || Math.abs(joystick.getY()) > DEADBAND_VEC.getY()) {
            return joystick;
        }
        return new Vector2d(0, 0);
    }
}
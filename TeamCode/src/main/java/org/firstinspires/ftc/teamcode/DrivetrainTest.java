package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp
public class DrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            final float omega = -gamepad1.right_trigger + gamepad1.left_trigger;
            final Vector joystick = new Vector(gamepad1.left_stick_x,-gamepad1.left_stick_y);
            Drivetrain.operate(joystick,omega);
        }
    }
}

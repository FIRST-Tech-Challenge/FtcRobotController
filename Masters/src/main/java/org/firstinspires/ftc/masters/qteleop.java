package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.masters.components.ControllerMap;
import org.firstinspires.ftc.masters.components.DriveTrain;
//import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;

@Config // Enables FTC Dashboard
@TeleOp(name = "qteleop", group = "ri30h")
public class qteleop extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap);
//        Intake intake = new Intake(hardwareMap);
        Outake outake = new Outake(hardwareMap, telemetry);

        outake.close();

        waitForStart();

        while (opModeIsActive()) {

            driveTrain.driveNoMultiplier(gamepad1, DriveTrain.RestrictTo.XYT);

            if (gamepad1.a) {outake.forward();}
            if (gamepad1.b) {outake.backward();}

            if (gamepad1.x) {outake.open();}
            if (gamepad1.y) {outake.close();}

            if (gamepad1.dpad_up) {
                outake.extend();
            } else if (gamepad1.dpad_down) {
                outake.retract();
            } else {
                outake.stop();
            }

            if (gamepad1.dpad_left) {
                outake.rotateUp();
            } else if (gamepad1.dpad_right) {
                outake.rotateDown();
            } else {
                outake.stop();
            }

        }
    }
}
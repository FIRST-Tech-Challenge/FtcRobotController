package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;

@TeleOp
public class CameraTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        Camera camera = new Camera(this, false, true);
        Servo s = hardwareMap.get(Servo.class, "s");
        waitForStart();
        while (opModeIsActive()) {
            s.setPosition(camera.getPostion());
        }
    }
}

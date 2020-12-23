package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp(name="Camera: Test", group="Test")
public class CameraTest extends OpMode{

    RingCamera camera = new RingCamera();

    /***********************************
     *
     * This program tests the ring detection with a camera
     *
     ***********************************/

    @Override
    public void init() {
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 25000;

        camera.init(hardwareMap);

    }

    @Override
    public void loop() {

        telemetry.addData("Number of Rings", camera.ringCount());
        telemetry.update();

    }
}

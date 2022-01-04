package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SkystoneCamTest", group="Test")
@Disabled
public class SkystoneCamTest extends OpMode {
    SkystoneCam cam    = new SkystoneCam();

    @Override
    public void init() {
        cam.init(hardwareMap);

        msStuckDetectInit = 11000;
    }

    @Override
    public void loop() {
        telemetry.addData("Position","{X,Y,Z} = %.1f, %.1f, %.1f",
                cam.getXPosition(),cam.getYPosition(),cam.getZPosition());
        telemetry.addData("Rotation","{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                cam.getRoll(),cam.getPitch(),cam.getHeading());
        telemetry.update();
    }
}

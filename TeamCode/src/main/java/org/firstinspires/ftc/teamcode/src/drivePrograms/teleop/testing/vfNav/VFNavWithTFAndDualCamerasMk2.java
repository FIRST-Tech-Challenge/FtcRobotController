package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing.vfNav;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoVFTrackingTemplate;

import java.util.Locale;

@Disabled
@TeleOp(name = "-VFNavWithTFAndDualCamerasMK2")
public class VFNavWithTFAndDualCamerasMk2 extends AutoVFTrackingTemplate {

    @Override
    public void opModeMain() throws InterruptedException {

        this.initAll();
        telemetry.addData("Initialization", "Finished");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested()) {

            Double[] pos = this.getLocation();
            telemetry.addData("TFOD Detection", this.findPositionOfMarker());
            if (pos != null) {
                telemetry.addData("Seen", this.getVisibleTargetName());
                telemetry.addData("Pos", String.format(Locale.ENGLISH, "{X, Y, Z} = %.0f, %.0f, %.0f", pos[0], pos[1], pos[2]));
            } else {
                telemetry.addData("Seen", "Nothing");
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
            Thread.sleep(60);

        }
        deactivateAll();
    }


}

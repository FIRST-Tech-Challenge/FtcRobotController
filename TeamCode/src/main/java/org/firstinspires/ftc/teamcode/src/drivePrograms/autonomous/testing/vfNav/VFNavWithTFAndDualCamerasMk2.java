package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing.vfNav;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoVFTrackingTemplate;

import java.util.Locale;

@TeleOp(name = "-VFNavWithTFAndDualCamerasMK2")
public class VFNavWithTFAndDualCamerasMk2 extends AutoVFTrackingTemplate {

    @Override
    public void opModeMain() throws InterruptedException {

        initVuforia();
        initTfod();
        changeToLeftCamera();
        telemetry.addData("Initialization", "Finished");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("Seen", this.getVisibleTargetName());
            Double[] pos = this.getLocation();
            if (pos != null) {
                telemetry.addData("Pos", String.format(Locale.ENGLISH, "{X, Y, Z} = %.0f, %.0f, %.0f", pos[0], pos[1], pos[2]));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();
            Thread.sleep(60);

        }
        deactivateAll();
    }


}

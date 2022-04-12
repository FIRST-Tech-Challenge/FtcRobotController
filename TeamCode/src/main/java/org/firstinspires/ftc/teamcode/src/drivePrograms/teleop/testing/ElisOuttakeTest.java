package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.outtake.OuttakeMk3;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "Elis Outtake Test")
public class ElisOuttakeTest extends TeleOpTemplate {

    boolean aDepressed = true;
    boolean bDepressed = true;

    @Override
    public void opModeMain() throws InterruptedException {
        OuttakeMk3 tmp = new OuttakeMk3(hardwareMap, bucketServoName, bucketColorSensorName, this::opModeIsActive, this::isStopRequested);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.a && aDepressed) {
                aDepressed = false;
                tmp._rotateForward(1000);
            } else if (gamepad2.b && bDepressed) {
                bDepressed = false;
                tmp._rotateBackward(1000);
            } else {
                tmp.update();
            }

            if (!gamepad2.b) {
                bDepressed = true;
            }

            if (!gamepad2.a) {
                aDepressed = true;
            }
            Thread.yield();
        }
    }
}

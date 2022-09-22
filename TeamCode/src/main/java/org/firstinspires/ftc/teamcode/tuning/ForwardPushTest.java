package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.List;

public final class ForwardPushTest extends LinearOpMode {
    private static double avgPos(List<? extends Encoder> es) {
        double avgPos = 0;
        for (Encoder e : es) {
            avgPos += e.getPositionAndVelocity().position;
        }
        return avgPos / es.size();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(hardwareMap);

        for (DcMotorEx m : view.motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        waitForStart();

        double initAvgPos = avgPos(view.forwardEncs);
        while (opModeIsActive()) {
            telemetry.addData("ticks traveled", avgPos(view.forwardEncs) - initAvgPos);
            telemetry.update();
        }
    }
}

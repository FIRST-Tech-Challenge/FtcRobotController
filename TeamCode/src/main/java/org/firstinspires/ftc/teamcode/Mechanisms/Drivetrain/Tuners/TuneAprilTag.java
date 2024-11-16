package org.firstinspires.ftc.teamcode.Mechanisms.Drivetrain.Tuners;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;

@Autonomous(name = "Test April Tag", group = "Autonomous")
public class TuneAprilTag extends LinearOpMode {

    private Limelight3A limelight;
    @Override

    public void runOpMode()
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.update();
                }
            }
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

public class TurretOpMode extends OpMode {
    // Declar variables
    private Limelight3A limelight;
    // 2 instances of the turret
    private potatoTurret turret = new potatoTurret();
    private potatoTurret turret2 = new potatoTurret();

    // testing
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;


    public void init() {
        // init limelight and connect turret init to hardwareMap
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        turret.init(hardwareMap);
        turret2.init(hardwareMap);
    }

    public void loop() {
        // classic
        LLResult result = limelight.getLatestResult();

        LLResultTypes.FiducialResult targetTag = null;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Search through detected AprilTags for ID 20
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 20) {
                    targetTag = fiducial;
                    break;
                }
            }
        }

        turret.update(targetTag);
        turret2.update(targetTag);

        // tuning
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setKP(turret.getKp() - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadRightWasPressed()) {
            turret.setKP(turret.getKp() + stepSizes[stepIndex]);
        }

        if (gamepad1.dpadDownWasPressed()) {
            turret.setKD(turret.getKd() - stepSizes[stepIndex]);
        }

        if (gamepad1.dpadUpWasPressed()) {
            turret.setKD(turret.getKd() + stepSizes[stepIndex]);
        }

        // apriltag detection
        if (targetTag != null) {
            telemetry.addData("curId", targetTag.getFiducialId());
            telemetry.addData("Target X (deg)", targetTag.getTargetXDegrees());
        } else {
            telemetry.addLine("No AprilTag Detected");
        }

        telemetry.addLine("----------TUNING----------");
        telemetry.addData("Tuning P","%.5f (D-Pad/Left or Right)", turret.getKp());
        telemetry.addData("Tuning D", "%.5f (D-Pad/Up or Down)", turret.getKd());
        telemetry.addData("Step Size", "%.5f B-Button", stepSizes[stepIndex]);
    }

    public void start() {
        turret.resetTimer();
        turret2.resetTimer();
    }
}


package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.List;

public class Vision implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private Limelight3A limelight;
    private LLResult result;

    private Boolean tipped = false;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(
                Limelight3A.class,
                RobotConstants.LIMELIGHT
        );

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void update() {
        result = limelight.getLatestResult();
    }

    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    public double getTx() {
        return hasTarget() ? result.getTx() : 0;
    }

    public double getTy() {
        return hasTarget() ? result.getTy() : 0;
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    @Override
    public boolean TipDetect(double startCluster, double endCluster) {
        // 42 and 45 returns true for Blue Audience
        // 38 and 41 returns true for Blue Far
        // add red audience and red far blah blah blah
        tipped = false;

        if (hasTarget()) {

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            List<LLResultTypes.FiducialResult> fiducialResults =
                    result.getFiducialResults();

            for (LLResultTypes.FiducialResult fr : fiducialResults) {

                int id = fr.getFiducialId();

                telemetry.addData(
                        "Fiducial",
                        "ID: %d, Family: %s, X: %.2f, Y: %.2f",
                        id,
                        fr.getFamily(),
                        fr.getTargetXDegrees(),
                        fr.getTargetYDegrees()
                );

                if (id >= startCluster && id <= endCluster) {
                    tipped = true;
                    break;
                }
            }
        }

        return tipped;
    }


    @Override
    public void stop() {
        //hi
    }
}

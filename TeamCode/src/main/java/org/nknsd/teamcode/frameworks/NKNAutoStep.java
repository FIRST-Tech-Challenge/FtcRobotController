package org.nknsd.teamcode.frameworks;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.teamcode.helperClasses.AutoSkeleton;

public interface NKNAutoStep {
    void link(AutoSkeleton autoSkeleton);
    void run(Telemetry telemetry, ElapsedTime runtime);
    boolean isDone(ElapsedTime runtime);
    String getName();
    void begin(ElapsedTime runtime, Telemetry telemetry);
}

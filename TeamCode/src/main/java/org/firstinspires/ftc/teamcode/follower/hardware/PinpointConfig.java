package org.firstinspires.ftc.teamcode.follower.hardware;

import org.firstinspires.ftc.teamcode.follower.config.ConfigVar;
import org.firstinspires.ftc.teamcode.follower.config.Configuration;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;

public class PinpointConfig {
    public final ConfigVar<String> name = ConfigVar.required();
    public final ConfigVar<GoBildaPinpointDriver.EncoderDirection> xPodDirection = ConfigVar.required();
    public final ConfigVar<GoBildaPinpointDriver.EncoderDirection> yPodDirection = ConfigVar.required();
    public final ConfigVar<Double> xPodOffset = ConfigVar.required();
    public final ConfigVar<Double> yPodOffset = ConfigVar.required();
    public final ConfigVar<DistanceUnit> offsetUnits = ConfigVar.of(DistanceUnit.INCH);

    /** Global distance unit for all poses and positions */
    public final ConfigVar<DistanceUnit> globalDistanceUnit = ConfigVar.of(DistanceUnit.INCH);
    public final ConfigVar<DistanceUnit> encoderResolutionUnit = ConfigVar.of(DistanceUnit.INCH);

    public final ConfigVar<GoBildaPinpointDriver.GoBildaOdometryPods> podType = ConfigVar.of(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    public final ConfigVar<OptionalDouble> ticksPerUnit = ConfigVar.of(OptionalDouble.empty());
    public final ConfigVar<PinpointLocalizer.ResetMode> resetMode = ConfigVar.of(PinpointLocalizer.ResetMode.RECALIBRATE_IMU);

    public PinpointConfig(Configuration<PinpointConfig> config) {
        config.configure(this);
    }
}
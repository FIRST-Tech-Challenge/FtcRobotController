package org.firstinspires.ftc.teamcode.robots.csbot.vision.provider.dummy;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.csbot.vision.Position;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.VisionProvider;

import java.util.Map;

public abstract class AbstractDummyProvider extends VisionProvider {
    @Override
    public void initializeVision(HardwareMap hardwareMap) {

    }

    @Override
    public void shutdownVision() {

    }

    @Override
    public abstract Position getPosition();

    @Override
    public void reset() {

    }

    @Override
    public boolean canSendDashboardImage() {
        return false;
    }

    @Override
    public Bitmap getDashboardImage() {
        return null;
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return super.getTelemetry(debug);
    }

    @Override
    public abstract String getTelemetryName();

    @Override
    public void updateVision() {

    }
}

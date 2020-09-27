package org.firstinspires.ftc.teamcode.playmaker;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public abstract class RobotHardware extends OpMode {

    public static final String vuforiaKey = "ATYXw6b/////AAAAGbsBoJKrv00tn+OIBZOySi93f157TAsX4H3f444TrvUXKWFiNjsiBUjhwGrShLYay8wFSrlf+nRtoS+xnZJ3IApbJe2W0NSLKz21/B3/IpstUZGH29ZD/ogg3ZixfNyUGyb+F5gy5LzvGTdRhGLwy0d4z2i6QauSDPYHU4bBnhmehHBFMPkA6aP94fqOfa4qJGKBCCrn1EcH+c5TXD2EP21vciteCYktsfBedAnveiDGR7yLbTPr5kdfLvem0iyH8ESxhOsr90wGnIGWOQJa83eilaVbmLHtWkQx/hT/CnNTglJXb6TGRuDEwv/Zs+zdswp9dvCHZL5Qq1pT4y+LNUZZfhtmLlYXNifiEn7HnM5f";
    public VuforiaLocalizer.Parameters vuforiaParameters;
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    public OmniDrive omniDrive;
    public static double COUNTS_PER_INCH;
    public static double COUNTS_PER_LAT_INCH;
    public static double COUNTS_PER_DEGREE;

    /**
     * All hardware should initialize sensors and stuff here
     */
    public abstract void initializeHardware();

    public void initializeAutonomous() {}

    public void initializeTeleOp() {}

    public <T extends HardwareDevice> T initializeDevice(Class<? extends T> deviceClass, String name) {
        try {
            return this.hardwareMap.get(deviceClass, name);
        } catch (Exception e) {
            this.telemetry.addLine(String.format("Err: Device \"%s\" cannot be found.", name));
            return null;
        }
    }

    @Override
    public void init() {
        this.initializeHardware();
    }

    public void hardware_loop() {}

}

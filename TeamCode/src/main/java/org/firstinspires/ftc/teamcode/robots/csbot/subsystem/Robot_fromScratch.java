package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robots.csbot.Field;
import org.firstinspires.ftc.teamcode.robots.csbot.util.PositionCache;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Config(value = "AA_CSRobot_fromScratch")
public class Robot_fromScratch extends Robot implements Subsystem{

    //components and subsystems
    public CSDriveTrain driveTrain;
//    public Intake;
    public Subsystem[] subsystems;


    private long[] subsystemUpdateTimes;
    private final List<LynxModule> hubs;
    private VoltageSensor batteryVoltageSensor;

    public Robot_fromScratch(HardwareMap hardwareMap, boolean simulated) {
        super(hardwareMap, false);
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // initializing subsystems
        driveTrain = new CSDriveTrain(hardwareMap, this, simulated);
        turret = new Turret(hardwareMap, this, simulated);
        underarm = new UnderArm(hardwareMap, this, simulated);
        crane = new Crane(hardwareMap, this, simulated);

        positionCache = new PositionCache( 5);

        subsystems = new Subsystem[] {driveTrain, turret, crane, underarm}; //{driveTrain, turret, crane};
        subsystemUpdateTimes = new long[subsystems.length];

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

//        articulation = Robot.Articulation.MANUAL;

//        articulationMap = new HashMap<>();

//        craneBitmap = Bitmap.createBitmap(CB_WIDTH, CB_HEIGHT, Bitmap.Config.RGB_565);
//        craneMat = new Mat(CB_HEIGHT, CB_WIDTH, CvType.CV_8UC3);
        field = new Field(true);
    }
    //end constructor

    @Override
    public void update(Canvas fieldOverlay) {

    }

    @Override
    public void stop() {

    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return null;
    }

    @Override
    public String getTelemetryName() {
        return null;
    }
}

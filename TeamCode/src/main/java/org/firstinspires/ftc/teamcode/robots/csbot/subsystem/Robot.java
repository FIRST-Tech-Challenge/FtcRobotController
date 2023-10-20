package org.firstinspires.ftc.teamcode.robots.csbot.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.csbot.vision.Target;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config(value = "AA_CSRobot")
public class Robot implements Subsystem{

    //components and subsystems
    public Subsystem[] subsystems;
    public CSDriveTrain driveTrain;
    //TODO - implement subsystems that build team comes up with
    public Intake intake;
    //TODO - create a field
//    public Field field;


    private long[] subsystemUpdateTimes;
    private final List<LynxModule> hubs;
    private VoltageSensor batteryVoltageSensor;
    private Articulation articulation;
    public List<Target> targets = new ArrayList<Target>();

    public enum Articulation {
        //beater bar, drivetrain, drone launcher, outtake
        MANUAL,
        AUTON,
        CALIBRATE,
        SCORE_PIXEL,
        INTAKE_PIXEL,
        FOLD,
        UNFOLD,
        HANG,
        LAUNCH_DRONE,

    }

    public void start() {
        //TODO - articulate starting position
        articulation = Articulation.MANUAL;
    }
    //end start


    public Robot(HardwareMap hardwareMap, boolean simulated) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // initializing subsystems
        driveTrain = new CSDriveTrain(hardwareMap, this, simulated);
        //TODO - THIS IS FOR MANUAL ONLY
        driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake = new Intake(hardwareMap, this);

        subsystems = new Subsystem[] {driveTrain, intake}; //{driveTrain, turret, crane};
        subsystemUpdateTimes = new long[subsystems.length];

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        articulation = Robot.Articulation.MANUAL;

//        field = new Field(true);
    }
    //end constructor

    public double deltaTime = 0;
    long lastTime = 0;
    @Override
    public void update(Canvas fieldOverlay) {
        deltaTime = (System.nanoTime()-lastTime)/1e9;
        lastTime = System.nanoTime();

        clearBulkCaches(); //ALWAYS FIRST LINE IN UPDATE

        articulate(articulation);

        //update subsystems
        for(int i = 0; i < subsystems.length; i++) {
            Subsystem subsystem = subsystems[i];
            long updateStartTime = System.nanoTime();
            subsystem.update(fieldOverlay);
            subsystemUpdateTimes[i] = System.nanoTime() - updateStartTime;
        }
    }
    //end update

    public Articulation articulate(Articulation target) {
        articulation = target;
        switch (this.articulation) {
            case MANUAL:
                break;
            case CALIBRATE:
                //TODO - WRITE A CALIBRATION ROUTINE
                break;

        }
        return articulation;
    }

    @Override
    public void stop() {
        for (Subsystem component : subsystems) {
            component.stop();
        }
    }
    //end stop

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
        telemetryMap.put("Articulation", articulation);

        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            telemetryMap.put(name + " Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (subsystemUpdateTimes[i] * 1e-6), (int) (1 / (subsystemUpdateTimes[i] * 1e-9))));
        }

        if(debug) {
            telemetryMap.put("DriveTrain Pose X", driveTrain.poseEstimate.getX());
            telemetryMap.put("DriveTrain Pose Y", driveTrain.poseEstimate.getY());
            telemetryMap.put("DriveTrain Pose Heading", driveTrain.poseEstimate.getHeading());
        }

        telemetryMap.put("Delta Time", deltaTime);



        return telemetryMap;
    }
    //end getTelemetry

    public void clearBulkCaches(){
        for (LynxModule module : hubs)
            module.clearBulkCache();
    }

    public double getVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    @Override
    public String getTelemetryName() {
        return "ROBOT";
    }
}

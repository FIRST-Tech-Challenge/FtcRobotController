package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.UtilMethods;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@Config
public class Robot implements Subsystem {
    // subsystems
    public DriveTrain driveTrain;
    public Turret turret;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    // sensors
    private List<LynxModule> hubs;

    // state
    private Articulation articulation;

    // constants
    private static final String TELEMETRY_NAME = "Robot";

    private Map<Articulation, StateMachine> articulationMap;

    public Robot(HardwareMap hardwareMap, boolean simulated) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // initializing subsystems
        driveTrain = new DriveTrain(hardwareMap, simulated);
        turret = new Turret(hardwareMap, simulated);
        crane = new Crane(hardwareMap, turret, simulated);
        gripper = new Gripper(hardwareMap, simulated);
        subsystems = new Subsystem[] {driveTrain, crane, gripper};

        articulation = Articulation.MANUAL;

        articulationMap = new HashMap<>();
        articulationMap.put(Articulation.INIT, init);
        articulationMap.put(Articulation.START, start);
        articulationMap.put(Articulation.DIAGNOSTIC, diagnostic);
        articulationMap.put(Articulation.TRANSFER, transfer);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new HashMap<>();
        telemetryMap.put("Articulation", articulation);

        return telemetryMap;
    }


    @Override
    public String getTelemetryName() {
        return TELEMETRY_NAME;
    }

    @Override
    public void update(Canvas fieldOverlay) {
        for (LynxModule module : hubs) {
            module.clearBulkCache();
        }
        for(Subsystem subsystem: subsystems)
            subsystem.update(fieldOverlay);

        articulate(articulation);
    }

    @Override
    public void stop() {
        for(Subsystem subsystem: subsystems)
            subsystem.stop();
    }

    //----------------------------------------------------------------------------------------------
    // Articulations
    //----------------------------------------------------------------------------------------------

    public enum Articulation {
        MANUAL,

        // misc. articulations
        INIT,
        START,
        DIAGNOSTIC,

        // tele-op articulations
        TRANSFER,
    }

    // Misc. Articulations
    private Stage diagnosticStage = new Stage();
    private StateMachine diagnostic = UtilMethods.getStateMachine(diagnosticStage)
            .build();

    // Tele-Op articulations
    private Stage transferStage = new Stage();
    private StateMachine transfer = UtilMethods.getStateMachine(transferStage)
            .addState(() -> {
                        driveTrain.setChassisLength(Constants.MIN_CHASSIS_LENGTH);
                        return driveTrain.chassisLengthOnTarget();
            })
            .addTimedState(1f, () -> crane.articulate(Crane.Articulation.TRANSFER), () -> gripper.articulate(Gripper.Articulation.TRANSFER))
            .build();

    private Stage initStage = new Stage();
    private StateMachine init = UtilMethods.getStateMachine(initStage)
            .addSingleState(() -> gripper.set())
            .addState(() -> crane.articulate(Crane.Articulation.INIT))
            .build();

    private Stage startStage = new Stage();
    private StateMachine start = UtilMethods.getStateMachine(startStage)
            .addTimedState(2, () -> driveTrain.setDuckSpinnerPower(0.5), () -> driveTrain.setDuckSpinnerPower(0))
            .build();

    public boolean articulate(Articulation articulation) {
        this.articulation = articulation;

        if(articulation.equals(Articulation.MANUAL))
            return true;
        else if(articulationMap.get(articulation).execute()) {
            this.articulation = Articulation.MANUAL;
            return true;
        }
        return false;
    }
}

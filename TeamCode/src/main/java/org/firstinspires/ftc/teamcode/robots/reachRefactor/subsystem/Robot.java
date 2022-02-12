package org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.*;

/**
 * @author Mahesh Natamai
 */

@Config
public class Robot implements Subsystem {
    private static final String TELEMETRY_NAME = "Robot";
    public static double HIGH_TIER_TRIGGER_PITCH_VELOCITY = 1000;

    public DriveTrain driveTrain;
    public Turret turret;
    public Crane crane;
    public Gripper gripper;
    public Subsystem[] subsystems;

    private List<LynxModule> hubs;

    private Articulation articulation;
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
        articulationMap.put(Articulation.START_END_GAME, startEnd);
        articulationMap.put(Articulation.TRANSFER, transfer);
        articulationMap.put(Articulation.TRANSFER_AND_HIGH_TIER, transferAndHighTier);
    }

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();
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

        if(driveTrain.getPitchVelocity() > HIGH_TIER_TRIGGER_PITCH_VELOCITY && crane.getArticulation() == Crane.Articulation.MANUAL)
            crane.articulate(Crane.Articulation.HIGH_TIER);

        articulate(articulation);
    }

    //----------------------------------------------------------------------------------------------
    // Articulations
    //----------------------------------------------------------------------------------------------

    public enum Articulation {
        MANUAL,

        // misc. articulations
        INIT,
        START, //use to prep for start - mostly stows the Crane
        START_END_GAME, //use on a timer to automatically deploy carousel spinner 10 seconds before end game

        // tele-op articulations
        TRANSFER,
        TRANSFER_AND_HIGH_TIER
    }

    // Tele-Op articulations
    private Stage transferAndHighTierStage = new Stage();
    private StateMachine transferAndHighTier = getStateMachine(transferAndHighTierStage)
            .addState(() -> {
                driveTrain.setChassisLength(Constants.MIN_CHASSIS_LENGTH);
                return driveTrain.chassisLengthOnTarget();
            })
            .addState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addTimedState(1f, () -> gripper.articulate(Gripper.Articulation.TRANSFER), () -> {})
            .addState(() -> crane.articulate(Crane.Articulation.HIGH_TIER))
            .build();

    private Stage transferStage = new Stage();
    private StateMachine transfer = getStateMachine(transferStage)
            .addState(() -> {
                        driveTrain.setChassisLength(Constants.MIN_CHASSIS_LENGTH);
                        return driveTrain.chassisLengthOnTarget();
            })
            .addState(() -> crane.articulate(Crane.Articulation.TRANSFER))
            .addTimedState(1f, () -> gripper.articulate(Gripper.Articulation.TRANSFER), () -> {})
            .addState(() -> crane.articulate(Crane.Articulation.HOME))
            .build();

    private Stage initStage = new Stage();
    private StateMachine init = getStateMachine(initStage)
            .addSingleState(() -> gripper.lift())
            .addState(() -> crane.articulate(Crane.Articulation.HOME)) //for visual confirmation that the crane is vertial and aligned
            .build();

    private Stage startEndGame = new Stage();
    private StateMachine startEnd = getStateMachine(startEndGame)
            .addTimedState(2, () -> driveTrain.setDuckSpinnerPower(0.5), () -> driveTrain.setDuckSpinnerPower(0))
            .build();

    private Stage startStage = new Stage(); //prep for Start
    private StateMachine start = getStateMachine(startStage)
            .addSingleState(() -> gripper.lift())
            .addState(() -> crane.articulate(Crane.Articulation.INIT))
            .build();

    public boolean articulate(Articulation articulation) {
        if(articulation.equals(Articulation.MANUAL))
            return true;
        this.articulation = articulation;
        if(articulationMap.get(articulation).execute()) {
            this.articulation = Articulation.MANUAL;
            return true;
        }
        return false;
    }
}

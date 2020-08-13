package org.firstinspires.ftc.teamcode.rework.Robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.Robot.Modules.Module;
import org.firstinspires.ftc.teamcode.rework.Robot.Modules.ModuleExecutor;
import org.firstinspires.ftc.teamcode.rework.Robot.Modules.ReworkDrivetrain;

import java.util.ArrayList;

public class ReworkRobot {
    public ReworkDrivetrain drivetrain;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

    ModuleExecutor moduleExecutor;

    // All modules in the robot (remember to update initModules() and updateModules() when adding)
    private Module[] modules;

    // REV Hubs
    private LynxModule revHub1;
    private LynxModule revHub2;

    // Data
    private LynxModule.BulkData revHub1Data;
    private LynxModule.BulkData revHub2Data;

    public ReworkRobot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        initHubs();
        initModules();
    }

    /**
     * Gets a DcMotor from the hardwareMap given the name of the motor, returning null if the
     * motor does not exist.
     *
     * @param name of the DcMotor to return.
     * @return DcMotor from hardwareMap, or null if the motor does not exist.
     */
    public DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Motor with name " + name + " could not be found. Exception: " + exception);
        }
    }

    /**
     * Updates all the modules in robot.
     */
    public void updateModules() {
        for(Module module : modules) {
            module.update();
        }
    }

    /**
     * Initializes all modules in robot. Starts another thread on which execution of the modules
     * will happen in the form of updates.
     *
     * Note that starting the thread also executes run(). However, note that run() will not update
     * the modules if the opMode is not active yet.
     */
    public void initModules() {

        // Hook modules here
        this.drivetrain = new ReworkDrivetrain();
        this.modules = new Module[] {
            this.drivetrain
        };

        for(Module module : modules)
            module.init();

        // Start the thread for executing modules.
        moduleExecutor = new ModuleExecutor(this);
        moduleExecutor.start();
    }

    /**
     * Runs the loop that updates modules in the separate thread. This loop continues running while
     * isOpModeActive() returns true.
     *
     * @see #isOpModeActive()
     */
    public void startModules() {
        moduleExecutor.run();
    }

    private void initHubs() {
        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Control Hub 1"); // TODO: Determine actual name of new control hub
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        } catch (Exception e) {
            throw new Error("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    /**
     * Gets all sensor data from the hubs.
     */
    public synchronized void getBulkData() {
        revHub1Data = revHub1.getBulkData();
        revHub2Data = revHub2.getBulkData();
    }

    public synchronized LynxModule.BulkData getRevHub1Data() {
        return revHub1Data;
    }

    public synchronized LynxModule.BulkData getRevHub2Data() {
        return revHub2Data;
    }

    /**
     * Returns if the robot's op mode is active.
     *
     * @return boolean representing whether or not the op mode is active.
     */
    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }
}

package org.firstinspires.ftc.teamcode.rework.Robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.Robot.Modules.Module;
import org.firstinspires.ftc.teamcode.rework.Robot.Modules.ModuleExecutor;
import org.firstinspires.ftc.teamcode.rework.Robot.Modules.Odometry.Odometry;
import org.firstinspires.ftc.teamcode.rework.Robot.Modules.DrivetrainModule;

public class Robot {
    // All modules in the robot (remember to update initModules() and updateModules() when adding)
    public DrivetrainModule drivetrain;
    public Odometry odometry;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

    // New thread that updates modules
    ModuleExecutor moduleExecutor;

    // Array that all modules will be loaded into for easier access
    private Module[] modules;

    // REV Hubs
    private LynxModule revHub1;
    private LynxModule revHub2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        initHubs();
        initModules();
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
        // Add individual modules into the array here
        this.drivetrain = new DrivetrainModule(this);
        this.odometry = new Odometry(this);

        this.modules = new Module[] {
            this.drivetrain, this.odometry
        };

        // Initialize modules
        for(Module module : modules) {
            module.init();
        }

        // Start the thread for executing modules.
        moduleExecutor = new ModuleExecutor(this, telemetry);
        moduleExecutor.start();
    }

    /**
     * Runs the loop that updates modules in the separate thread. This loop continues running while
     * isOpModeActive() returns true.
     *
     * @see #isOpModeActive()
     */
    public void startModules() {
        moduleExecutor.start();
    }

    private void initHubs() {
        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Expansion Hub 3"); // TODO: Determine actual name of new control hub
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (Exception e) {
            throw new Error("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    /**
     * Gets all sensor data from the hubs.
     */
    public void getBulkData() {
        // revHub1.getBulkData(); // No need for data from Hub 1 as of right now
        revHub2.getBulkData();
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
     * Returns if the robot's op mode is active.
     *
     * @return boolean representing whether or not the op mode is active.
     */
    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }

    public Telemetry getTelemetry(){
        return telemetry;
    }
}

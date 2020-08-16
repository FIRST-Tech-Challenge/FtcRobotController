package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.Module;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.ModuleExecutor;
import org.firstinspires.ftc.teamcode.rework.Modules.OdometryModule;
import org.firstinspires.ftc.teamcode.rework.Modules.DrivetrainModule;

public class Robot {
    // All modules in the robot (remember to update initModules() and updateModules() when adding)

    public DrivetrainModule drivetrainModule;
    public OdometryModule odometryModule;
    public long currentTimeMilli;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

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

    public void update() {
        refreshData2();

        currentTimeMilli = SystemClock.elapsedRealtime();

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
        this.drivetrainModule = new DrivetrainModule(this);
        this.odometryModule = new OdometryModule(this);

        this.modules = new Module[] {
            this.drivetrainModule, this.odometryModule
        };

        // Initialize modules
        for(Module module : modules) {
            module.init();
        }

        // Start the thread for executing modules.
        moduleExecutor = new ModuleExecutor(this, telemetry);
    }

    /**
     * Starts running the loop that updates modules
     */
    public void startModules() {
        moduleExecutor.start();
    }

    private void initHubs() {
        try {
            revHub1 = hardwareMap.get(LynxModule.class, "Expansion Hub 3");
            revHub1.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            revHub2 = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            revHub2.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        } catch (Exception e) {
            throw new Error("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    public void refreshData1() {
        revHub1.getBulkData();
    }

    public void refreshData2() {
        revHub2.getBulkData();
    }

    public DcMotor getDcMotor(String name) {
        try {
            return hardwareMap.dcMotor.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Motor with name " + name + " could not be found. Exception: " + exception);
        }
    }

    public Servo getServo(String name) {
        try {
            return hardwareMap.servo.get(name);
        } catch (IllegalArgumentException exception) {
            throw new Error("Servo with name " + name + " could not be found. Exception: " + exception);
        }
    }

    public boolean isOpModeActive() {
        return linearOpMode.opModeIsActive();
    }
}

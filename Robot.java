package org.firstinspires.ftc.teamcode.rework;

import android.os.SystemClock;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.ModuleTools.TelemetryProvider;
import org.firstinspires.ftc.teamcode.rework.Modules.DrivetrainModule;
import org.firstinspires.ftc.teamcode.rework.Modules.Module;
import org.firstinspires.ftc.teamcode.rework.Modules.OdometryModule;
import org.firstinspires.ftc.teamcode.rework.Modules.VelocityModule;
import org.firstinspires.ftc.teamcode.rework.RobotTools.FileDump;
import org.firstinspires.ftc.teamcode.rework.RobotTools.ModuleExecutor;
import org.firstinspires.ftc.teamcode.rework.RobotTools.TelemetryDump;

public class Robot {
    // All modules in the robot (remember to update initModules() and updateModules() when adding)
    public DrivetrainModule drivetrainModule;
    public OdometryModule odometryModule;
    public VelocityModule velocityModule;

    public long currentTimeMilli;

    public HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

    public TelemetryDump telemetryDump;
    public FileDump fileDump;

    // New thread that updates modules
    ModuleExecutor moduleExecutor;

    // Array that all modules will be loaded into for easier access
    private Module[] modules;

    // REV Hubs
    private LynxModule revHub1;
    private LynxModule revHub2;

    public final boolean WILL_FILE_DUMP = false;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        this.telemetryDump = new TelemetryDump(telemetry);
        fileDump = new FileDump();

        initHubs();
        initModules();
    }

    public void update() {
        refreshData2();

        currentTimeMilli = SystemClock.elapsedRealtime();

        for(Module module : modules) {
            if(module.isOn()) {
                try {
                    module.update();
                }catch (Exception e) {
                    Log.d("Module", "Module couldn't update: " + module.getName());
                }
                if (WILL_FILE_DUMP) {
                    module.fileDump();
                }
            }
        }
        telemetryDump.update();
    }

    public void initModules() {
        // Add individual modules into the array here
        this.drivetrainModule = new DrivetrainModule(this,true);
        this.odometryModule = new OdometryModule(this,true);
        this.velocityModule = new VelocityModule(this,true);

        this.modules = new Module[] {
                this.drivetrainModule, this.odometryModule, this.velocityModule
        };

        // Initialize modules
        for(Module module : modules) {
            module.init();
        }

        // Start the thread for executing modules.
        moduleExecutor = new ModuleExecutor(this);
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

    public boolean isStopRequested(){
        return linearOpMode.isStopRequested();
    }
}

package org.firstinspires.ftc.teamcode.rework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.Modules.Module;
import org.firstinspires.ftc.teamcode.rework.Modules.ModuleExecutor;
import org.firstinspires.ftc.teamcode.rework.Modules.ReworkDrivetrain;

public class ReworkRobot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

    // All modules in the robot (remember to update init and get)
    protected ReworkDrivetrain drivetrain;

    public ReworkRobot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;

        initModules();
    }

    /**
     * Starts new thread that will execute modules. Run after starting the program.
     */
    public void startExecutingModules() {
        // Start the thread for executing modules.
        ModuleExecutor moduleExecutor = new ModuleExecutor(this);
        moduleExecutor.start();
    }

    /**
     * Returns all modules in robot, in the form of an array.
     */
    public Module[] getModules() {
        return new Module[] {drivetrain};
    }

    /**
     * Initializes all modules in robot.
     */
    public void initModules() {
        drivetrain = new ReworkDrivetrain(hardwareMap);
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

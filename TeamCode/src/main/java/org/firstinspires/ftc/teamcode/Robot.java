package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Owns every subsystem. OpModes create one Robot and call init(), update(), stop().
 */
public class Robot {

    public final Drivetrain drive;
    public final Odometry odo;
    public final Launcher launcher;
    public final Intake intake;
    public final Vision vision;

    private final Telemetry telemetry;
    private final Subsystem[] subsystems;
    private final boolean[] initOk; // false if a subsystem failed to init (e.g. device not configured)

    public Robot(HardwareMap hardwareMap, Telemetry opModeTelemetry) {
        // One telemetry that shows on the Driver Station AND FTC Dashboard.
        RobotLog.i("Robot: Initializing telemetry");
        telemetry = new MultipleTelemetry(opModeTelemetry, FtcDashboard.getInstance().getTelemetry());
        RobotLog.i("Robot: Telemetry initialized");
        RobotLog.i("Robot: Initializing subsystems");
        drive    = new Drivetrain(hardwareMap, telemetry);
        odo      = new Odometry(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap, telemetry);
        intake   = new Intake(hardwareMap, telemetry);
        vision   = new Vision(hardwareMap, telemetry);
        RobotLog.i("Robot: Subsystems initialized");
        RobotLog.i("Robot: Creating subsystems array and initOk array");
        subsystems = new Subsystem[] {drive, odo, launcher, intake, vision};
        initOk = new boolean[subsystems.length];
    }

    /**
     * A subsystem whose hardware is missing is skipped (and reported) instead of crashing the OpMode.
     * Don't call methods on a skipped subsystem, or you'll get a NullPointerException.
     */
    public void init() {
        RobotLog.ii("Robot", "init");
        for (int i = 0; i < subsystems.length; i++) {
            String name = subsystems[i].getClass().getSimpleName();
            try {
                subsystems[i].init();
                initOk[i] = true;
            } catch (Exception e) {
                initOk[i] = false;
                RobotLog.ee("Robot", e, name + " failed to init - skipped");
                telemetry.addData("WARNING", name + " skipped: " + e.getMessage());
            }
        }
        telemetry.update();
    }

    /** Call once per loop. */
    public void update() {
        for (int i = 0; i < subsystems.length; i++) {
            if (initOk[i]) subsystems[i].update();
        }
        telemetry.update(); // only Robot calls this, once per loop
    }

    public void stop() {
        RobotLog.ii("Robot", "stop");
        for (int i = 0; i < subsystems.length; i++) {
            if (initOk[i]) subsystems[i].stop();
        }
    }
}

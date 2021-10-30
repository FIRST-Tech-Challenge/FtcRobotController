package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Util.QuickTelemetry;

/**
 * Superclass to all minor subsystems, it does some bootstrapping for them (Vision, Control, and Drive)
 *
 */
public class Subsystem {
    protected LinearOpMode opMode; // protected because of inheritance
    protected QuickTelemetry telemetry;
    protected ElapsedTime timer;
    protected HardwareMap hardwareMap;

    public Subsystem(Robot robot, String file){
        this.opMode = robot.getOpMode();
        this.telemetry = robot.getQuickTelemetry(file);
        this.hardwareMap = opMode.hardwareMap;
        this.timer = robot.getTimer();
    }
}

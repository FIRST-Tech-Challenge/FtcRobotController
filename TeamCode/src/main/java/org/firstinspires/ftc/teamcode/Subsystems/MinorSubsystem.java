package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Superclass to all minor subsystems, it does some bootstrapping for them (Vision, Control, and Drive)
 *
 */
public class MinorSubsystem extends Subsystem {
    protected LinearOpMode opMode; // protected because of inheritance
    protected Telemetry telemetry;
    protected ElapsedTime timer;
    protected HardwareMap hardwareMap;

    public MinorSubsystem(Robot robot){
        this.opMode = robot.getOpMode();
        this.telemetry = robot.getTelemetry();
        this.hardwareMap = opMode.hardwareMap;
        this.timer = robot.getTimer();
    }
}

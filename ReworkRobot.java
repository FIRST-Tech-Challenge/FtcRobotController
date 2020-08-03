package org.firstinspires.ftc.teamcode.rework;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rework.Modules.ReworkDrivetrain;

public class ReworkRobot {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;

    // Modules on the robot
    protected ReworkDrivetrain drivetrain;

    public ReworkRobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        initModules();
    }

    private void initModules() {
        drivetrain = new ReworkDrivetrain(hardwareMap);
    }
}

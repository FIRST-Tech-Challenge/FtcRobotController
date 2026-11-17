package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.NewHardwareMap;
import org.firstinspires.ftc.teamcode.Core.OldPinpoint;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;
import org.firstinspires.ftc.teamcode.Core.HWMap;
import org.firstinspires.ftc.teamcode.Core.RobotSettings;


@TeleOp(group = "Drivetrain", name = "Old Pinpoint")
public class OldPinpointTest extends LinearOpMode {

    public MechanumHardwareMap mechanumHardwareMap;
    public OldPinpoint pinpoint;
    private RobotSettings robotSettings;

    @Override
    public void runOpMode() {
        mechanumHardwareMap = new MechanumHardwareMap(hardwareMap);
        pinpoint = new OldPinpoint(mechanumHardwareMap, robotSettings, false);

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            telemetry.addData("X pos", pinpoint.getX());
            telemetry.addData("Y pos", pinpoint.getY());
            telemetry.addData("Heading", pinpoint.getHeading());
            telemetry.update();
        }
    }
}
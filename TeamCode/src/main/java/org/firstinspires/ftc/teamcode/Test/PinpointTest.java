package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Core.MechanumHardwareMap;
import org.firstinspires.ftc.teamcode.Core.NewHardwareMap;
import org.firstinspires.ftc.teamcode.Core.Pinpoint;


@TeleOp(group = "Drivetrain", name = "Pinpoint")
public class PinpointTest extends LinearOpMode {

    public MechanumHardwareMap mechanumHardwareMap;
    public Pinpoint pinpoint;

    @Override
    public void runOpMode() {
        mechanumHardwareMap = new MechanumHardwareMap(hardwareMap);
        pinpoint = new Pinpoint(mechanumHardwareMap);

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
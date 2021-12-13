package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.chassis.MecanumChassis;

@Autonomous(group="Test")
public class EncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumChassis chassis = new MecanumChassis();
        chassis.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("fl", chassis.frontLeft.getCurrentPosition());
            telemetry.addData("fr", chassis.frontRight.getCurrentPosition());
            telemetry.addData("bl", chassis.backLeft.getCurrentPosition());
            telemetry.addData("br", chassis.backRight.getCurrentPosition());
            telemetry.update();
        }
    }
}

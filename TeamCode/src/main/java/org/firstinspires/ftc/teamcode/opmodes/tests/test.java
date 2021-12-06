package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
public class test extends LinearOpMode {
    @Override
    public void runOpMode() {
        final DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "tapeMeasure"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backEncoder"));
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("leftEncoder",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEncoder",rightEncoder.getCurrentPosition());
            telemetry.addData("frontEncoder",frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}

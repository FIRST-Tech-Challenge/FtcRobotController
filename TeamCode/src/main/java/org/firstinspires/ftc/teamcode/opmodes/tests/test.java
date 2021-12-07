package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
public class test extends LinearOpMode {
    @Override
    public void runOpMode() {
        final DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        final int leftOffset = Math.abs(leftEncoder.getCurrentPosition());
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "tapeMeasure"));
        final int rightOffset = Math.abs(rightEncoder.getCurrentPosition());
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backEncoder"));
        final int frontOffset = Math.abs(frontEncoder.getCurrentPosition());
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final ControllerMovement move = new ControllerMovement(hardwareMap,moveGamepad);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("leftEncoder",leftEncoder.getCurrentPosition() - leftOffset);
            telemetry.addData("rightEncoder",rightEncoder.getCurrentPosition() - rightOffset);
            telemetry.addData("frontEncoder",frontEncoder.getCurrentPosition() - frontOffset);
            telemetry.update();
            move.update();
        }
    }
}

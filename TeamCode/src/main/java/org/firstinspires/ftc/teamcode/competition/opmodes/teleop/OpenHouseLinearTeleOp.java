package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;

@TeleOp(name="OpenHouseTeleOp", group="linear")
public class OpenHouseLinearTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Motor r = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor l = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.REVERSE);
        Motor intake = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {
            r.driveWithEncoder((int) Range.clip((-gamepad1.left_stick_y + gamepad1.right_stick_x) * 100, -100, 100));
            l.driveWithEncoder((int) Range.clip((-gamepad1.left_stick_y - gamepad1.right_stick_x) * 100, -100, 100));
            intake.driveWithEncoder((int) Range.clip(gamepad1.left_trigger * 100, -100, 100));
        }
        r.stop();
        l.stop();
        intake.stop();
        stop();
    }

}

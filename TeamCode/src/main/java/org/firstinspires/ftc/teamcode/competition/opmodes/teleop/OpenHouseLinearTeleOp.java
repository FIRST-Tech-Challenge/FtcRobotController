package org.firstinspires.ftc.teamcode.competition.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.ButtonPriority;
import org.firstinspires.ftc.teamcode.competition.utils.GamepadExtended;
import org.firstinspires.ftc.teamcode.competition.utils.Motor;

@TeleOp(name="OpenHouseTeleOp", group="linear")
public class OpenHouseLinearTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        resetStartTime();
        Motor r = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1), DcMotorSimple.Direction.REVERSE);
        Motor l = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1), DcMotorSimple.Direction.FORWARD);
        Motor intake = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_SPINNER), DcMotorSimple.Direction.FORWARD);
        Motor lift = new Motor(telemetry, hardwareMap, hardwareMap.appContext.getString(R.string.HW_LIFT), DcMotorSimple.Direction.FORWARD);
        waitForStart();
        resetStartTime();
        while(opModeIsActive()) {
            r.driveWithEncoder((int) Range.clip((-gamepad1.left_stick_y + gamepad1.right_stick_x) * 100, -100, 100));
            l.driveWithEncoder((int) Range.clip((-gamepad1.left_stick_y - gamepad1.right_stick_x) * 100, -100, 100));
            // i couldnt merge michael's changes easily, so i rewrote a simple version of the priority using the dpad
            if(gamepad1.dpad_up) {
                intake.driveWithEncoder(100);
            }else if(gamepad1.dpad_down) {
                intake.driveWithEncoder(-100);
            }else if(gamepad2.dpad_up) {
                intake.driveWithEncoder(100);
            }else if(gamepad2.dpad_down) {
                intake.driveWithEncoder(-100);
            }
            if(gamepad1.dpad_left) {
                lift.driveWithEncoder(10);
            }else if(gamepad1.dpad_right) {
                lift.driveWithEncoder(10);
            }else if(gamepad2.dpad_left) {
                lift.driveWithEncoder(10);
            }else if(gamepad2.dpad_right) {
                lift.driveWithEncoder(10);
            }
        }
        r.stop();
        l.stop();
        intake.stop();
        lift.stop();
        stop();
    }

}

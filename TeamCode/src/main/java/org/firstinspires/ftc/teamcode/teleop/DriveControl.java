package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
@TeleOp(name = "DriveControl")
public class DriveControl extends OpMode {
    final double MAX_DRIVE_POWER = 0.5;
    MecanumDrive drive;

    @Override
    public void init() {
        this.drive = new MecanumDrive(
            MAX_DRIVE_POWER,
            hardwareMap.get(DcMotor.class, "rightFront"),
            hardwareMap.get(DcMotor.class,"rightRear"),
            hardwareMap.get(DcMotor.class,"leftFront"),
            hardwareMap.get(DcMotor.class,"leftRear"));
    }

    @Override
    public void loop() {
        this.drive.shift(this.gamepad1.left_stick_x, this.gamepad1.left_stick_y);
        this.drive.rotate(this.gamepad1.right_stick_x);
        this.drive.updatePowers();
    }
}

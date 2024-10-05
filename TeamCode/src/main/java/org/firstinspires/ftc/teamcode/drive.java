package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;

@TeleOp
public class drive extends OpMode {

    MecanumEncoder drive = new MecanumEncoder(this);

    @Override
    public void init() {
        // run once when init is pressed
        drive.initHardware(hardwareMap);
        drive.resetYaw();

    }

    @Override
    public void init_loop() {
        // runs after the init is pressed
    }

    @Override
    public void loop() {
        // runs while in play
        drive.driverInput(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1.0, MecanumEncoder.DriveMode.FieldCentric);
    }
}


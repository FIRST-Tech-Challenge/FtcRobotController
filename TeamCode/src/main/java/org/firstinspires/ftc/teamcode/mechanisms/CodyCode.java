/*package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.CodyDrive;

@TeleOp(name = "CodyDrive")

public class CodyCode extends OpMode {

    CodyDrive drive = new CodyDrive();
    double forward, strafe, rotate;

    @Override
    public void init() {
       drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.drive(forward, strafe, rotate);
    }

}*/
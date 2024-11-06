package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// @TeleOp
@Disabled
public class AndyMarkTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor andymark = hardwareMap.get(DcMotor.class, "andymark");

        andymark.setPower(1);
    }

}

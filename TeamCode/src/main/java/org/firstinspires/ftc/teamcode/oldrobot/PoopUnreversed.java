package org.firstinspires.ftc.teamcode.oldrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class PoopUnreversed extends Poop {
    @Override
    public void runOpMode() throws InterruptedException {
        reverse = false;
        super.runOpMode();
    }
}

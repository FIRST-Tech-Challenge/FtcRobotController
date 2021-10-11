package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor bl,br,fl,fr;
        double straight, strafe, rotation;

        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            straight = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotation = gamepad1.right_stick_x;

            bl.setPower(straight + strafe + rotation);
            br.setPower(straight - strafe - rotation);
            fl.setPower(straight - strafe + rotation);
            fr.setPower(straight + strafe - rotation);
        }
    }
}

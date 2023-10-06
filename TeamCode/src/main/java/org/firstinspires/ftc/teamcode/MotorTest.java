package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorTest extends LinearOpMode {
    private DcMotor TestMotor = null;
    @Override

    public void runOpMode() throws InterruptedException{
        waitForStart();
        if (isStopRequested())
            return;
        while (opModeIsActive()) {
        TestMotor = hardwareMap.get(DcMotor.class, "motor1");
        TestMotor.setPower(0.75);
        wait(2);
        TestMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
}

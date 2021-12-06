package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleToggleOp", group = "Iterative Opmode")
@Disabled
public class TeleToggleOp extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;

    @Override
    public void init() {
        telemetry.addData("Status", "meh");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "lmotorized");
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    boolean lock = false;
    boolean state = false;

    @Override
    public void loop() {
        if (gamepad1.a && !lock) {
            lock = true;
            if (state) {
                leftMotor.setPower(0);
                state = false;

            } else {
                leftMotor.setPower(1);
                state = true;

            }
            telemetry.addData("Status", "good");
            telemetry.addData("Status", leftMotor.getPower());
        } else if (!gamepad1.a && lock) {
            lock = false;
        }
    }

    @Override
    public void stop() {

    }

}

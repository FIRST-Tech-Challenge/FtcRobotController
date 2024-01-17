package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Memdev.Ezekiel;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

@Disabled
@TeleOp
public class TestTeleOp extends LinearOpMode{
    private DcMotor[] motors;
    private IMU imu;
    private Test test;

    @Override
    public void runOpMode() throws InterruptedException{
        Gamepad g1 = new Gamepad();

        motors = new DcMotor[] {
                hardwareMap.dcMotor.get("Left Motor 1"),
                hardwareMap.dcMotor.get("Left Motor 2"),
                hardwareMap.dcMotor.get("Right Motor 1"),
                hardwareMap.dcMotor.get("Right Motor 2"),
        };
        imu = hardwareMap.get(IMU.class, "imu");
        test = new Test(motors, imu);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            test.move(g1.left_stick_y, g1.left_stick_x, g1.right_stick_x);
        }
    }
}

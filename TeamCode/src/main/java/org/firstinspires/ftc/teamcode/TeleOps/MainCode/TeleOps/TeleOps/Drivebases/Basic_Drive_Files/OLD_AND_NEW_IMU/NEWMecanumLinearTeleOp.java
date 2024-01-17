package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Basic_Drive_Files.OLD_AND_NEW_IMU;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp
public class NEWMecanumLinearTeleOp extends LinearOpMode {
    BOTHMecanum mec;

    private DcMotor[] motors;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        motors = new DcMotor[] {
                hardwareMap.dcMotor.get("fl"),
                hardwareMap.dcMotor.get("fr"),
                hardwareMap.dcMotor.get("bl"),
                hardwareMap.dcMotor.get("br"),
        };

        imu = hardwareMap.get(IMU.class, "imu");

        mec = new BOTHMecanum(motors, imu);

        waitForStart();

        while (opModeIsActive()) {
            mec.NEWdrive(gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);

            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                mec.NEWreset();
                gamepad1.rumble(250); //Angle recalibrated
            }
        }
    }
}

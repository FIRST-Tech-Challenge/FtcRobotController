package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.Drivebases.Basic_Drive_Files.OLD_AND_NEW_IMU;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled

@TeleOp
public class BNO055CommandTeleOp extends CommandOpMode {

    BOTHMecanum drivetrain;

    private DcMotor[] motors;

//    private IMU imu;


    @Override
    public void initialize() {
        //Initializing hardware
        GamepadEx mechanism = new GamepadEx(gamepad2);

        motors = new DcMotor[] {
                hardwareMap.dcMotor.get("fl"),
                hardwareMap.dcMotor.get("fr"),
                hardwareMap.dcMotor.get("bl"),
                hardwareMap.dcMotor.get("br"),
        };


        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu = hardwareMap.get(IMU.class, "imu");
        drivetrain = new BOTHMecanum(motors, imu);


        telemetry.addLine("Initialization Done");
        telemetry.update();
    }

    @Override
    public void run(){
        super.run();
        drivetrain.OLDdrive(gamepad1.left_stick_y, -gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            drivetrain.OLDreset();
            gamepad1.rumble(250); //Angle recalibrated
        }
    }
}

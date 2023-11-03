package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name = "AutoLeauge0blue", group = "Robot")
public class AutoLeaugeBlue0 extends LinearOpMode {

    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive;


    @Override
    public void runOpMode() {
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");
        telemetry.addData("Robot", "ready");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            left_drive.setPower(1); // left drive is 0
            right_drive.setPower(1); // right drive is 2
            back_left_drive.setPower(1); // back left drive is 1
            back_right_drive.setPower(1); // back right drive is 3
            sleep(270);
            left_drive.setPower(-1); // left drive is 0
            right_drive.setPower(-1); // right drive is 2
            back_left_drive.setPower(-1); // back left drive is 1
            back_right_drive.setPower(-1); // back right drive is 3
            sleep(180);
            left_drive.setPower(-1); // left drive is 0
            right_drive.setPower(-1); // right drive is 2
            back_left_drive.setPower(1); // back left drive is 1
            back_right_drive.setPower(1); // back right drive is 3
            sleep(2500);
            drive(0);
        }

    }

    private void drive(double speed) {
        left_drive.setPower(speed);
        right_drive.setPower(speed);
        back_left_drive.setPower(speed);
        back_right_drive.setPower(speed);
    }


}


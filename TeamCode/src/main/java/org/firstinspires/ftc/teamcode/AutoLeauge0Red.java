

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name = "AutoLeauge0Red", group = "Robot")
public class AutoLeauge0Red  extends LinearOpMode {

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

            drive(1);
            sleep(300);
            left_drive.setPower(1);
            right_drive.setPower(1);
            back_left_drive.setPower(-1);
            back_right_drive.setPower(-1);
            sleep(2300);
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


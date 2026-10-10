package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="DemoBot", group="Iterative Opmode")
public class DemoBot extends LinearOpMode {
    public DcMotor left = null;
    public DcMotor right = null;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "left_Forward");
        right = hardwareMap.get(DcMotor.class, "right_Forward");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y;
            double twist = -gamepad1.right_stick_x;

            double[] speeds = {(drive - twist), (drive +  twist)};

            double max = Math.abs(speeds[0]);
            for (int i = 0; i < speeds.length; i++) {
                if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
            }
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            left.setPower(speeds[0]);
            right.setPower(speeds[1]);
        }
    }
}

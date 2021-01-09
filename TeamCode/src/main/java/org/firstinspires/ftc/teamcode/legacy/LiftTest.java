package org.firstinspires.ftc.teamcode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SciLift;

@TeleOp(name="Lift Test", group="Linear Opmode")

public class LiftTest extends LinearOpMode {

    private SciLift lift;

    @Override
    public void runOpMode() {


        lift = new SciLift(
                hardwareMap.get(DcMotor.class, "liftmotor")
        );

        waitForStart();
        while (opModeIsActive()) {

            if (-gamepad2.right_stick_y > 0) {
                lift.up(Math.abs(gamepad2.right_stick_y));
            } else if (-gamepad2.right_stick_y < 0){
                lift.down(Math.abs(gamepad2.right_stick_y));
            } else {
                lift.rest();
            }

            telemetry.addData("Status", "Run Time: ");
//            telemetry.addData("Collect Power", collector.getPower());
            // telemetry.addData("Dist Sensor", collector.getDistance());
            telemetry.addData("Ly", gamepad1.left_stick_y);
            telemetry.addData("Lx", gamepad1.left_stick_x);
            telemetry.addData("Rx", gamepad1.right_stick_x);
            telemetry.addData("Lift", lift.getClicks());
            telemetry.update();

        }
    }
}

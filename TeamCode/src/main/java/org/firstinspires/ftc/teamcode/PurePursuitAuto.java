package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.purepursuit.PurePursuit;


@Autonomous(name="PurePursuitAuto", group="Linear Opmode2")
public class PurePursuitAuto extends LinearOpMode {
    private MecanumPurePursuitController controller;


    @Override
    public void runOpMode() {
        // Initialize controller with starting position
        controller = new MecanumPurePursuitController(hardwareMap, 0, 0);

        // Add path segments
        controller.addPathSegment(0, 0, 2, 0);// Example path
        controller.addPathSegment(2, 0, 2, 2);
        controller.setPathEndPoint(2, 2);  // Set final destination



        waitForStart();

        while (opModeIsActive() && !controller.isPathComplete()) {
            controller.followPath();
            telemetry.addData("Current X", controller.current_X);
            telemetry.addData("Current Y", controller.current_Y);
            telemetry.addData("fl",controller.fl);
            telemetry.addData("bl",controller.bl);
            telemetry.addData("br",controller.br);
            telemetry.addData("fr",controller.fr);
            telemetry.addData("targetA",controller.targetA);
            telemetry.addData("dxReceiver",controller.dxReceiver);
            telemetry.update();

        }

        controller.stopMotors();
    }
}

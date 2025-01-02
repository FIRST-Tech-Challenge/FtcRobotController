package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.purepursuit.PurePursuit;


@Autonomous(name="PurePursuitAuto", group="Linear Opmode2")
public class PurePursuitAuto extends LinearOpMode {
    private MecanumPurePursuitController controller;
//    private MecanumPurePursuitController controller1;

    @Override
    public void runOpMode() {
        // Initialize controller with starting position
        controller = new MecanumPurePursuitController(hardwareMap, 0, 0);

        // Add path segments
        controller.addPathSegment(0, 0, 5, 5);
        controller.addPathSegment(5, 5, 8, 0);
        controller.addPathSegment(8, 0, 12, 12);
        controller.setPathEndPoint(12,12 );  // Set final destination

//        controller1 = new MecanumPurePursuitController(hardwareMap, 0, 2);
//
//         //Add path segments
//        controller1.addPathSegment(0, 2, 0, 0); // Example path
//        controller1.setPathEndPoint(0, 0);

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
       // sleep(2000);
//        while (opModeIsActive()&& !controller1.isPathComplete()){
//            controller1.followPath();
//            telemetry.addData("Current X", controller1.current_X);
//            telemetry.addData("Current Y", controller1.current_Y);
//            telemetry.addData("fl",controller1.fl);
//            telemetry.addData("bl",controller1.bl);
//            telemetry.addData("br",controller1.br);
//            telemetry.addData("fr",controller1.fr);
//            telemetry.addData("targetA",controller1.targetA);
//            telemetry.addData("dxReceiver",controller1.dxReceiver);
//            telemetry.update();
//
//        }
//        controller1.stopMotors();
    }
}

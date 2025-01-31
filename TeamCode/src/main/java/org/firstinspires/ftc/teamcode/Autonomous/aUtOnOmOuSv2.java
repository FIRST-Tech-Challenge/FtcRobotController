package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Systems.BotTelemetry;
import org.firstinspires.ftc.teamcode.Systems.IMU;
import org.firstinspires.ftc.teamcode.Systems.Input;

@Autonomous(name="Auto-Main-better")

public class aUtOnOmOuSv2 extends LinearOpMode {


    private boolean end;

    public void runOpMode() throws InterruptedException {
        Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry(); //AND THIS BEFORE COMPETITION also line 109
        BotTelemetry.setTelemetry(telemetry, dashboardTelemetry);

        Input input = new Input(hardwareMap, true);
        IMU imu = new IMU(hardwareMap);

        ElapsedTime time = new ElapsedTime();

        BotTelemetry.addData("Before Start", 0);
        BotTelemetry.update();

        waitForStart();

        imu.SetYaw();

        input.claw(false,true);
        input.automaticallyMoveWrist(false);

        BotTelemetry.addData("After Start", 0);
        BotTelemetry.update();


        armSynchronous a1 = new armSynchronous(input);
        holdYawSync yawSync = new holdYawSync(input);

        Actions.setupActions(input, a1, imu, yawSync);

        a1.start();
        //yawSync.start();


        while (opModeIsActive()) {
            
            while (!end) {

                BotTelemetry.addData("Active", 0);
                BotTelemetry.update();


                Actions.opModeIsActive(opModeIsActive());

//                a1.setPos(pos);


//                Actions.hangAndReleaseSpecimen();
//                a1.extendArm(-2550, 0);
//
//
//                pos = 1639;
//                a1.setPos(pos);
//
//                a1.retractArm(-1500, 1639);


                //a1.stop();

                /** Step 1 */
            Actions.driveToBarFromInitialPositionForSpecimen();
            Actions.hangAndReleaseSpecimen();

////
//                /** Step 2 */
//            Actions.driveBehindSampleFromLocations(1,"bar");
//            Actions.pushOrReverseSampleToHumanPlayer("push");
////
////            // Step 3
//            Actions.pushOrReverseSampleToHumanPlayer("reverse");
//            Actions.driveRightUntilBehindSample();
////
////            // Step 4
//            Actions.pushOrReverseSampleToHumanPlayer("push");
////
////            // Step 5
//            Actions.grabSpecimen(1);
//            Actions.driveToInitialPositionFromHumanPlayerPitStopX(2);
//            Actions.driveToBarFromInitialPositionForSpecimen();
//            Actions.hangAndReleaseSpecimen();
////
////            //Step 6
//            Actions.driveBehindSampleFromLocations(3, "bar");
//            Actions.pushOrReverseSampleToHumanPlayer("push");
////
////            //Step 7
//            Actions.grabSpecimen(2);
//            Actions.driveToInitialPositionFromHumanPlayerPitStopX(3);
//            Actions.driveToBarFromInitialPositionForSpecimen();
//            Actions.hangAndReleaseSpecimen();
////
//            //Step 8
//            Actions.driveToHumanPlayerFromBarForFinalSpecimen();
//            Actions.grabSpecimen(3);
//            Actions.driveToInitialPositionFromHumanPlayerPitStopX(1);
//            Actions.driveToBarFromInitialPositionForSpecimen();
//            Actions.hangAndReleaseSpecimen();
//
//            Actions.driveToHumanPlayerFromBarForFinalSpecimen();
//            Actions.grabSpecimen(4);
//            Actions.driveToInitialPositionFromHumanPlayerPitStopX(1);
//            Actions.driveToBarFromInitialPositionForSpecimen();
//            Actions.hangAndReleaseSpecimen();
////
//            //Step 9
//            Actions.park();


//            Actions.driveBehindSampleFromLocations(1, "human player");


                end = true;
            }
            a1.stop();
            yawSync.stop();
        }
        a1.stop();
        yawSync.stop();
    }
}

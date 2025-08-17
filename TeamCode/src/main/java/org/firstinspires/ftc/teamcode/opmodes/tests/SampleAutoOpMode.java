package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;


@Config
@Autonomous (name = "Sample Auto")
public class SampleAutoOpMode extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private TelemetryPacket packet;
    private FtcDashboard dash;
    PinPointOdometrySubsystem pinPointOdo;
    private int stage1 = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        enum AUTO_STATE {
            FIRST_BUCKET,
            SECOND_BUCKET,
            THIRD_BUCKET,
            FOURTH_BUCKET,
            SUB_PICKUP,

            FIFTH_BUCKET
        }
        boolean firstInstance = true;
        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        packet = new TelemetryPacket();
        ElapsedTime timer;


        AUTO_STATE autoState = AUTO_STATE.FIRST_BUCKET;

        while (opModeIsActive()) {

            // run processes
            updateTelemetry();
            motorProcess();
            processPinPoint();

            switch (autoState) {
                case FIRST_BUCKET:
                    processFirstBucket();
                    break;

                default:
                    break;
            }
        }

    }


    public void updateTelemetry() {
        //telemetry.addData("nan occurences: ", pinPointOdo.getNanCounter());
        packet.put("x: ", pinPointOdo.getX());
        packet.put("y: ", pinPointOdo.getY());
        packet.put("theta: ", pinPointOdo.getHeading());

    }

    private void motorProcess() {
        mecanumSubsystem.motorProcessNoEncoder();
    }

    public void processPinPoint() {
        pinPointOdo.deadReckoning();

    }

    private void processFirstBucket() {
        switch (stage1) {
            case 0:
                //     levelPositional = 4;
                //   moveToPos(11,-45,-0.77,false,0,0);
                break;
            case 1:
                // waitPosition(2600);
                break;
            case 2:
                // outputCommand.outputToBucket();
                // waitTime(200);
                break;
            case 3:
                // outputCommand.openClaw();
                //   waitTime(100);
                break;
            case 4:
                stage1++;
                break;
            case 5:
                // levelPositional = 0;
                stage1++;
                break;
            case 6:
//                autoState = AUTO_STATE.SUB_PICKUP;
                stage1 = 0;


        }
    }
}

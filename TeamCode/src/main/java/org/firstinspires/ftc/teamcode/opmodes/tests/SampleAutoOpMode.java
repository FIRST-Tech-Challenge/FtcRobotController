package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

import org.firstinspires.ftc.teamcode.Hardware;
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
    private int stage1 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware(hardwareMap);
        mecanumCommand = new MecanumCommand(this, hw);

        enum AUTO_STATE {
            FIRST_BUCKET,
            SECOND_BUCKET,
            THIRD_BUCKET,
            FOURTH_BUCKET,
            SUB_PICKUP,
            FINISH

        }
        boolean firstInstance = true;
        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        packet = new TelemetryPacket();
        ElapsedTime timer;


        AUTO_STATE autoState = AUTO_STATE.FIRST_BUCKET;
waitForStart();
        while (opModeIsActive()) {
            // run processes
            updateTelemetry();
            motorProcess();
            processPinPoint();
            switch (autoState) {
                case FIRST_BUCKET:
                    if(mecanumCommand.moveToPos(0, 10,0.8)){
                        autoState = AUTO_STATE.SUB_PICKUP;
                    }
                    break;

//                case SUB_PICKUP:
//                    mecanumCommand.moveGlobalPartialPinPoint(true, 10, 15,0.8);
//                    autoState = AUTO_STATE.FINISH;
//                    break;

                case FINISH:
                    stopRobot();
                    break;
            }
            updateTelemetry();

        }

    }

    public void updateTelemetry() {
        //telemetry.addData("nan occurences: ", pinPointOdo.getNanCounter());
        packet.put("x: ", mecanumCommand.getOdoX());
        packet.put("y: ", mecanumCommand.getOdoY());
        packet.put("theta: ", mecanumCommand.getOdoHeading());

    }



    private void stopRobot() {
        mecanumCommand.moveGlobalPartialPinPoint(false, 0, 0, 0);
    }
    private void motorProcess() {
        mecanumSubsystem.motorProcessNoEncoder();
    }



    public void processPinPoint() {


    }

}

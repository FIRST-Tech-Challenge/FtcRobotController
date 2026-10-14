package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.KronBot;

import java.util.logging.Logger;

import dev.nextftc.robot.Telemetry;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop(name = "Simple Driving TeleOp", group = "Teleop")

public class SimpleDrivingTeleOp extends NextOpMode {
    private final KronBot robot;
    private Limelight3A limelight;

    int pipelineNumber = 3;

    CommandGamepad gp1 = new CommandGamepad(gamepad1);

    public SimpleDrivingTeleOp(KronBot robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        Telemetry.log("pipeline", pipelineNumber);
        limelight.start();
        robot.drivetrain.startMecanumDrive(gamepad1);
    }
    public void periodic(){
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                Telemetry.log("tx", result.getTx());
                Telemetry.log("ty", result.getTy());
                Telemetry.log("Botpose", botpose.toString());
            }
        }
        if(gamepad1.circle) pipelineNumber = 3;
        if(gamepad1.square) pipelineNumber = 4;
        limelight.pipelineSwitch(pipelineNumber);
        Telemetry.log("pipeline" , pipelineNumber);
        Telemetry.update();
    }
}
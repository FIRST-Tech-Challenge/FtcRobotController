package org.firstinspires.ftc.teamcode.drive.actuators;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drive.camera.SensorLimelight3A;
@TeleOp
public class Garra extends OpMode {
    Servo rotate;
    Servo garra;
    Servo pleft;
    Servo pright;
    public static Limelight3A limelight;
    public static double position(double angle){
        return (angle + 15) / 30;
    }

    public void init() {
        rotate = hardwareMap.get(Servo.class, "rotate");
        garra = hardwareMap.get(Servo.class, "garra");
        pleft = hardwareMap.get(Servo.class, "pleft");
        pright = hardwareMap.get(Servo.class, "pright");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                rotate.setPosition(position(limelight.getLatestResult().getTyNC()));
                telemetry.addData("rotate", rotate.getPosition());
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.update();
        }


            if (gamepad1.a) {
                garra.setPosition(0.3);
            }
            if (gamepad1.b) {
                garra.setPosition(0.7);
            }
            if (gamepad1.x) {
                pleft.setPosition(0.8);
                pright.setPosition(0.2);
            }
            if (gamepad1.y) {
                pleft.setPosition(0);
                pright.setPosition(1);
            }
        }
    }
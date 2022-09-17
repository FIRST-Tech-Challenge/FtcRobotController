package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.masters.MultipleCameraCV;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name="Fblthp")
public class TestWarehouseCV extends LinearOpMode {

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);
        drive.openCVInnitShenanigans("blue");
        telemetry.addData("after open cv", "test");
        telemetry.update();
        MultipleCameraCV.WarehousePipeline.FreightPosition freightPosition = drive.analyzeWarehouse();
        waitForStart();

        drive.getWarehouseFreight();
    }
}

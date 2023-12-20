package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.openftc.easyopencv.OpenCvCamera;

@Autonomous
public class luatDinStack extends LinearOpMode {
    double rectx, recty, hperw,x;
    public DcMotorEx motorBR,motorBL,motorFR,motorFL;
    int varrez = 2;
    public OpenCvCamera webcam;
    public PachetelNouAlbastru pipelineRosu = new PachetelNouAlbastru();
    public ChestiiDeAutonom c = new ChestiiDeAutonom();
    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        c.melctarget(2.29,800,3000);
        c.setMacetaPower(-1);
        c.target(-1500,800, c.getSlider(), 3000,1);
    }
}

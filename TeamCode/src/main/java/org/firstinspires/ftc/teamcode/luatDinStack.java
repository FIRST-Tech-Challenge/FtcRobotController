package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Var_Red.CV_detectionType;
import static org.firstinspires.ftc.teamcode.Var_Red.Webcam_h;
import static org.firstinspires.ftc.teamcode.Var_Red.Webcam_w;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

@Autonomous
public class luatDinStack extends LinearOpMode {
    double rectx, recty, hperw,x;
    public DcMotorEx motorBR,motorBL,motorFR,motorFL;
    int varrez = 2;
    public OpenCvCamera webcam;
    public PachetelNouOpenCV pipelineRosu = new PachetelNouOpenCV();
    public ChestiiDeAutonom c = new ChestiiDeAutonom();
    @Override
    public void runOpMode() throws InterruptedException {
        c.init(hardwareMap);
        c.melctarget(2.29,800,3000);
        c.setMacetaPower(-1);
        c.target(-1500,800, c.getSlider(), 3000,1);
    }
}

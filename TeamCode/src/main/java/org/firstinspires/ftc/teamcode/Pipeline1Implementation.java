package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.cameratests.pipeline1Detector;
import com.arcrobotics.ftclib.vision.UGRectDetector;
public class Pipeline1Implementation extends LinearOpMode {
    pipeline1Detector pipeline1Detector;
    @Override
    public void runOpMode(){
        pipeline1Detector = new pipeline1Detector(hardwareMap, "webcam");
    }
}
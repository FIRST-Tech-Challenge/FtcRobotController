package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptCompassCalibration;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.team6220_PowerPlay.AprilTagDetect;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "CVtest", group = "Test")
public class CVtest extends ConeDetection {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        telemetry.addData("distance to center", ConeDetectionPipeline.dist);
        telemetry.addData("grab boolean", ConeDetectionPipeline.grab);
    }
}

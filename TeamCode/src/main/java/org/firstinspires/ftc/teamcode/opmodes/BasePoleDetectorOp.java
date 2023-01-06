package org.firstinspires.ftc.teamcode.opmodes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.BasePoleDetector;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class BasePoleDetectorOp extends RogueBaseAuto {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void executeOrder66() {
        setPoleDetectorAsPipeline();

        while (!opModeIsActive()){
            telemetry.addData("Angle", Math.toDegrees(getPoleDetector().getPoleAngle()));
            telemetry.update();
        }
    }

    public void setPoleDetectorAsPipeline(){
        getCamera().setPipeline(getPoleDetector());
    }

}

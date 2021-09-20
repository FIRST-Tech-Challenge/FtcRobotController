package teamcode.offSeason;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import teamcode.common.AbstractOpMode;
import teamcode.common.Utils;

@TeleOp(name="VISLAM Test")
public class VISLAMTest extends AbstractOpMode {
    Transform2d cameraToRobot = new Transform2d();
    double encoderMeasureCovariance = 1.0;
    Pose2d startingPose = new Pose2d(0,0,new Rotation2d());
    static T265Camera slamra;

    @Override
    protected void onInitialize() {
        if(slamra == null) {
            slamra = new T265Camera(cameraToRobot, encoderMeasureCovariance, hardwareMap.appContext);
        }

    }

    @Override
    protected void onStart() {

        slamra.start();
        Utils.sleep(2000);
        slamra.setPose(startingPose);
        while(opModeIsActive()){

            T265Camera.CameraUpdate update = slamra.getLastReceivedCameraUpdate();
            Translation2d translation = new Translation2d(update.pose.getTranslation().getX() / 0.0254, update.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = update.pose.getRotation();
            telemetry.addData("X", translation.getX());
            telemetry.addData("Y", translation.getY());
            telemetry.addData("Rotation", rotation.getDegrees());
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        slamra.stop();
    }
}

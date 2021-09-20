package teamcode.offSeason;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.Vector;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.RobotPositionStateUpdater;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Autonomous(name="PID test")
public class SimplePIDTest extends AbstractOpMode {

    Localizer localizer;
    MecanumDriveTrain driveTrain;

    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0, 10);
        driveTrain = new MecanumDriveTrain(hardwareMap, localizer);
        localizer.setPriority(10);
    }

    @Override
    protected void onStart() {
        localizer.start();
        driveTrain.moveToPosition(new Vector2D(0,24), 12, 0 , 0);
        //driveTrain.moveToPosition(new Vector2D(0,0), 0, Math.PI / 2.0, 0.4);
        //driveTrain.moveToRotation(Math.PI / 2.0, 0.4);

//        telemetry.addData("LV", localizer.getLeftVerticalOdometerPosition());
//        telemetry.addData("RV", localizer.getRightVerticalOdometerPosition());
//        telemetry.addData("H", localizer.getHorizontalOdometerPosition());


//        Utils.sleep(500);
//        driveTrain.moveToPosition(new Vector2D(48,48), 18, 0);
//        Utils.sleep(500);
//        driveTrain.moveToPosition(new Vector2D(60,60), 12, 0);
//        Utils.sleep(500);
//        driveTrain.moveToPosition(new Vector2D(48,48), 12, 0);
//        Utils.sleep(500);
//        driveTrain.moveToPosition(new Vector2D(24,24), 12, 0);
//        Utils.sleep(500);
//        driveTrain.moveToPosition(new Vector2D(0,0), 12, 0);
//        Debug.log(localizer.getMinElapsedTime());
//        Debug.log(localizer.getMaxElapsedTime());



        while(opModeIsActive()){

        }

    }

    @Override
    protected void onStop() {
      //  ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("KalmanStateReadings.txt"), localizer.loggingString);
        localizer.stopThread();
    }
}

package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.Point;
import teamcode.common.Vector2D;


@Autonomous(name="roll")
public class OdoRoll extends AbstractOpMode {

    Localizer localizer;
    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0, 10);
    }

    @Override
    protected void onStart() {
        localizer.start();
        while(opModeIsActive()){
            //telemetry.addData("leftVertical: ", localizer.getLeftVerticalOdometerPosition());
            //telemetry.addData("rightVertical: ", localizer.getRightVerticalOdometerPosition());
            //telemetry.addData("horizontal: ", localizer.getHorizontalOdometerPosition());
            //telemetry.addData("", localizer.getCurrentState());
            //telemetry.update();
        }
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("KalmanStateReadings.txt"), localizer.loggingString);
    }

    @Override
    protected void onStop() {

        localizer.stopThread();
    }
}

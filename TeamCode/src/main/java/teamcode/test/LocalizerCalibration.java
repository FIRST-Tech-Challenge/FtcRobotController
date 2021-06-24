package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Point;
import teamcode.common.PurePursuit.PurePursuitMovement;
import teamcode.common.Utils;
import teamcode.common.Vector2D;

@Disabled
@Autonomous(name="OdoCalibrate")
public class LocalizerCalibration extends AbstractOpMode {
    Localizer localizer;
    MecanumDriveTrain driveTrain;
    private final double POWER = 0.9;
    private double horizontalPositionSum;
    private double instances;

    @Override
    protected void onInitialize() {
        driveTrain = new MecanumDriveTrain(hardwareMap);
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0);
    }

    @Override
    protected void onStart() {
        new Thread(){
            @Override
            public void run(){
                try {
                    Thread.currentThread().sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                while(AbstractOpMode.currentOpMode().opModeIsActive()){
                    double currentCycleHorizontalPositionOffsetConstant = (localizer.getHorizontalOdometerPosition());
                    horizontalPositionSum += currentCycleHorizontalPositionOffsetConstant;
                    instances++;
                }
            }
        }.start();
//        driveTrain.setPower(-POWER, POWER, POWER, -POWER);
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        driveTrain.setPower(0,0,0,0);
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        Debug.log(((double)(localizer.getHorizontalOdometerPosition()) / localizer.getGlobalRads()) / 8192.0 );
//        while(opModeIsActive());
        driveTrain.setPower(-POWER, POWER, POWER, -POWER);
        Utils.sleep(5000);
        driveTrain.setPower(0,0,0,0);
        Debug.log(Localizer.encoderTicksToInches((int)(horizontalPositionSum / instances)));
        Utils.sleep(5000);

    }

    @Override
    protected void onStop() {

    }
}

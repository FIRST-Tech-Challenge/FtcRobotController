package teamcode.test;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.MecanumDriveTrain;
import teamcode.common.Vector2D;

@TeleOp(name="OdoTest")
public class OdoTeleOpTest extends AbstractOpMode {

    Localizer localizer;
    MecanumDriveTrain driveTrain;

    @Override
    protected void onInitialize() {
        driveTrain = new MecanumDriveTrain(hardwareMap, localizer);
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0);
    }

    @Override
    protected void onStart() {
        localizer.start();
        while(opModeIsActive()){
            //driveTrain.moveToPosition(new Vector2D(0,60), 12, 0);
            //driveTrain.moveToPosition(new Vector2D(0,0), 12, 0);

        }
    }

    @Override
    protected void onStop() {
        localizer.stopThread();
    }
}

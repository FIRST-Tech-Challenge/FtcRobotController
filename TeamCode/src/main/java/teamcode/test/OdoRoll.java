package teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import teamcode.common.AbstractOpMode;
import teamcode.common.Localizer;
import teamcode.common.Point;
import teamcode.common.Vector2D;

@Disabled
@Autonomous(name="roll")
public class OdoRoll extends AbstractOpMode {

    Localizer localizer;
    @Override
    protected void onInitialize() {
        localizer = new Localizer(hardwareMap, new Vector2D(0,0), 0);
    }

    @Override
    protected void onStart() {
        while(opModeIsActive()){
            telemetry.addData("LeftVerticalOdoTicks", localizer.getLeftVerticalOdometerPosition());
            telemetry.addData("RightVerticalOdoTicks", localizer.getRightVerticalOdometerPosition());
            telemetry.addData("HorizontalOdoTicks", localizer.getHorizontalOdometerPosition());

            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }
}

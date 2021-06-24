package teamcode.Competition.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubServo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;

@Disabled
@TeleOp(name="Servo Calibration")
public class ServoCalibration extends AbstractOpMode {
    ExpansionHubServo index;
    @Override
    protected void onInitialize() {
        index = (ExpansionHubServo) hardwareMap.servo.get("Indexer");
        //Debug.log(index);
        //Debug.log(index.getPortNumber());

    }

    @Override
    protected void onStart() {

        index.setPosition(0.15);
        //Debug.log(index.getPosition());
        try {
            Thread.currentThread().sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        index.setPosition(1);
        Debug.log(index.getPosition());
        while(opModeIsActive());


    }

    @Override
    protected void onStop() {

    }
}

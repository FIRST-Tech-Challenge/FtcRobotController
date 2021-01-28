package teamcode.League1.CalibrationClasses;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.common.AbstractOpMode;
import teamcode.common.Debug;
import teamcode.test.revextensions2.ExpansionHubServo;

@TeleOp(name="Servo Calibration")
public class ServoCalibration extends AbstractOpMode {
    ExpansionHubServo index;
    @Override
    protected void onInitialize() {
        index = (ExpansionHubServo) hardwareMap.servo.get("Indexer");
        Debug.log(index);
        Debug.log(index.getPortNumber());

    }

    @Override
    protected void onStart() {

        index.setPosition(0.65);
        Debug.log(index.getPosition());
        try {
            Thread.currentThread().sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        index.setPosition(1.0);
        Debug.log(index.getPosition());
        while(opModeIsActive());


    }

    @Override
    protected void onStop() {

    }
}

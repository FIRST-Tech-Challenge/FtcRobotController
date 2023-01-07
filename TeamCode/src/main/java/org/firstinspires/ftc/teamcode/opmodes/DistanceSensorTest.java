package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodekt.components.DeviceNames;
import org.firstinspires.ftc.teamcodekt.components.LongRangeSensor;
import org.firstinspires.ftc.teamcodekt.components.ShortRangeSensor;

import ftc.rogue.blacksmith.BlackOp;

@TeleOp
public class DistanceSensorTest extends BlackOp {
    private ShortRangeSensor shortLeft;
    private ShortRangeSensor shortRight;
    private LongRangeSensor longLeft;
    private LongRangeSensor longRight;

    @Override
    public void go() {
        PhotonCore.enable();

        shortLeft = new ShortRangeSensor(DeviceNames.SHORT_RANGE_SENSOR_LEFT);
        shortRight = new ShortRangeSensor(DeviceNames.SHORT_RANGE_SENSOR_RIGHT);
        longLeft = new LongRangeSensor(DeviceNames.LONG_RANGE_SENSOR_LEFT);
        longRight = new LongRangeSensor(DeviceNames.LONG_RANGE_SENSOR_RIGHT);

        while(!opModeIsActive()) {
            mTelemetry.addData("Short Left", shortLeft.getDistance());
            telemetry.addData("Short Right", shortRight.getDistance());
            telemetry.addData("Long Left", longLeft.getDistance());
            telemetry.addData("Long Right", longRight.getDistance());
            mTelemetry.update();
        }
    }
}

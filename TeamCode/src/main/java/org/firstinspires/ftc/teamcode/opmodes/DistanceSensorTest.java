package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames;
import org.firstinspires.ftc.teamcodekt.components.LongRangeSensor;

import ftc.rogue.blacksmith.BlackOp;
import ftc.rogue.blacksmith.Scheduler;

@TeleOp
public class DistanceSensorTest extends BlackOp {
    @Override
    public void go() {
        PhotonCore.enable();

//        ShortRangeSensor shortLeft  = new ShortRangeSensor(DeviceNames.SHORT_RANGE_SENSOR_LEFT );
//        ShortRangeSensor shortRight = new ShortRangeSensor(DeviceNames.SHORT_RANGE_SENSOR_RIGHT);
        LongRangeSensor  longLeft   = new LongRangeSensor (DeviceNames.LONG_RANGE_SENSOR_LEFT  );
//        LongRangeSensor  longRight  = new LongRangeSensor (DeviceNames.LONG_RANGE_SENSOR_RIGHT );

        Scheduler.launchOnStart(this, () -> {
//            mTelemetry.addData("Short Left",  shortLeft .getDistance());
//            mTelemetry.addData("Short Right", shortRight.getDistance());
//
//            mTelemetry.addData("Short avg.", (shortLeft.getDistance() + shortRight.getDistance()) / 2.0);

//            mTelemetry.addData("Short Left Avg",  shortLeft.avgOf(12));
//            mTelemetry.addData("Short Right Avg", shortRight.avgOf(12));

            mTelemetry.addData("Long Left",   longLeft.getDistanceRaw());
            mTelemetry.addData("Long Left",   longLeft.distanceLeft());
//            mTelemetry.addData("Long Right",  longRight .distance());

            mTelemetry.update();
        });
    }
}

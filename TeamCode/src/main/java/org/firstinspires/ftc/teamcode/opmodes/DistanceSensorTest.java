package org.firstinspires.ftc.teamcode.opmodes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcodekt.components.DeviceNames;
import org.firstinspires.ftc.teamcodekt.components.LongRangeSensor;
import org.firstinspires.ftc.teamcodekt.components.ShortRangeSensor;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    private ShortRangeSensor shortLeft;
    private ShortRangeSensor shortRight;
    private LongRangeSensor longLeft;
    private LongRangeSensor longRight;

    @Override
    public void runOpMode() throws InterruptedException {
        PhotonCore.enable();
        shortLeft = new ShortRangeSensor(hardwareMap, DeviceNames.SHORT_RANGE_SENSOR_LEFT);
        shortRight = new ShortRangeSensor(hardwareMap, DeviceNames.SHORT_RANGE_SENSOR_RIGHT);
        longLeft = new LongRangeSensor(hardwareMap, DeviceNames.LONG_RANGE_SENSOR_LEFT);
        longRight = new LongRangeSensor(hardwareMap, DeviceNames.LONG_RANGE_SENSOR_RIGHT);
        while(!opModeIsActive()){
            telemetry.addData("Short Left", shortLeft.getDistance());
            telemetry.addData("Short Right", shortRight.getDistance());
            telemetry.addData("Long Left", longLeft.getDistance());
            telemetry.addData("Long Right", longRight.getDistance());
            telemetry.update();
        }
    }
}

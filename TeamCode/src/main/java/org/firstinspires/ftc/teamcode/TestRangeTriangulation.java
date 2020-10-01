package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Ethan on 12/2/2016.
 */

//@TeleOp(name="Omni: TestRangeTriangulation", group ="TeleOp")
public class TestRangeTriangulation extends OpMode {

    /**
     * Hardware Mappings
     */
    public final static String BACK_RIGHT_RANGE = "BackRightTof";
    public final static String BACK_LEFT_RANGE = "BackLeftTof";
    protected Rev2mDistanceSensor backRightTof = null;
    protected Rev2mDistanceSensor backLeftTof = null;

    @Override
    public void init() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);

        backRightTof = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, BACK_RIGHT_RANGE);
        backLeftTof = (Rev2mDistanceSensor)hardwareMap.get(DistanceSensor.class, BACK_LEFT_RANGE);
        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("Back Left TOF: ", backLeftTof.getDistance(DistanceUnit.CM));
        telemetry.addData("Back Right TOF: ", backRightTof.getDistance(DistanceUnit.CM));

    }
}

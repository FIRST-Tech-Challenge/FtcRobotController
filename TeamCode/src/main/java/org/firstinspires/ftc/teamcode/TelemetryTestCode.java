package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "Telemetry Test", group = "linear autoMode")
public class TelemetryTestCode extends RobotLinearOpMode {
    @Override
    public void runOpMode() {
        declareHardwareProperties();
        waitForStart();
        telemetry.addData("Testing", "Test complete");
        telemetry.update();
        sleep(10000);
        telemetry.addData("Testing2", "Changed");
        telemetry.update();
    }
}

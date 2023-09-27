package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

public class BasicOpMode_Iterative extends OpMode {

    private CustomHardwareHandler customHardwareHandler;

    @Override
    public void init() {
        telemetry.addData("Status","Initialized");
        //customHardwareHandler = hardwareMap hardwarehandler;
    }

    @Override
    public void loop() {
        //axial = -gamepad1.left_stick_x;
        //lateral = gamepad1.left_stick_y;
        //yaw = gamepad1.right_stick_x;

        //backLeftPower = Range.clip(axial-lateral+yaw, -1.0, 1.0);
        //backRightPower = Range.clip(axial+lateral-yaw, -1.0, 1.0);
        //frontRightPower = Range.clip(axial-lateral-yaw, -1.0, 1.0);
        //frontLeftPower = Range.clip(axial+lateral+yaw, -1.0, 1.0);

        //backLeft.setPower(backLeftPower);
        //backRight.setPower(backRightPower);
        //frontLeft.set(frontLeftPower);
        //frontRight.set(frontRightPower);
    }
}

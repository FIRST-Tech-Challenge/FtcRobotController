package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * Second chassis only has 4 dc motors. No arm motor or any other servo.
 */
@TeleOp(name = "TeleopBaseChassis", group = "Furious Frog")
public class OpModeBaseChassisTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Make sure your ID's match your configuration

        List<HardwareMap.DeviceMapping<? extends HardwareDevice>> allDeviceMappings = hardwareMap.allDeviceMappings;

        allDeviceMappings.forEach(d -> {
            System.out.println(d.getDeviceTypeClass().getCanonicalName());
        });

        MacanumWheelsTeleop wheels = new MacanumWheelsTeleop(hardwareMap, telemetry);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double chassisY = getChassisY();
            double chassisX = getChassisX();
            double chassisTurn = gamepad1.right_stick_x;
            wheels.move(chassisX, chassisY, chassisTurn);
            telemetry.update();

        }
    }

    private double getChassisX() {
        double chassisX = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        //  System.out.println("gamepad1.left_stick_x is " + chassisX);
        return chassisX;
    }

    private double getChassisY() {
        double chassisY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        //  System.out.println("gamepad1.left_stick_y is " + chassisY);
        return chassisY;
    }
}

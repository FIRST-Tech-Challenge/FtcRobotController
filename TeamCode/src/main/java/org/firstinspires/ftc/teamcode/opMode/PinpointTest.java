package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.subsystems.Imu_Pinpoint;

/**
 * Checks the goBILDA Pinpoint and the two pods plugged into it. Nothing moves; you push.
 *
 * <p>Push the robot forward: {@code X ticks} and {@code X inches} grow. Push it left:
 * {@code Y} grows. Turn it left: {@code Heading} grows. A pod whose count never changes
 * is unplugged from the Pinpoint, or plugged into the wrong port (X is forward, Y is
 * sideways). Press circle to zero everything.
 *
 * <p>The Pinpoint is set up here exactly as the drive sets it up, so a number that is
 * wrong here is wrong in HyDrive and in the autos too.
 */
@TeleOp(name = "Test: Pinpoint", group = "Test")
public class PinpointTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        BotProperties properties = new BotProperties();
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, properties.cfgPinpoint);
        Imu_Pinpoint.Configure(pinpoint, properties);

        while (opModeInInit() || opModeIsActive()) {
            if (gamepad1.circle) {
                pinpoint.resetPosAndIMU();
            }
            pinpoint.update();

            telemetry.addLine("Push forward: X grows. Push left: Y grows. Turn left: heading grows.");
            telemetry.addLine("Circle: zero everything. Hold the robot still for a moment after.");
            telemetry.addLine();
            telemetry.addData("Pinpoint", "%s, version %d", pinpoint.getDeviceStatus(), pinpoint.getDeviceVersion());
            telemetry.addData("X ticks (forward pod)", pinpoint.getEncoderX());
            telemetry.addData("Y ticks (strafe pod)", pinpoint.getEncoderY());
            telemetry.addData("X inches", "%.1f", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y inches", "%.1f", pinpoint.getPosY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg, left +)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addLine();
            telemetry.addData("Pod offsets (in)", "forward pod %.2f sideways, strafe pod %.2f forward",
                    properties.pinpointParYTicks * properties.pinpointInPerTick,
                    properties.pinpointPerpXTicks * properties.pinpointInPerTick);
            telemetry.update();
            idle();
        }
    }
}

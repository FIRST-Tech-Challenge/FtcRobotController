package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.bot.BotProperties;

/**
 * Finds which motor port each odometry pod is plugged into. WHEELS OFF THE GROUND.
 *
 * <p>A pod plugs into the encoder jack of a motor port, so the pod's count shows up as
 * that motor's position. Roll one pod wheel with your finger. The port whose number
 * changes is that pod's port. Then check it against {@code cfgForwardPodPort} and
 * {@code cfgStrafePodPort} in {@link BotProperties}.
 *
 * <p>No motor is ever powered here. Every motor is set to FLOAT, so the wheels turn freely.
 * Press circle to zero the counts.
 */
@TeleOp(name = "Test: Odometry Pods", group = "Test")
public class OdometryPodTest extends LinearOpMode {

    /** Let a wheel be turned by hand: no power, no brake, and a fresh count of 0. */
    static void SetUpToBePushed(DcMotorEx motor) {
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {
        BotProperties properties = new BotProperties();
        String[] portNames = {
                properties.cfgDriveFrontLeft, properties.cfgDriveFrontRight,
                properties.cfgDriveBackLeft, properties.cfgDriveBackRight};
        DcMotorEx[] ports = new DcMotorEx[portNames.length];
        for (int i = 0; i < portNames.length; i++) {
            ports[i] = hardwareMap.get(DcMotorEx.class, portNames[i]);
            SetUpToBePushed(ports[i]);
        }

        // The counts show before and after START, so the check can happen either way.
        while (opModeInInit() || opModeIsActive()) {
            if (gamepad1.circle) {
                for (DcMotorEx port : ports) {
                    SetUpToBePushed(port);
                }
            }

            telemetry.addLine("WHEELS OFF THE GROUND. Roll one pod wheel with a finger.");
            telemetry.addLine("The port whose number changes is that pod's port.");
            telemetry.addLine("Circle: zero the counts.");
            telemetry.addLine();
            for (int i = 0; i < portNames.length; i++) {
                telemetry.addData(portNames[i], ports[i].getCurrentPosition());
            }
            telemetry.addLine();
            telemetry.addData("BotProperties says forward pod is on", properties.cfgForwardPodPort);
            telemetry.addData("BotProperties says strafe pod is on", properties.cfgStrafePodPort);
            telemetry.update();
            idle();
        }
    }
}

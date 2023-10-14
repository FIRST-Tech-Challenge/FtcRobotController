package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFUltrasonic;
import org.firstinspires.ftc.teamcode.Robots.BradBot;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/**
 * William
 * Program to tune the constants of an RFUltrasonic(x,y,a offset), must have inputable expected obstacle in the form of a linear equation
 */
public class RFUltrasonicTest extends LinearOpMode {
    DigitalChannelController controller = new DigitalChannelController() {
        @Override
        public SerialNumber getSerialNumber() {
            return null;
        }

        @Override
        public DigitalChannel.Mode getDigitalChannelMode(int channel) {
            return null;
        }

        @Override
        public void setDigitalChannelMode(int channel, DigitalChannel.Mode mode) {

        }

        @Override
        public void setDigitalChannelMode(int channel, Mode mode) {

        }

        @Override
        public boolean getDigitalChannelState(int channel) {
            return false;
        }

        @Override
        public void setDigitalChannelState(int channel, boolean state) {

        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return null;
        }

        @Override
        public String getConnectionInfo() {
            return null;
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };
    LED ultras = new LED(controller,0);

    BradBot robot = new BradBot(this, false);

    double loopNum = 0;

    public void runOpMode() {
        loopNum++;
        if (loopNum % 10 == 0) {
            ultras.enable(!ultras.isLightOn());
        }

    }

    TrajectorySequence backdrop = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -63.25, Math.toRadians(-90)))
            .setReversed(true)
            .lineToLinearHeading(new Pose2d(-46, -37, toRadians(90)))
            .addTemporalMarker(robot::done)
            .build();

    TrajectorySequence backUp = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -63.25, Math.toRadians(-90)))
            .setReversed(true)
            .lineToLinearHeading(new Pose2d(-46, -37, toRadians(90)))
            .addTemporalMarker(robot::done)
            .build();

    TrajectorySequence deposit = robot.roadrun.trajectorySequenceBuilder(new Pose2d(-38.5, -63.25, Math.toRadians(-90)))
            .setReversed(true)
            .lineToLinearHeading(new Pose2d(-46, -37, toRadians(90)))
            .addTemporalMarker(robot::done)
            .build();
}

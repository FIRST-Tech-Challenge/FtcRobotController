package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public final class AngularRampLogger extends LinearOpMode {
    private static double power(double seconds) {
        return Math.min(0.1 * seconds, 0.9);
    }

    private static class Signal {
        public final List<Double> times = new ArrayList<>();
        public final List<Double> values = new ArrayList<>();
    }

    private static void recordEncoderData(Encoder e, Map<SerialNumber, Double> ts, Signal ps, Signal vs) {
        SerialNumber sn = ((LynxDcMotorController) e.getController()).getSerialNumber();
        Encoder.PositionVelocityPair p = e.getPositionAndVelocity();

        ps.times.add(ts.get(sn));
        ps.values.add((double) p.position);

        vs.times.add(ts.get(sn));
        vs.values.add((double) p.velocity);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(hardwareMap);
        view.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        class Data {
            public final String type = view.type;

            public final List<Signal> leftPowers = new ArrayList<>();
            public final List<Signal> rightPowers = new ArrayList<>();

            public final Signal voltages = new Signal();

            public final List<Signal> leftEncPositions = new ArrayList<>();
            public final List<Signal> rightEncPositions = new ArrayList<>();
            public final List<Signal> parEncPositions = new ArrayList<>();
            public final List<Signal> perpEncPositions = new ArrayList<>();

            public final List<Signal> leftEncVels = new ArrayList<>();
            public final List<Signal> rightEncVels = new ArrayList<>();
            public final List<Signal> parEncVels = new ArrayList<>();
            public final List<Signal> perpEncVels = new ArrayList<>();

            public final List<Signal> angVels = new ArrayList<>();
        }

        Data data = new Data();
        for (DcMotorEx m : view.leftMotors) {
            data.leftPowers.add(new Signal());
        }
        for (DcMotorEx m : view.rightMotors) {
            data.rightPowers.add(new Signal());
        }
        for (Encoder e : view.leftEncs) {
            data.leftEncPositions.add(new Signal());
            data.leftEncVels.add(new Signal());
        }
        for (Encoder e : view.rightEncs) {
            data.rightEncPositions.add(new Signal());
            data.rightEncVels.add(new Signal());
        }
        for (Encoder e : view.parEncs) {
            data.parEncPositions.add(new Signal());
            data.parEncVels.add(new Signal());
        }
        for (Encoder e : view.perpEncs) {
            data.perpEncPositions.add(new Signal());
            data.perpEncVels.add(new Signal());
        }
        for (int i = 0; i < 3; i++) {
            data.angVels.add(new Signal());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        while (opModeIsActive()) {
            for (int i = 0; i < view.leftMotors.size(); i++) {
                double power = -power(t.seconds());
                view.leftMotors.get(i).setPower(power);

                Signal s = data.leftPowers.get(i);
                s.times.add(t.addSplit());
                s.values.add(power);
            }

            for (int i = 0; i < view.rightMotors.size(); i++) {
                double power = power(t.seconds());
                view.rightMotors.get(i).setPower(power);

                Signal s = data.rightPowers.get(i);
                s.times.add(t.addSplit());
                s.values.add(power);
            }

            data.voltages.values.add(view.voltageSensor.getVoltage());
            data.voltages.times.add(t.addSplit());

            Map<SerialNumber, Double> encTimes = view.resetAndBulkRead(t);

            for (int i = 0; i < view.leftEncs.size(); i++) {
                recordEncoderData(
                        view.leftEncs.get(i),
                        encTimes,
                        data.leftEncPositions.get(i),
                        data.leftEncVels.get(i)
                );
            }
            for (int i = 0; i < view.rightEncs.size(); i++) {
                recordEncoderData(
                        view.rightEncs.get(i),
                        encTimes,
                        data.rightEncPositions.get(i),
                        data.rightEncVels.get(i)
                );
            }
            for (int i = 0; i < view.parEncs.size(); i++) {
                recordEncoderData(
                        view.parEncs.get(i),
                        encTimes,
                        data.parEncPositions.get(i),
                        data.parEncVels.get(i)
                );
            }
            for (int i = 0; i < view.perpEncs.size(); i++) {
                recordEncoderData(
                        view.perpEncs.get(i),
                        encTimes,
                        data.perpEncPositions.get(i),
                        data.perpEncVels.get(i)
                );
            }

            AngularVelocity av = view.imu.getRobotAngularVelocity(AngleUnit.RADIANS);
            for (int i = 0; i < 3; i ++) {
                data.angVels.get(i).times.add(t.addSplit());
            }
            data.angVels.get(0).values.add((double) av.xRotationRate);
            data.angVels.get(1).values.add((double) av.yRotationRate);
            data.angVels.get(2).values.add((double) av.zRotationRate);
        }

        for (DcMotorEx m : view.motors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.ANGULAR_RAMP, data);
    }
}

package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;

public final class AngularRampLogger extends LinearOpMode {
    private static double power(double seconds) {
        return Math.min(0.1 * seconds, 0.9);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(hardwareMap);

        class Data {
            public final String type = view.type;

            public final List<List<Double>> leftPowerTimes = new ArrayList<>();
            public final List<List<Double>> leftPowers = new ArrayList<>();

            public final List<List<Double>> rightPowerTimes = new ArrayList<>();
            public final List<List<Double>> rightPowers = new ArrayList<>();

            public final List<Double> voltageTimes = new ArrayList<>();
            public final List<Double> voltages = new ArrayList<>();

            public final List<Double> encTimes = new ArrayList<>();
            public final List<List<Integer>> leftEncPositions = new ArrayList<>();
            public final List<List<Integer>> leftEncVels = new ArrayList<>();
            public final List<List<Integer>> rightEncPositions = new ArrayList<>();
            public final List<List<Integer>> rightEncVels = new ArrayList<>();
            public final List<List<Integer>> parEncPositions = new ArrayList<>();
            public final List<List<Integer>> parEncVels = new ArrayList<>();
            public final List<List<Integer>> perpEncPositions = new ArrayList<>();
            public final List<List<Integer>> perpEncVels = new ArrayList<>();

            public final List<Double> angVelTimes = new ArrayList<>();
            public final List<List<Double>> angVels = new ArrayList<>();
        }

        Data data = new Data();
        for (DcMotorEx m : view.leftMotors) {
            data.leftPowerTimes.add(new ArrayList<>());
            data.leftPowers.add(new ArrayList<>());
        }
        for (DcMotorEx m : view.rightMotors) {
            data.rightPowerTimes.add(new ArrayList<>());
            data.rightPowers.add(new ArrayList<>());
        }
        for (Encoder e : view.leftEncs) {
            data.leftEncPositions.add(new ArrayList<>());
            data.leftEncVels.add(new ArrayList<>());
        }
        for (Encoder e : view.rightEncs) {
            data.rightEncPositions.add(new ArrayList<>());
            data.rightEncVels.add(new ArrayList<>());
        }
        for (Encoder e : view.parEncs) {
            data.parEncPositions.add(new ArrayList<>());
            data.parEncVels.add(new ArrayList<>());
        }
        for (Encoder e : view.perpEncs) {
            data.perpEncPositions.add(new ArrayList<>());
            data.perpEncVels.add(new ArrayList<>());
        }
        for (int i = 0; i < 3; i++) {
            data.angVels.add(new ArrayList<>());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        while (opModeIsActive()) {
            for (int i = 0; i < view.leftMotors.size(); i++) {
                double power = -power(t.seconds());
                view.leftMotors.get(i).setPower(power);

                data.leftPowers.get(i).add(power);
                data.leftPowerTimes.get(i).add(t.addSplit());
            }

            for (int i = 0; i < view.rightMotors.size(); i++) {
                double power = power(t.seconds());
                view.rightMotors.get(i).setPower(power);

                data.rightPowers.get(i).add(power);
                data.rightPowerTimes.get(i).add(t.addSplit());
            }

            data.voltages.add(view.voltageSensor.getVoltage());
            data.voltageTimes.add(t.addSplit());

            for (int i = 0; i < view.leftEncs.size(); i++) {
                Encoder.PositionVelocityPair p = view.leftEncs.get(i).getPositionAndVelocity();
                data.leftEncPositions.get(i).add(p.position);
                data.leftEncVels.get(i).add(p.velocity);
            }
            for (int i = 0; i < view.rightEncs.size(); i++) {
                Encoder.PositionVelocityPair p = view.rightEncs.get(i).getPositionAndVelocity();
                data.rightEncPositions.get(i).add(p.position);
                data.rightEncVels.get(i).add(p.velocity);
            }
            for (int i = 0; i < view.parEncs.size(); i++) {
                Encoder.PositionVelocityPair p = view.parEncs.get(i).getPositionAndVelocity();
                data.parEncPositions.get(i).add(p.position);
                data.parEncVels.get(i).add(p.velocity);
            }
            for (int i = 0; i < view.perpEncs.size(); i++) {
                Encoder.PositionVelocityPair p = view.perpEncs.get(i).getPositionAndVelocity();
                data.perpEncPositions.get(i).add(p.position);
                data.perpEncVels.get(i).add(p.velocity);
            }
            data.encTimes.add(t.addSplit());

            AngularVelocity av = view.imu.getAngularVelocity();
            data.angVels.get(0).add((double) av.xRotationRate);
            data.angVels.get(1).add((double) av.yRotationRate);
            data.angVels.get(2).add((double) av.zRotationRate);
            data.angVelTimes.add(t.addSplit());
        }

        for (DcMotorEx m : view.motors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.ANGULAR_RAMP, data);
    }
}

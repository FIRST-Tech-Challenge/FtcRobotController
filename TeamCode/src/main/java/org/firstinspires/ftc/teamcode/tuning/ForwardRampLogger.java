package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;

public final class ForwardRampLogger extends LinearOpMode {
    private static double power(double seconds) {
        return Math.min(0.1 * seconds, 0.9);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(hardwareMap);

        class Data {
            public final String type = view.type;

            public final List<List<Double>> powerTimes = new ArrayList<>();
            public final List<List<Double>> powers = new ArrayList<>();

            public final List<Double> voltageTimes = new ArrayList<>();
            public final List<Double> voltages = new ArrayList<>();

            public final List<Double> encTimes = new ArrayList<>();
            public final List<List<Integer>> forwardEncPositions = new ArrayList<>();
            public final List<List<Integer>> forwardEncVels = new ArrayList<>();
        }

        Data data = new Data();
        for (DcMotorEx m : view.motors) {
            data.powerTimes.add(new ArrayList<>());
            data.powers.add(new ArrayList<>());
        }
        for (Encoder e : view.forwardEncs) {
            data.forwardEncPositions.add(new ArrayList<>());
            data.forwardEncVels.add(new ArrayList<>());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        while (opModeIsActive()) {
            for (int i = 0; i < view.motors.size(); i++) {
                double power = power(t.seconds());
                view.motors.get(i).setPower(power);

                data.powers.get(i).add(power);
                data.powerTimes.get(i).add(t.addSplit());
            }

            data.voltages.add(view.voltageSensor.getVoltage());
            data.voltageTimes.add(t.addSplit());

            for (int i = 0; i < view.forwardEncs.size(); i++) {
                Encoder.PositionVelocityPair p = view.forwardEncs.get(i).getPositionAndVelocity();
                data.forwardEncPositions.get(i).add(p.position);
                data.forwardEncVels.get(i).add(p.velocity);
            }
            data.encTimes.add(t.addSplit());
        }

        for (DcMotorEx m : view.motors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.FORWARD_RAMP, data);
    }
}

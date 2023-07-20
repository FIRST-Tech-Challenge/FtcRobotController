package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public final class  ForwardRampLogger extends LinearOpMode {
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

            public final List<Signal> powers = new ArrayList<>();

            public final Signal voltages = new Signal();

            public final List<Signal> forwardEncPositions = new ArrayList<>();
            public final List<Signal> forwardEncVels = new ArrayList<>();
        }

        Data data = new Data();
        for (DcMotorEx m : view.motors) {
            data.powers.add(new Signal());
        }
        for (Encoder e : view.forwardEncs) {
            data.forwardEncPositions.add(new Signal());
            data.forwardEncVels.add(new Signal());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        while (opModeIsActive()) {
            for (int i = 0; i < view.motors.size(); i++) {
                double power = power(t.seconds());
                view.motors.get(i).setPower(power);

                Signal s = data.powers.get(i);
                s.times.add(t.addSplit());
                s.values.add(power);
            }

            data.voltages.values.add(view.voltageSensor.getVoltage());
            data.voltages.times.add(t.addSplit());

            Map<SerialNumber, Double> encTimes = view.resetAndBulkRead(t);

            for (int i = 0; i < view.forwardEncs.size(); i++) {
                recordEncoderData(
                        view.forwardEncs.get(i),
                        encTimes,
                        data.forwardEncPositions.get(i),
                        data.forwardEncVels.get(i)
                );
            }
        }

        for (DcMotorEx m : view.motors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.FORWARD_RAMP, data);
    }
}

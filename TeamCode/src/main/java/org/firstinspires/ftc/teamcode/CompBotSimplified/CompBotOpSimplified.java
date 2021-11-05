package org.firstinspires.ftc.teamcode.CompBotSimplified;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CompBot.CompBotHW;

@TeleOp(name="Viridian Competition Teleop Simplified",group="CompBot")
public class CompBotOpSimplified extends OpMode {
    CompBotHWSimplified r = new CompBotHWSimplified();

    @Override
    public void init() {
        r.init(hardwareMap);
    }

    @Override
    public void loop() {
        double f = clamp(gamepad1.left_stick_y); 
        double s = -1*clamp(gamepad1.left_stick_x);
        double t = clamp(gamepad1.right_stick_x);
        r.m.driveRobotCentric(s,f,t);

    }

    @Override
    public void stop() {
        r.m.stop();
    }

    public double clamp(double in) {
        if(Math.abs(in) < 0.1) {
            return 0;
        }else if(Math.abs(in) > 0.9) {
            return Math.signum(in);
        }
        return in;
    }
}

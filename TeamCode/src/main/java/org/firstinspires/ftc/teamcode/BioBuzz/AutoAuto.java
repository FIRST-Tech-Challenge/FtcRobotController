package org.firstinspires.ftc.teamcode.BioBuzz;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NueralNetworks.Classes.NNFileReader;
import org.firstinspires.ftc.teamcode.NueralNetworks.Classes.Network;
import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;

public class AutoAuto implements Runnable {

    Network nn;
    public static Telemetry tem;
    Follower follow;
    LinearOpMode2026 opMode;
    Status status = Status.SEARCHING;

    public AutoAuto(LinearOpMode2026 op, String network) {
        opMode = op;
        nn = NNFileReader.readC(network);
    }

    @Override
    public void run() {

    }

    enum Status {
        SEARCHING,
        COLLECTING,
        TERMINAL,
        RETURNING,
        SHOOTING
    }
}

package org.firstinspires.ftc.teamcode.BioBuzz;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.NueralNetworks.Classes.BallsTargetingSystem;
import org.firstinspires.ftc.teamcode.NueralNetworks.Classes.NNFileReader;
import org.firstinspires.ftc.teamcode.NueralNetworks.Classes.Network;
import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.Threads.FollowerThread;

import java.util.ArrayList;

public class AutoAuto {

    Network nn;
    public static Telemetry tem;
    LinearOpMode2026 opMode;
    Status status = Status.SEARCHING;
    Status oStatus = Status.SEARCHING;
    int tick = 0;
    boolean active = false;
    FollowerThread fT;
    ArrayList<BallsTargetingSystem.Ball> balls = new ArrayList<>();

    public AutoAuto(LinearOpMode2026 op, String network) {
        opMode = op;
        nn = NNFileReader.readC(network);
    }

    public void init() {
        fT.start();
        active = true;
    }

    public void tick() {
        if(!active) init();
        if(oStatus != status) {
            tick = 0;
            oStatus = status;
        }
        switch(status) {
            case SEARCHING: {
                if(tick == 0) {
                    opMode.lla.setPosition(0.5);
                }

                break;
            }
            case COLLECTING: {

                break;
            }
            case TERMINAL: {

                break;
            }
            case RETURNING: {

                break;
            }
            case SHOOTING: {

                break;
            }
        }
    }

    public void stop() {
        fT.interrupt();
        active = false;
    }

    enum Status {
        SEARCHING,
        COLLECTING,
        TERMINAL,
        RETURNING,
        SHOOTING
    }
}

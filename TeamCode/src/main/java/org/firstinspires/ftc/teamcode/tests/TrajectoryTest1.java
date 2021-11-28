package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectory.markers.Marker;
import org.firstinspires.ftc.teamcode.trajectory.markers.TemporalMarker;

@TeleOp(name = "Trajectory Test 1", group = "Tests")
public class TrajectoryTest1 extends OpMode {
    private Trajectory trajectory;
    private Marker marker;

    @Override
    public void init() {
        marker = new TemporalMarker(10) {
            @Override
            public void run() {

            }
        };
    }

    @Override
    public void loop() {

    }

    private void foo() {

    }
}

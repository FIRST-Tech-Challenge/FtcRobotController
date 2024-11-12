package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotParts.Claw;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;
import org.firstinspires.ftc.teamcode.robotParts.Turn;

@Autonomous(name="LeftAuto", group="Linear OpMode")
@Config
public class LeftAuto extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private final double[] PidConstantsAngle = new double[]{1, 200, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.01, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();

    public static String[][] instructions = {
            //Move forward
            {"Drive", "0", "9000"},

            //Align
            {"Turn", "180"},
            {"Rotate", "0.85"},

            //Clip on
            {"Drive", "0", "0"},
            {"Rotate", "0.9"},
            {"Drive", "0", "0"},
            {"Claw", "Open"},

            //Align to first sample
            {"Rotate", "0.45"},
            {"Turn", "180"},
            {"Drive", "0", "0"}, //Diagonally Left
            {"Drive", "0", "0"}, //Up

            // Pickup
            {"Claw", "Open"},
            {"Rotate", "0.45"},
            {"DLift", ".9"},
            {"Claw", "Close"},
            {"DLift", "0"},

            // Place
            {"Drive", "0", "0"},
            {"Turn", "15"},
            {"Lift", "3000"},
            {"Rotate", "0.9"},
            {"Claw", "Open"},
            {"Rotate", "0.45"},
            {"Lift", "0"},


    };

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        turn.init(hardwareMap);
        claw.init(hardwareMap);

        waitForStart();

        for(String[] instruction : instructions){
            double theta = 0;
            double distance = 0;
            switch (instruction[0]) {
                case "Drive":
                    theta = Double.parseDouble(instruction[1]);
                    distance = Double.parseDouble(instruction[2]);
                    dt.driveToLocation(PidConstantsDistance, theta, distance);
                    break;
                case "Turn":
                    theta = Double.parseDouble(instruction[1]);
                    while(dt.getImu().getAngularOrientation().firstAngle > .05 && opModeIsActive()){
                        dt.fixAngle(PidConstantsAngle, theta);
                        sleep(20);
                    }
                    break;
                case "Lift":
                    distance = Double.parseDouble(instruction[1]);
                    lin.gotoPosition(distance);
                    break;
                case "DLift":
                    theta = Double.parseDouble(instruction[1]);
                    turn.gotoMaxPosition(theta);
                    break;
                case "Rotate":
                    theta = Double.parseDouble(instruction[1]);
                    claw.setRotate(theta);
                    break;
                case "Claw":
                    if (instruction[1].equals("Close")){
                        claw.setClaw(.8);
                    }else{
                        claw.setClaw(.5);
                    }
                    break;
            }
        }
    }
}

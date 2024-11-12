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

@Autonomous(name="RightAuto", group="Linear OpMode")
@Config
public class RightAuto extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private final double[] PidConstantsAngle = new double[]{1, 200, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.01, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();

    public static String[][] instructions = {
            //Figure n
            {"Drive", "0", "9000"},
            {"Drive", "270", "1000"},
            {"Drive", "180", "9000"},
            //Figure n
            {"Drive", "0", "9000"},
            {"Drive", "270", "1000"},
            {"Drive", "180", "9000"},
            //Figure n
            {"Drive", "0", "9000"},
            {"Drive", "270", "1000"},
            {"Drive", "180", "9000"},
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
                    dt.fixAngle(PidConstantsAngle, theta);
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

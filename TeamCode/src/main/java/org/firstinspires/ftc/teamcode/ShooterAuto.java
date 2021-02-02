package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;

@Autonomous(name="ShooterAuto")
public class ShooterAuto extends LinearOpMode {
    private ElapsedTime elapsedTime;

    private FlyWheel flywheel;
    private Hitter hitter;
    public void initRobot() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE));
        hitter = new Hitter(hardwareMap.servo.get("sv"));
        elapsedTime = new ElapsedTime();
    }
    @Override
    public void runOpMode() {
        initRobot();

        elapsedTime.reset();
        while(elapsedTime.seconds() < 10) {
            flywheel.on();
            if(flywheel.isReady()) {
//                telemetry.addData("GO", "it's a go :)");
                hitter.hitFullMotion(0.7);
            } else {
//                telemetry.addData("GO", "no go :(");
            }
        }

        stop();
    }
}

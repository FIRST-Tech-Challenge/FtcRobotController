package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot_utilities.Vals;

@Autonomous(name="ShooterAuto")
public class ShooterAuto extends LinearOpMode {
    private ElapsedTime elapsedTime;

    private FlyWheel flywheel;
    private Hitter hitter;
    public void initRobot() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);
        hitter = new Hitter(hardwareMap.servo.get("sv"));
        elapsedTime = new ElapsedTime();
    }
    @Override
    public void runOpMode() {
        initRobot();
        waitForStart();

        elapsedTime.reset();

        int ticks = 0;

        while(ticks < 3 && elapsedTime.seconds() < 28) {
            telemetry.addData("FlyWheel Filtered Velocity", Vals.flywheel_filtered_speed);
            telemetry.addData("Flywheel Speed", flywheel.flywheel.get());
            telemetry.addData("Flywheel Power", flywheel.flywheel.motor.getPower());
            telemetry.addData("Flywheel Velocity", flywheel.flywheel.getCorrectedVelocity());
            telemetry.addData("Flywheel Position", flywheel.flywheel.getCurrentPosition());
            telemetry.addData("Flywheel P", flywheel.pidFlywheel.getP());
            telemetry.addData("Flywheel I", flywheel.pidFlywheel.getI());
            telemetry.addData("Flywheel D", flywheel.pidFlywheel.getD());
            telemetry.update();
            flywheel.on();
            if(flywheel.isReady()) {
                hitter.hitFullMotion(0.7);
                ticks++;
            }
        }
        elapsedTime.reset();

        while(elapsedTime.seconds() < 1);


        stop();
    }
}

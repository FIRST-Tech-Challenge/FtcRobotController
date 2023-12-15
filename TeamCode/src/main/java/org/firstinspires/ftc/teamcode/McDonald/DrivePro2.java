package org.firstinspires.ftc.teamcode.McDonald;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.shared.MotionHardware;

@TeleOp(name = "Drive Pro 2", group = "TeleOp")
@Config
public class DrivePro2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor
    private Servo leftGripper;
    private Servo rightGripper;
    private Servo launcherServo = null;
    private Servo DroneCoverServo = null;
    private DcMotor armMotor = null;
    MotionHardware robot = new MotionHardware(this);
    @Override
    public void runOpMode () {
        robot.init();
    }
}

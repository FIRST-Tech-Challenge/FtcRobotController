package org.firstinspires.ftc.teamcode.subsystems.Arm;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;
import org.firstinspires.ftc.teamcode.utils.Math.MovingAvrage;
import org.firstinspires.ftc.teamcode.utils.Math.ProfileVelAcc;
import org.firstinspires.ftc.teamcode.utils.Math.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.utils.PID.PIDController;
import org.firstinspires.ftc.teamcode.utils.RunCommand;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
public class armSubsystem {

    public MotorEx arm1;
    public MotorEx arm2;
    public MotorEx pivot1;
    public MotorEx pivot2;
    public Motor.Encoder pivotEncoder;



}

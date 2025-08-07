package org.firstinspires.ftc.teamcode.subsystems.mecanum;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class MecanumCommand {
    private MecanumSubsystem mecanumSubsystem;
    private PinPointOdometrySubsystem odometrySubsystem;
    private LinearOpMode opMode;
    public MecanumCommand(MecanumSubsystem mecanumSubsystem) {
        this.mecanumSubsystem = mecanumSubsystem;
    }
}

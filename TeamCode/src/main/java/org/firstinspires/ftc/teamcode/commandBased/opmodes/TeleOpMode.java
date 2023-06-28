package org.firstinspires.ftc.teamcode.commandBased.opmodes;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.DrivetrainSubsystem;

public class TeleOpMode extends BaseOpMode {

    protected DrivetrainSubsystem drivetrainSS;

    @Override
    public void initialize() {
        drivetrainSS = new DrivetrainSubsystem(hardwareMap);
        super.initialize();

    }


}

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ArmToPosition;
import org.firstinspires.ftc.teamcode.commands.DriveDistanceCmd;
import org.firstinspires.ftc.teamcode.commands.MoveLinearSlide;
import org.firstinspires.ftc.teamcode.subsystems.ArmSub;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSub;

@Autonomous(name = "Auto Test")

public class AutoTest extends AutoCommandOpMode {
    @Override
    public void logic(){
        LinearSlideSub linearSlideSub = new LinearSlideSub(hardwareMap, telemetry);
        ArmToPosition armPos=new ArmToPosition(arm, 30.0, drive, true);
        schedule(new SequentialCommandGroup(
            armPos
            //, new MoveLinearSlide(linearSlideSub,1.0,telemetry)
        ));
    }
}
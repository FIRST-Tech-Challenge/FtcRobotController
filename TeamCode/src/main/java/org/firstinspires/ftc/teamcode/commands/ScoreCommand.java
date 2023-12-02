package org.firstinspires.ftc.teamcode.commands;

import android.content.pm.LabeledIntent;
import android.graphics.LinearGradient;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DoubleCassetSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LinearSlideSubsystem;

public class ScoreCommand extends CommandBase {
    LinearSlideSubsystem m_linearSlide;
    DoubleCassetSubsystem m_casset;

    public ScoreCommand(LinearSlideSubsystem linearSlideSubsystem, DoubleCassetSubsystem casset){
        m_casset = casset;
        m_linearSlide = linearSlideSubsystem;
    }
    public Command score(String level, String whichCasset){
        if(whichCasset == "BOTH"){
            return new SequentialCommandGroup(
                    m_linearSlide.setAndExtendCommand(level)
                            .andThen(new WaitCommand(1000))
                            .andThen(m_casset.depositBothCommand())
                            .andThen(new WaitCommand(1000)));
        } else if(whichCasset == "RIGHT"){

            return new SequentialCommandGroup(
                    m_linearSlide.setAndExtendCommand(level)
                            .andThen(new WaitCommand(1000))
                            .andThen(m_casset.depositRightCommand())
                            .andThen(new WaitCommand(1000)));


        } else if(whichCasset == "LEFT"){

            return new SequentialCommandGroup(
                    m_linearSlide.setAndExtendCommand(level)
                            .andThen(new WaitCommand(1000))
                            .andThen(m_casset.depositLeftPosition())
                            .andThen(new WaitCommand(1000)));


        }


        return new SequentialCommandGroup(
                m_linearSlide.setAndExtendCommand(level)
                        .andThen(new WaitCommand(1000))
                        .andThen(m_casset.depositRightCommand())
        );

    }
}

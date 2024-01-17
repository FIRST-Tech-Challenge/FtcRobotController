package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

    public class changeFoldableCommand extends CommandBase {
        private final DoubleSupplier m_leftYSupplier;
        private final DoubleSupplier m_leftXSupplier;
        private final IntakeSubsystem m_intakeSubsystem;
        public changeFoldableCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier leftYSupplier,
                            DoubleSupplier leftXSupplier) {

            m_intakeSubsystem = intakeSubsystem;
            m_leftXSupplier = leftXSupplier;
            m_leftYSupplier = leftYSupplier;


        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            m_intakeSubsystem.setFoldablePose(
                    m_leftXSupplier.getAsDouble(),
                    m_leftYSupplier.getAsDouble());

        }


    }



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.InstantCommand;
import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.structure.AutonomousCommandOpMode;
import com.technototes.library.subsystem.simple.SimpleServoSubsystem;
@Autonomous(name = "test1")
public class Test extends AutonomousCommandOpMode {
    public SimpleServoSubsystem s;

    @Override
    public void beginInit() {
        s = new SimpleServoSubsystem(new Servo("testServo"));
        CommandScheduler.getRunInstance().schedule(new InstantCommand(() -> s.setPosition(0.5)));
    }
}

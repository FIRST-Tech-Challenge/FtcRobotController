package org.firstinspires.ftc.teamcode.Utilities.Templates;

import com.qualcomm.robotcore.util.ElapsedTime;

public class CommandTemplate {
    ElapsedTime runtime;
    double initialRuntime;
    boolean isFinished;

    //Declare Variables and Utilized Subsystems
    SubsystemTemplate subsystemTemplate;
    double exampleDouble;

    //Constructor intakes subsystem objects and runtime.
    public CommandTemplate(SubsystemTemplate subsystemTemplate, double exampleDouble, ElapsedTime runtime) {
        this.subsystemTemplate = subsystemTemplate;
        this.exampleDouble = exampleDouble;
        this.runtime = runtime;

        runCommand();
    }

    //Command Below
    private void runCommand() {
        //Actions that require no execution time or pause should be listed as such:
        subsystemTemplate.exampleActionThree();

        //Below calls ActionOne, does nothing for 3 seconds, and then calls ActionTwo
        subsystemTemplate.exampleActionOne(exampleDouble);
        initialRuntime = runtime.seconds();
        while (runtime.seconds() - initialRuntime < 3) {}
        subsystemTemplate.exampleActionTwo();
    }
}

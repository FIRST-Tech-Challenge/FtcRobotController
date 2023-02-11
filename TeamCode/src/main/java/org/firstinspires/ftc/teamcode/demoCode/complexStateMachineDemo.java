package org.firstinspires.ftc.teamcode.demoCode;

import static org.firstinspires.ftc.teamcode.demoCode.anotherClass.stateMachine;


public class complexStateMachineDemo {

    static demoSubsystem system;

    static Boolean demoSubsystemCondition;


    public static void main(String[] args){
        system = new demoSubsystem();
        demoSubsystemCondition = system.inPosition();
        anotherClass.driverInput = true;
        while(true){
            stateMachine();
        }
    }
}

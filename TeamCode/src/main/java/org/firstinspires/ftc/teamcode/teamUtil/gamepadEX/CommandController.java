package org.firstinspires.ftc.teamcode.teamUtil.gamepadEX;

public class CommandController {

    public CommandController(){}
    public void conditionalAction(ButtonEX.buttonCondition buttonCondition, ButtonEX.buttonAction buttonAction){
        if(buttonCondition.buttonCondition()){
            buttonAction.buttonAction();
        }
    }

    public void bothConditionAction(ButtonEX.buttonCondition buttonCondition, ButtonEX.buttonAction trueAction, ButtonEX.buttonAction falseAction){
        if(buttonCondition.buttonCondition()){
            trueAction.buttonAction();
        }
        else{
            falseAction.buttonAction();
        }
    }
}

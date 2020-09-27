package org.firstinspires.ftc.teamcode.playmaker;

import org.firstinspires.ftc.teamcode.playmaker.Action;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by djfigs1 on 11/18/16.
 */
public class  ActionSequence {

    private List<Action> actions = new ArrayList<Action>();
    private int currentPosition = 0;

    public ActionSequence(Action[] actionArray) {
        for (Action action : actionArray) {
            addAction(action);
        }
    }

    public void initializeSequence() {
        currentPosition = 0;
    }

    public ActionSequence(List<Action> actions) {
        this.actions = actions;
    }

    public ActionSequence(Action action) {
        addAction(action);
    }

    public ActionSequence() {}

    public void addAction(Action action){
        actions.add(action);
    }

    public Action getCurrentAction() {
        Action action = null;
        if (currentPosition < actions.size()) {
            action = actions.get(currentPosition);
        }
        return action;
    }

    public int numberOfActions() {
        return actions.size();
    }

    public void currentActionComplete() {
        currentPosition++;
    }

}
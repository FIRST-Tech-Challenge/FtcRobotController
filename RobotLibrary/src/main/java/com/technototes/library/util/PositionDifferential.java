package com.technototes.library.util;

import com.technototes.library.odometry.Position;

public class PositionDifferential {
    private Position currPos;
    private Position deltaPos;
    public PositionDifferential(){
        new PositionDifferential(new Position(0, 0, 0));
    }
    public PositionDifferential(Position startingPos){
        currPos = startingPos.compare(startingPos);
        deltaPos = new Position(0, 0, 0);
    }

    public PositionDifferential update(Position newPos){
        deltaPos = currPos.compare(newPos);
        currPos = newPos.compare(newPos);
        return this;
    }

    public Position getCurrValue() {
        return currPos;
    }

    public Position getDeltaV() {
        return  deltaPos;
    }
}

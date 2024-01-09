package org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base;

import java.util.ArrayList;

public class StateMTrajectory
{

    int sequence = 0;

    ArrayList<StateMMovmentPerformer> movements = new ArrayList<>();

    public StateMTrajectory(ArrayList<StateMMovmentPerformer> movements)
    {
        this.movements = movements;
    }

    public static class Builder
    {
        ArrayList<StateMMovmentPerformer> movements = new ArrayList<>();

        public Builder addMovement(StateMMovmentPerformer movementPerformer)
        {
            movements.add(movementPerformer);
            return this;
        }

        public StateMTrajectory build()
        {
            return new StateMTrajectory(movements);
        }
    }

    public void reset()
     {
       sequence = 0;
         for (StateMMovmentPerformer Movment:movements) {
             Movment.reset();
         }
     }

    public void followInteration() {
        if (sequence < movements.size()) {
            if (movements.get(sequence).run()) {
                sequence += 1;

            }

        }
    }
}

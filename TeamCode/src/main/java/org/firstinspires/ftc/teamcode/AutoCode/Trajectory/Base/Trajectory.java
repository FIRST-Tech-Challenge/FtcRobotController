package org.firstinspires.ftc.teamcode.AutoCode.Trajectory.Base;

import java.util.ArrayList;

public class Trajectory
{
    ArrayList<MovementPerformer> movements = new ArrayList<>();

    public Trajectory(ArrayList<MovementPerformer> movements)
    {
        this.movements = movements;
    }

    public static class Builder
    {
        ArrayList<MovementPerformer> movements = new ArrayList<>();

        public Builder addMovement(MovementPerformer movementPerformer)
        {
            movements.add(movementPerformer);
            return this;
        }

        public Trajectory build()
        {
            return new Trajectory(movements);
        }
    }

    public void follow()
    {
        for(MovementPerformer movementPerformer : movements)
        {
            System.out.println("Starting block");
            movementPerformer.run();
        }
    }
}

package team12395;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class FinalNode implements Node{

    private Vector2d pose;
    private ArrayList<PathNode> pathList;
    private int pathIndex;
    private Double nextVector;
    private Double lastVector;
    public boolean hasNullNeighbours;
    public FinalNode(PathNode node, Double nextVector, Double lastVector){
        this.pose = node.getRealCoords();
        this.nextVector = nextVector;
        this.lastVector = lastVector;
        hasNullNeighbours = node.getNeighbours().contains(null);
    }

    public Vector2d getRealCoords(){
        return pose;
    }

    public Double getNextVector(){
        return nextVector;
    }

    public Double getLastVector(){
        return lastVector;
    }


}

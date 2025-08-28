package team12395;

import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;

public class PathNode implements Node{

    private Vector2d pose;
    private boolean visited = false;
    private PathNode parent = null;
    private ArrayList<ArrayList<PathNode>> masterList;
    private int xList;
    private int yList;
    private Vector2d listPos;
    public PathNode(Vector2d pose, ArrayList<ArrayList<PathNode>> masterList, int xLoc, int yLoc){
        this.pose = pose;
        this.masterList = masterList;
        listPos = new Vector2d(xLoc, yLoc);
        xList = xLoc; yList = yLoc;
    }

    public void visit(){
        visited = true;
    }
    public void unvisit(){
        visited = false;
    }

    public void setParent(PathNode pNode){
        parent = pNode;
    }
    public PathNode getParent(){
        return parent;
    }

    public boolean isVisited(){
        return visited;
    }

    public Vector2d getRealCoords(){
        return pose;
    }

    public  ArrayList<PathNode> getNeighbours (){
        ArrayList<PathNode> neighbours = new ArrayList<>();
        for(int i=xList-1; i<= xList+1; i++) {
            for (int j = yList + 1; j >= yList - 1; j--) {
                if (i < 0 || i > masterList.size() - 1 || j < 0 || j > masterList.get(i).size() - 1) {
                    neighbours.add(null);
                } else {
                    neighbours.add(masterList.get(i).get(j));
                }
            }
        }
        neighbours.remove(this);

        return neighbours;
    }

    public Vector2d getListPos(){
        return listPos;
    }

    @Override
    public String toString(){
        return pose.toString();
    }

}

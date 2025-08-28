package team12395;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.lang.Math;
import java.util.ArrayList;
import java.util.Objects;

public class MasterDrive {
    RoadRunnerBotEntity m;
    private DraftPath drafter;

    private Vector2d start;
    private Vector2d end;


    /**
     * ONLY USE THIS CONSTRUCTOR FOR MEEPMEEP TESTING
     */
    public MasterDrive(RoadRunnerBotEntity m){
        this.m = m;
    }

    public void setEnvironment(double sparsityIN, double botRadiusIN, double driveTol){
        drafter = new DraftPath(sparsityIN, botRadiusIN+driveTol);
    }


    public Action driveToMeepMeep(Pose2d start, Pose2d end){
        this.start = start.position; this.end = end.position;
        ArrayList<FinalNode> fPath = vectorSetup(drafter.createPath(start, end));
        //siftPath(fPath);
        //pickPath(fPath);
        return actionConverter(fPath);
    }

    private void siftPath(ArrayList<FinalNode> path){
        // woah
        path.removeIf(node -> !node.hasNullNeighbours && (node.getRealCoords().equals(start) || node.getRealCoords().equals(end)));
    }

    private void pickPath(ArrayList<FinalNode> path){
        for (int i=0; i<path.size(); i++){
            if (Objects.equals(path.get(i).getNextVector(), path.get(i).getLastVector()) &&
                    !(path.get(i).getRealCoords().equals(start) || path.get(i).getRealCoords().equals(end))){
                path.remove(i);
                i--;
            }
        }
    }

    private ArrayList<FinalNode> vectorSetup(ArrayList<PathNode> path){
        ArrayList<FinalNode> arr = new ArrayList<>();


        for (int i=0; i<path.size(); i++){
            if (path.get(i).getRealCoords().equals(start)){
                arr.add(new FinalNode(path.get(i), // 0 node
                        getVector(path.get(i).getRealCoords(), path.get(i+1).getRealCoords()),
                        null));
            } else if (path.get(i).getRealCoords().equals(end)){
                arr.add(new FinalNode(path.get(i), // last node
                        null,
                        getVector(path.get(i-1).getRealCoords(), path.get(i).getRealCoords())));
            } else {
                arr.add(new FinalNode(path.get(i),
                        getVector(path.get(i).getRealCoords(), path.get(i + 1).getRealCoords()),
                        getVector(path.get(i - 1).getRealCoords(), path.get(i).getRealCoords())));
            }
        }
        return arr;

    }

    private Double getVector(Vector2d start, Vector2d end){
        ArrayList<Double> slope = new ArrayList<>();
        slope.add(end.y - start.y);
        slope.add(end.x - start.x);

        if (slope.get(1) != 0){

            double rad = Math.atan(slope.get(0)/slope.get(1));
            if (slope.get(1) < 0){
                rad += Math.PI;
            }
            return rad;

        } else {
            return Math.PI/2 * slope.get(0)/Math.abs(slope.get(0));
        }
    }

    private Action actionConverter(ArrayList<FinalNode> fPath){
        TrajectoryActionBuilder d = m.getDrive().actionBuilder(new Pose2d(fPath.get(0).getRealCoords(), 0));
        int i=0;

        for (FinalNode f : fPath){
            try {
                d= d.setTangent(f.getNextVector());
                d= d.lineToX(f.getRealCoords().x);
            }
            catch (Exception e){
                if (f.getNextVector() != null) {
                    d= d.setTangent(f.getNextVector());
                    d= d.lineToY(f.getRealCoords().y);
                }
            }
            //d= d.waitSeconds(0);
            i++;
            System.out.println(f.getRealCoords().toString());
        }
        System.out.println(i + " movements made");
        return d.build();
    }



}

package team12395;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;

class DraftPath {
    private ArrayList<ArrayList<PathNode>> gridIN = new ArrayList<>();

    private final int FIELD_WIDTH = 24*6;

    // here in case some new map idk
    private final int FIELD_HEIGHT = 24*6;
    private double startX;
    private double sparsity;

    private final Vector2d[] LOWER_BOUND_DMZ = {new Vector2d(-25, -25), new Vector2d(25, -20)};
    private final Vector2d[] UPPER_BOUND_DMZ = {new Vector2d(-25, 20), new Vector2d(25, 25)};
    private final Vector2d[] NML_DMZ = {new Vector2d(-16, -25), new Vector2d(16, 25)};


    protected DraftPath(double sparsity, double botRadius){
        /*LOWER_BOUND_DMZ[0] = new Vector2d(LOWER_BOUND_DMZ[0].x - botRadius, LOWER_BOUND_DMZ[0].y - botRadius);
        LOWER_BOUND_DMZ[1] = new Vector2d(LOWER_BOUND_DMZ[1].x + botRadius, LOWER_BOUND_DMZ[1].y + botRadius);

        UPPER_BOUND_DMZ[0] = new Vector2d(UPPER_BOUND_DMZ[0].x - botRadius, UPPER_BOUND_DMZ[0].y - botRadius);
        UPPER_BOUND_DMZ[1] = new Vector2d(UPPER_BOUND_DMZ[1].x + botRadius, UPPER_BOUND_DMZ[1].y + botRadius);

         */

        startX = -0.5*FIELD_WIDTH + botRadius;
        double startY = -0.5*FIELD_HEIGHT + botRadius;
        this.sparsity = sparsity;
        double distance = FIELD_WIDTH - 2*botRadius;

        int rowsColumns = (int) (distance/sparsity);

        // reference node, does not get added
        final PathNode origin = new PathNode(new Vector2d(startX, startY), gridIN, 0, 0);

        // for the length of the row, add a new column list to each spot
        for(int r = 0; r < rowsColumns; r++){
            ArrayList<PathNode> newestColumn = new ArrayList<>();

            // for the height of each column, add a new PathNode to the column list
            for(int c = 0; c < rowsColumns; c++){
                Vector2d tP = new Vector2d(origin.getRealCoords().x + r*sparsity, origin.getRealCoords().y + c*sparsity);
                if (!pointFitsIn(LOWER_BOUND_DMZ, tP) && !pointFitsIn(UPPER_BOUND_DMZ, tP) && !pointFitsIn(NML_DMZ, tP)) {
                    newestColumn.add(new PathNode(
                            // scale rows & columns to sparsity level for real world
                            tP,
                            // pass in masterList reference
                            gridIN,
                            // list location
                            r, c));
                } else {
                    newestColumn.add(null);
                }
            }
            // add column
            gridIN.add(newestColumn);
        }

        for (int i=0; i<gridIN.size(); i++){
            if (gridIN.get(i).get(0) == null){
                gridIN.remove(i);
                i--;
                System.out.println("expected rows: " + rowsColumns + " deleted row " + i);
            }
        }
    }

    private boolean pointFitsIn(Vector2d[] dmzArr, Vector2d pose){
        // btm left - top right
        return (pose.x >= dmzArr[0].x && pose.x <= dmzArr[1].x) && (pose.y >= dmzArr[0].y && pose.y <= dmzArr[1].y);
    }

    // BFS
    private void adventure(PathNode start, PathNode end) throws BootstrapMethodError {
        // create queue
        Queue<PathNode> q = new LinkedList<>();
        // start with start node (duh)
        q.add(start);
        start.visit();
        PathNode node = start;

        // search until queue is empty (done) or you found the end node
        while(!q.isEmpty() && !node.getListPos().equals(end.getListPos())){
            // remove current item from queue & save it temporarily
            node = q.remove();
            // get current item's neighbours
            ArrayList<PathNode> neighbours = node.getNeighbours();

            // for each neighbour...
            for(PathNode tNode : neighbours){
                if (tNode != null) {
                    // add interest to neighbour if not visited
                    if (!tNode.isVisited()) {
                        // add this neighbour
                        q.add(tNode);
                        // mark down the parent of this neighbour (item you are on)
                        tNode.setParent(node);
                        // mark neighbour as visited
                        tNode.visit();
                    }
                }
            }
            if (q.isEmpty()){
                throw new NullPointerException();
            }
        }
        if (node.getListPos().equals(end.getListPos())) {
            end.setParent(node.getParent());
            System.out.println(end.getRealCoords().toString() + " -> " + node.getRealCoords().toString());
        } else if (q.isEmpty()){
            throw new OutOfMemoryError();
        }

    }

    private ArrayList<PathNode> retrace(PathNode end){
        // initialize return array
        ArrayList<PathNode> path = new ArrayList<>();
        path.add(end);

        // set first ancestor as the end parent (final descendant [O_O neh] )
        PathNode ancestor = end.getParent();
        // while the ancestor has an ancestor...
        while(ancestor != null){
            // add this ancestor to the return array
            path.add(ancestor);
            // set the new ancestor to the current ancestor's ancestor (get the ancestor of the current node)
            ancestor = ancestor.getParent();
            // repeat
        }

        // since we started from the end node, reverse list to go from start-end.
        Collections.reverse(path);

        // return path
        return path;
    }

    // master method
    protected ArrayList<PathNode> createPath(Pose2d start, Pose2d finish){

        // translate inciting Poses to PathNodes
        PathNode startActorNode = new PathNode(start.position, gridIN, (int) closestListCoords(start).x, (int) closestListCoords(start).y);
        PathNode endActorNode = new PathNode(finish.position, gridIN, (int) closestListCoords(finish).x, (int) closestListCoords(finish).y);
        /* notice these aren't added to the grid, only reference it as their masterList & that their list coords are approximates.
           this is because the poses passed will likely not be directly on the grid but still need to be referenced to.
           to solve this we find the nearest node to them on the grid and use that node's identity for the algorithm to process.
           however, the original position is not changed; their quality is kept for later reference (the algorithm is not influenced
           by real-world Poses)

           essentially, we are downscaling the inciting Poses for our grid but keeping their real-world position data.

         */
        int j=0;
        int i=0;
        while (i<=8){
            if (startActorNode.getNeighbours().get(i) == (null)){
                j++;
            }
            i++;
        }
        if (j >= i){
            throw new NullPointerException();
        }


        // run BFS
        adventure(startActorNode, endActorNode);

        // return solved path
        ArrayList<PathNode> retArr = retrace(endActorNode);
        for(int k=0; k<retArr.size(); k++){
            System.out.println(k+": "+retArr.get(k).toString());
        }
        return retArr;
    }

    private Vector2d closestListCoords(Pose2d pose){
        return new Vector2d((int) ((pose.position.x - startX)/sparsity),
                (int) ((pose.position.y - startX)/sparsity));
    }
}

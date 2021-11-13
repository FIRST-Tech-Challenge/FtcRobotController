package org.firstinspires.ftc.teamcode.Util.CoordinateSystem.PathFinder;
// Java Program to Implement Dijkstra's Algorithm
// Using Priority Queue

// Importing utility classes

import java.util.*;

// Main class DPQ
public class DjikstraPathFinder {

    List<List<NodeDjikstra>> adj;
    // Member variables of this class
    private int dist[];
    private Set<Integer> settled;
    private PriorityQueue<NodeDjikstra> pq;
    // Number of vertices
    private int V;

    // Constructor of this class
    public DjikstraPathFinder(int V) {
        // This keyword refers to current object itself
        this.V = V;
        dist = new int[V];
        settled = new HashSet<Integer>();
        pq = new PriorityQueue<NodeDjikstra>(V, new NodeDjikstra());
    }

    // Dijkstra's Algorithm
    public void getShortestPath(List<List<NodeDjikstra>> adj, int src) {
        this.adj = adj;

        for (int i = 0; i < V; i++)
            dist[i] = Integer.MAX_VALUE;

        // Add source node to the priority queue
        pq.add(new NodeDjikstra(src, 0));

        // Distance to the source is 0
        dist[src] = 0;

        while (settled.size() != V) {

            // Terminating ondition check when
            // the priority queue is empty, return
            if (pq.isEmpty())
                return;

            // Removing the minimum distance node
            // from the priority queue
            int u = pq.remove().node;

            // Adding the node whose distance is
            // finalized
            if (settled.contains(u))

                // Continue keyword skips exwcution for
                // following check
                continue;

            // We don't have to call e_Neighbors(u)
            // if u is already present in the settled set.
            settled.add(u);

            e_Neighbours(u);
        }
    }

    // Method 2
    // To process all the neighbours
    // of the passed node
    private void e_Neighbours(int u) {

        int edgeDistance = -1;
        int newDistance = -1;

        // All the neighbors of v
        for (int i = 0; i < adj.get(u).size(); i++) {
            NodeDjikstra v = adj.get(u).get(i);

            // If current node hasn't already been processed
            if (!settled.contains(v.node)) {
                edgeDistance = v.cost;
                newDistance = dist[u] + edgeDistance;

                // If new distance is cheaper in cost
                if (newDistance < dist[v.node])
                    dist[v.node] = newDistance;

                // Add the current node to the queue
                pq.add(new NodeDjikstra(v.node, dist[v.node]));
            }
        }
    }

    // Main driver method
    public void test() {

        int V = 5;
        int source = 0;

        // Adjacency list representation of the
        // connected edges by declaring List class object
        // Declaring object of type List<Node>
        List<List<NodeDjikstra>> adj
                = new ArrayList<List<NodeDjikstra>>();

        // Initialize list for every node
        for (int i = 0; i < V; i++) {
            List<NodeDjikstra> item = new ArrayList<NodeDjikstra>();
            adj.add(item);
        }

        // Inputs for the GFG(dpq) graph
        adj.get(0).add(new NodeDjikstra(1, 9));
        adj.get(0).add(new NodeDjikstra(2, 6));
        adj.get(0).add(new NodeDjikstra(3, 5));
        adj.get(0).add(new NodeDjikstra(4, 3));

        adj.get(2).add(new NodeDjikstra(1, 2));
        adj.get(2).add(new NodeDjikstra(3, 4));

        // Calculating the single source shortest path
        DjikstraPathFinder dpq = new DjikstraPathFinder(V);
        dpq.getShortestPath(adj, source);

        // Printing the shortest path to all the nodes
        // from the source node
        System.out.println("The shorted path from node :");

        for (int i = 0; i < dpq.dist.length; i++)
            System.out.println(source + " to " + i + " is "
                    + dpq.dist[i]);
    }
}

// Class 2
// Helper class implementing Comparator interface
// Representing a node in the graph
class NodeDjikstra implements Comparator<NodeDjikstra> {

    // Member variables of this class
    public int node;
    public int cost;

    // Constructors of this class

    // Constructor 1
    public NodeDjikstra() {
    }

    // Constructor 2
    public NodeDjikstra(int node, int cost) {

        // This keyword refers to current instance itself
        this.node = node;
        this.cost = cost;
    }

    @Override
    public int compare(NodeDjikstra node1, NodeDjikstra node2) {

        if (node1.cost < node2.cost)
            return -1;

        if (node1.cost > node2.cost)
            return 1;

        return 0;
    }
}
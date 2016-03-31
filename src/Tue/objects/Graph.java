package Tue.objects;

import java.util.ArrayList;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Graph
{

    private ArrayList<Edge> edges = new ArrayList<Edge>();
    private ArrayList<Node> nodes = new ArrayList<Node>();

    public Graph(ArrayList<Node> nodes, ArrayList<Edge> edges )
    {
        this.edges = edges;
        this.nodes = nodes;
    }

    public double[] BFS( Node source )
    {
        ArrayList<Node> queue = new ArrayList<Node>();
        double[] dist = new double[nodes.size()];
        Node[] prev = new Node[nodes.size()];

        for( Node node : nodes )
        {
            dist[node.getIndex()] = Integer.MAX_VALUE;
            queue.add(node);
        }

        dist[source.getIndex()] = 0;

        while(!queue.isEmpty())
        {
            double current = 999;
            int indexCurrent = -1;
            for( Node node : queue )
            {
                if(dist[node.getIndex()] < current)
                {
                    current = dist[node.getIndex()];
                    indexCurrent = node.getIndex();
                }
            }

            if( indexCurrent == -1 )
            {
                for( Node node : nodes )
                {
                    if( dist[node.getIndex()] == Integer.MAX_VALUE )
                    {
                        dist[node.getIndex()] = -1;
                    }
                }
                break;
            }

            Node u = nodes.get(indexCurrent);
            queue.remove(u);

            for( Node node : queue )
            {
                for( Edge edge : edges )
                {
                    if( edge.getSource() == node && edge.getDest() == u )
                    {
                        double alt = dist[indexCurrent] + edge.getWeight();
                        if( alt < dist[node.getIndex()])
                        {
                            dist[node.getIndex()] = alt;
                            prev[node.getIndex()] = node;
                        }
                    }
                    if( edge.getDest() == node && edge.getSource() == u )
                    {
                        double alt = dist[indexCurrent] + edge.getWeight();
                        if( alt < dist[node.getIndex()])
                        {
                            dist[node.getIndex()] = alt;
                            prev[node.getIndex()] = node;
                        }
                    }
                }
            }
        }

        return dist;
    }
}

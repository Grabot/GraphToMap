package Tue.GraphGenerator;

import java.io.*;
import java.util.ArrayList;
import java.util.Random;

/**
 * Created by s138362 on 4-4-2016.
 */
public class RandomGraph
{
    private int clusteramount = -1;
    private int edgeamount = -1;
    private int nodeamount = -1;

    private Random rand = new Random();

    public RandomGraph()
    {
        this.nodeamount = rand.nextInt(300);
        this.clusteramount = rand.nextInt(20);
        this.edgeamount = rand.nextInt(1000);
    }

    public RandomGraph( int nodeamount, int clusteramount, int edgeamount )
    {
        this.nodeamount = nodeamount;
        this.clusteramount = clusteramount;
        this.edgeamount = edgeamount;
    }

    public void createGraphSimple()
    {
        PrintWriter writer = null;
        try {
            writer = new PrintWriter("datasets/random4.gv", "UTF-8");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }

        if (writer != null)
        {
            writer.println("graph {");
        }

        String graph = "graph {\n";
        int index = 0;

        //create a random amount of nodes with a random amount of clusters randomly distributed
        for( int i = 0; i < nodeamount; i++ )
        {
            if (writer != null)
            {
                writer.println("\"" + index + "\"" + " [cluster=\"" + rand.nextInt(clusteramount) + "\", label=\"" + (index+1) + "\"];");
            }
            graph = (graph + ("\"" + index + "\"" + " [cluster=\"" + rand.nextInt(clusteramount) + "\", label=\"" + (index+1) + "\"];\n"));
            index++;
        }

        //create a random amount of edges with a random nodes connecting each other
        for( int i = 0; i < edgeamount; i++ )
        {
            int source = rand.nextInt(nodeamount);
            int dest = rand.nextInt(nodeamount);

            if (writer != null)
            {
                writer.println("\"" + source + "\" -- \"" + dest + "\";");
            }
            graph = (graph + ("\"" + source + "\" -- \"" + dest + "\";\n"));
        }

        if (writer != null)
        {
            writer.println("}");
            writer.close();
        }
        graph = (graph + "}");
        System.out.println("" + graph );

    }

    public static void main( String[] args )
    {
        RandomGraph run = new RandomGraph(40, 14, 200);
        run.createGraphSimple();
    }

}

package Tue;

import Tue.load.Display;
import Tue.objects.Edge;
import Tue.objects.Graph;
import Tue.objects.Node;
import Tue.parser.DotParser;
import Tue.parser.DotScanner;

import java.awt.EventQueue;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class Main {

    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    public static void main(String[] args)
    {

        Main main = new Main();
        main.execute();

    }

    private void execute()
    {
        final DotParser parser = parserInput();

        nodes = parser.getNodes();
        edges = parser.getEdges();

        Graph g = new Graph(nodes, edges);


        float[][] pairD = new float[nodes.size()][nodes.size()];

        for( int i = 0; i < nodes.size(); i++ )
        {
            pairD[i] = g.BFS(nodes.get(i));
        }

        for( int i = 0; i < nodes.size(); i++ )
        {
            for( int j = 0; j < nodes.size(); j++ )
            {
                System.out.print( pairD[i][j] + " ");
            }
            System.out.println("");
        }

        EventQueue.invokeLater(new Runnable()
        {
            @Override
            public void run() {
                new Display(parser).create();
            }
        });
    }


    private DotParser parserInput()
    {
        DotScanner scanner = null;
        try {
            scanner = new DotScanner(new FileReader("datasets/sample.gv"));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        DotParser parser = new DotParser(scanner, nodes, edges);
        try {
            parser.graph();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return parser;
    }

}
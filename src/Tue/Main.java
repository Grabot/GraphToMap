package Tue;

import Tue.display.Display;
import Tue.objects.Edge;
import Tue.objects.Node;
import Tue.parser.DotParser;
import Tue.parser.DotScanner;

import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class Main {

    public ArrayList<Node> nodes = new ArrayList<Node>();
    public ArrayList<Edge> edges = new ArrayList<Edge>();

    public static void main(String[] args) throws IOException {
        Main main = new Main();
        main.Run();
    }

    private void Run() throws IOException
    {
        DotScanner scanner = new DotScanner(new FileReader("datasets/universities.gv"));

        DotParser parser = new DotParser(scanner, nodes, edges);
        parser.graph();

        nodes = parser.getNodes();
        edges = parser.getEdges();

        for( int i = 0; i < nodes.size(); i++ )
        {
            System.out.println("node: " + nodes.get(i).getIndex() + " with label: " + nodes.get(i).getLabel() + " it has cluster: " + nodes.get(i).getCluster() );
        }

        for( int i = 0; i < edges.size(); i++ )
        {
            System.out.println("edge: " + i + " edge from " + edges.get(i).getFrom().getName() + " to " + edges.get(i).getDest().getName() );
        }

        Display display = new Display();
        display.run();
    }
}

package Tue;

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
        System.out.println("Hello World");

        DotScanner scanner = new DotScanner(new FileReader("datasets/universities.gv"));
        Token t;

        DotParser parser = new DotParser(scanner, nodes, edges);
        parser.graph();

        nodes = parser.getNodes();
        edges = parser.getEdges();

        for( int i = 0; i < nodes.size(); i++ )
        {
            System.out.println("node: " + i + " with label: " + nodes.get(i).getLabel() );
        }

        for( int i = 0; i < edges.size(); i++ )
        {
            System.out.println("edge: " + i + " edge from " + edges.get(i).getFrom().getName() + " to " + edges.get(i).getDest().getName() );
        }

        /*
        try {
            t = scanner.nextToken();

            while ( t.type != Token.EOF_TYPE )
            {
                System.out.println(t.text);
                t = scanner.nextToken();
            }
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        */
    }
}

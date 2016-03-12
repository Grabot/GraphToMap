package Tue;

import java.util.ArrayList;
import java.util.Scanner;

/**
 * Created by s138362 on 12-3-2016.
 */
public class DotParser
{

    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();
    private Scanner s;

    public DotParser(Scanner s)
    {
        this.s = s;
        graph();
    }

    private void graph()
    {
        String line = s.nextLine();

        if(line.contains("graph"))
        {
            getNextStatement();
        }
        else
        {
            //error, wrong format
            System.out.println("wrong format");
        }
    }

    private void getNextStatement()
    {
        String line = s.nextLine();

        if( line.contains("}"))
        {
            //done
        }
        else if( line.contains("--") || line.contains("->"))
        {
            //edge
            getEdge(line);
        }
        else if( line.contains(";"))
        {
            //node
            getNode(line);
        }
        else
        {
            //either empty row or "{" mark. Get new line
            getNextStatement();
        }
    }

    private void getEdge(String line)
    {

    }

    private void getNode(String line)
    {
        //in case of attributes split it with "[", otherwise no problem
        String[] temp = line.split("\\[");
        String node = temp[0];
        node = node.replace("\"", "" );
        node = node.replace(";", "" );
        if( temp.length > 1 )
        {
            //it has attributes, get attributes
            getNodeAttributes(temp[1]);
        }
        System.out.println("node: "  + node );

        //add node to system
        Node n = new Node();

        getNextStatement();
    }

    private void getNodeAttributes(String att)
    {

    }

}

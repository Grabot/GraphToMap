package Tue;

import java.io.IOException;
import java.io.StreamTokenizer;
import java.util.ArrayList;
import java.util.Scanner;
import java.util.Vector;

/**
 * Created by s138362 on 12-3-2016.
 */
public class DotParser
{

    //logger

    DotScanner scanner;
    public Token t;
    private int index = -1;
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    public DotParser(DotScanner s,ArrayList m_nodes,ArrayList m_edges)
    {
        scanner = s;
        this.nodes = m_nodes;
        this.edges = m_edges;
    }

    public void next() throws IOException
    {
        t = scanner.nextToken();
    }

    /*
     * @description :: recursively descend the syntax and process each part.
     */
    public void graph() throws IOException{

        next();
        if(t.text.equalsIgnoreCase("graph")){
            next();
            if(t.type == '{'){
                //System.out.println("Entering stmt_list expecting { got: " + t.text);
                stmt_list();
            }else if(t.type == '}'){
                System.out.println("Congrats");
            }
        }else{
            throw new IOException("Structure incorrect at token: " + t.text);
        }

    }


    public void stmt_list() throws IOException{
        next();

        if(t.type == '}' || t.type == StreamTokenizer.TT_EOF){
            return;
        }
        else{
            //System.out.println("Entering statement:" + t.text);
            stmt();
            stmt_list();
        }
    }

    public void nodeStmt() throws IOException{
        next();
        Node temp = nodes.get(index);

        if(t.type == ']' || t.type == StreamTokenizer.TT_EOF)
        {
            return;
        }
        else if(t.type == StreamTokenizer.TT_WORD)
        {
            if(t.text.equalsIgnoreCase("label")){
                next();
                if(t.text.equalsIgnoreCase("=")){
                    next();
                    if(t.type == StreamTokenizer.TT_WORD || t.type == '"'){
                        //System.out.println("Adding label " + t.text + " to node: " + index );
                        temp.addLabel(t.text);
                    }else{
                        System.out.println("There was an error unable to deal with: " + t.text + " at line:" + t.line);
                        scanner.pushback();
                    }
                }
                else
                {
                    System.out.println("There was an error unable to deal with: " + t.text + " at line:" + t.line);
                    scanner.pushback();
                }
            }
            else
            {
                //System.out.println("There was an error unable to deal with: " + t.text + " at line:" + t.line);
                //scanner.pushback();
            }
        }
        nodeStmt();
    }


    public void stmt() throws IOException{
        //next();


        if(t.text.equalsIgnoreCase("graph") || t.text.equalsIgnoreCase("node") || t.text.equalsIgnoreCase("edge"))
        {
            //do nothing
        }
        else
        {

            boolean exists = false;
            Node origin = null;
            for( int i = 0; i < nodes.size(); i++ )
            {
                if( nodes.get(i).getName().equals(t.text))
                {
                    //we expect an edge now.
                    origin = nodes.get(i);
                    exists = true;
                }
            }

            if( !exists )
            {
                node();
            }

            //int nodeindex = nodes.indexOf(new Node(t.text));
            //System.out.println("Created node:" + t.text + " at index:" + nodeindex);
            next();

            //System.out.println(t.text);
            if(t.type == '[' && !exists){
                //System.out.println("Entering node statement expect [ got " + t.text);
                nodeStmt();
            }
            else if(t.type == '-' && exists)
            {
                //System.out.println("Entering node statement expect - got " + t.text);
                edgeStmt( origin );
            }

        }


    }

    public void edgeStmt( Node origin ) throws IOException{
        next();

        Node dest = null;

        //System.out.println("Edge statement expecting: - got:" + t.text);
        Edge temp = null;
        if(t.type == ']' || t.type == StreamTokenizer.TT_EOF){
            return;
        }else if(t.type == '-'){
            next();

            for( int i = 0; i < nodes.size(); i++ )
            {
                if( nodes.get(i).getName().equals(t.text))
                {
                    dest = nodes.get(i);
                }
            }

            Edge edge = new Edge( origin, dest );
            edges.add( edge );

            next();

            if(t.type == '[' && temp!=null)
            {
                System.out.println("Entering edge attribute:" + t.text);
                //edgeAttr(temp);
            }
            else
            {
                //System.out.println("Did not find a label for this edge");
                scanner.pushback();
            }
        }
        else
        {
            System.out.println("There was an error at line: " + t.line);

            //return;
        }

    }

    public void node() throws IOException{
        //next();
        if(nodes !=null) {
            index++;
            //System.out.println("added node " + t.text);
            nodes.add(new Node(t.text, index));
        }
    }

    public ArrayList<Node> getNodes()
    {
        return nodes;
    }

    public ArrayList<Edge> getEdges()
    {
        return edges;
    }
}

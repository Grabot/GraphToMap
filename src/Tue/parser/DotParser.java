package Tue.parser;

import Tue.load.Forces.Force;
import Tue.objects.Edge;
import Tue.objects.Node;

import java.io.IOException;
import java.io.StreamTokenizer;
import java.util.ArrayList;

/**
 * Created by s138362 on 15-3-2016.
 */
public class DotParser {

    public Token t;
    DotScanner scanner;
    private int index = -1;
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();
    private Force forces;

    public DotParser(DotScanner s, ArrayList<Node> m_nodes, ArrayList<Edge> m_edges, Force forces) {
        scanner = s;
        this.nodes = m_nodes;
        this.edges = m_edges;
        this.forces = forces;
    }

    public void next() throws IOException {
        t = scanner.nextToken();
    }


    public void graph() throws IOException {

        next();
        if (t.text.equalsIgnoreCase("graph")) {
            next();
            if (t.type == '{') {
                //System.out.println("Entering stmt_list expecting { got: " + t.text);
                stmt_list();
            } else if (t.type == '}') {
                System.out.println("no data");
            }
        } else {
            throw new IOException("Structure incorrect at token: " + t.text);
        }
    }

    public void stmt_list() throws IOException {
        next();

        if (!((t.type == '}' || t.type == StreamTokenizer.TT_EOF))) {
            //ignore "graph", "node" and "edge" statements. we don't handle this data
            if (!(t.text.equalsIgnoreCase("graph") || t.text.equalsIgnoreCase("node") || t.text.equalsIgnoreCase("edge"))) {
                boolean exists = false;
                Node origin = null;

                for (Node node : nodes) {
                    if (node.getName().equals(t.text)) {
                        //we expect an edge now.
                        origin = node;
                        exists = true;
                    }
                }

                if (!exists) {
                    node();
                }

                //int nodeindex = nodes.indexOf(new Node(t.text));
                //System.out.println("Created node:" + t.text + " at index:" + nodeindex);
                next();

                //System.out.println(t.text);
                if (t.type == '[' && !exists) {
                    //get a new node statement
                    nodeStmt();
                } else if (t.type == '-' && exists) {
                    //get a new edge statement
                    edgeStmt(origin);
                }

            }

            stmt_list();
        }
    }

    public void nodeStmt() throws IOException {
        next();
        Node temp = nodes.get(index);

        if (t.type == ']' || t.type == StreamTokenizer.TT_EOF) {
            return;
        } else if (t.type == StreamTokenizer.TT_WORD) {
            if (t.text.equalsIgnoreCase("label")) {
                next();
                if (t.text.equalsIgnoreCase("=")) {
                    next();
                    if (t.type == StreamTokenizer.TT_WORD || t.type == '"') {
                        //System.out.println("Adding label " + t.text + " to node: " + index );
                        temp.addLabel(t.text);
                    } else {
                        System.out.println("There was an error unable to deal with: " + t.text + " at line:" + t.line);
                        scanner.pushback();
                    }
                } else {
                    System.out.println("There was an error unable to deal with: " + t.text + " at line:" + t.line);
                    scanner.pushback();
                }
            } else if (t.text.equalsIgnoreCase("cluster") || t.text.equalsIgnoreCase("clusters") || t.text.equalsIgnoreCase("group")) {
                next();
                if (t.text.equalsIgnoreCase("=")) {
                    next();
                    if (t.type == StreamTokenizer.TT_WORD || t.type == '"') {
                        //System.out.println("Adding cluster " + t.text + " to node: " + index );
                        temp.addCluster(t.text);
                    } else {
                        System.out.println("There was an error unable to deal with: " + t.text + " at line:" + t.line);
                        scanner.pushback();
                    }
                }
            }
            else if( t.text.equalsIgnoreCase("fontsize"))
            {
                next();
                if (t.text.equalsIgnoreCase("=")) {
                    next();
                    if (t.type == StreamTokenizer.TT_WORD || t.type == '"') {
                        temp.addWeight(Double.parseDouble(t.text));
                    }else {
                        System.out.println("There was an error unable to deal with: " + t.text + " at line:" + t.line);
                        scanner.pushback();
                    }
                }
            }
        }
        nodeStmt();
    }


    public void edgeStmt(Node origin) throws IOException {
        next();

        Node dest = null;

        if (t.type == '-') {
            next();

            for (Node node : nodes) {
                if (node.getName().equals(t.text)) {
                    dest = node;
                }
            }

            Edge edge = new Edge(origin, dest, forces);
            edges.add(edge);

            next();

            if (t.type == '[') {
                System.out.println("Entering edge attribute:" + t.text);
                //edgeAttr(temp);
            } else {
                //there are no attributes for this edge.
                scanner.pushback();
            }
        } else {
            System.out.println("There was an error at line: " + t.line);
            //return;
        }

    }

    public void node() throws IOException {
        if (nodes != null) {
            //add new node.
            index++;
            nodes.add(new Node(forces, t.text, index));
        }
    }

    public ArrayList<Node> getNodes() {
        return nodes;
    }

    public ArrayList<Edge> getEdges() {
        return edges;
    }
}
package Tue;

import Tue.objects.Edge;
import Tue.objects.Node;
import Tue.parser.DotParser;
import Tue.parser.DotScanner;
import Tue.parser.Token;

import java.awt.EventQueue;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

public class Main extends JPanel implements ActionListener {

    private final Timer timer = new Timer(40, this);
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    public static void main(String[] args)
    {
        EventQueue.invokeLater(new Runnable()
        {
            @Override
            public void run() {
                new Main().create();
            }
        });
    }

    private void create()
    {
        parserInput();

        JFrame f = new JFrame("Graph To Map");
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.add(this);
        f.pack();
        f.setLocation(400, 100);
        //f.setLocationRelativeTo(null);
        f.setSize(1280, 800);
        f.setVisible(true);
        timer.start();
    }

    public Main()
    {
        super(true);
        this.setOpaque(false);
        this.addMouseListener(new MouseHandler());
    }

    @Override
    protected void paintComponent(Graphics g)
    {
        super.paintComponent(g);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        this.repaint();
    }

    private class MouseHandler extends MouseAdapter {

        @Override
        public void mousePressed(MouseEvent e) {
            super.mousePressed(e);
        }
    }

    private void parserInput()
    {
        System.out.println("Hello World");

        DotScanner scanner = null;
        try {
            scanner = new DotScanner(new FileReader("datasets/universities.gv"));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        DotParser parser = new DotParser(scanner, nodes, edges);
        try {
            parser.graph();
        } catch (IOException e) {
            e.printStackTrace();
        }

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
    }

}
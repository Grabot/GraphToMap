package Tue.load;

import Tue.objects.Edge;
import Tue.objects.Node;
import Tue.parser.DotParser;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Display extends JPanel implements ActionListener
{
    private DotParser parser;
    private final Timer timer = new Timer(40, this);
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    public void create()
    {
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

    public Display(DotParser parser)
    {
        super(true);

        this.parser = parser;
        nodes = parser.getNodes();
        edges = parser.getEdges();

        this.setOpaque(false);
        this.addMouseListener(new MouseHandler());
    }

    @Override
    protected void paintComponent(Graphics g)
    {
        super.paintComponent(g);

        drawNodes(g);
        drawEdges(g);

    }

    private void drawNodes( Graphics g )
    {
        for (Node node : nodes)
        {
            int radius = 20;
            g.fillOval((int)(node.getX()-(radius/2)), (int)(node.getY()-(radius/2)), radius, radius);
        }
    }

    private void drawEdges( Graphics g )
    {
        for (Edge edge : edges )
        {
            g.drawLine((int)edge.getSource().getX(), (int)edge.getSource().getY(), (int)edge.getDest().getX(), (int)edge.getDest().getY());
        }
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
}

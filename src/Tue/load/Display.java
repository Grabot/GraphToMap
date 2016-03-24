package Tue.load;

import Tue.Main;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;
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
    private Main main;
    private final Timer timer = new Timer(40, this);

    private ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();
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

    public Display( Main main )
    {
        super(true);

        this.main = main;

        this.clusternodes = main.clusternodes;
        this.clusteredges = main.clusteredges;

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
        for (ClusterNode Cnode : clusternodes)
        {
            int radius = 20;
            g.fillOval((int)(Cnode.getPos().x-(radius/2)), (int)(Cnode.getPos().y-(radius/2)), radius, radius);
        }
    }

    private void drawEdges( Graphics g )
    {
        for (ClusterEdge edge : clusteredges )
        {
            g.drawLine((int)edge.getSource().getPos().x, (int)edge.getSource().getPos().y, (int)edge.getDest().getPos().x, (int)edge.getDest().getPos().y);
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

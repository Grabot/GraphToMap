package Tue.load;

import Tue.Main;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;
import Tue.objects.Edge;
import Tue.objects.Node;
import Tue.parser.DotParser;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Display extends JPanel implements ActionListener
{
    private Main main;
    private final Timer timer;

    private ArrayList<ClusterNode> clusternodes = new ArrayList<ClusterNode>();
    private ArrayList<ClusterEdge> clusteredges = new ArrayList<ClusterEdge>();

    long lastLoopTime;
    float delta;

    private boolean showEdges = true;
    private boolean movement = true;

    public void create()
    {
        lastLoopTime = System.currentTimeMillis();

        JFrame f = new JFrame("Graph To Map");
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.add(this);
        f.pack();
        f.setLocation(400, 100);
        //f.setLocationRelativeTo(null);
        f.setSize(main.width, main.height);
        f.setVisible(true);

        f.addKeyListener(new KeyListener() {
            public void keyPressed(KeyEvent e) {
                if( e.getKeyCode() == KeyEvent.VK_E )
                {
                    showEdges = (!showEdges);
                }
                if( e.getKeyCode() == KeyEvent.VK_V )
                {
                    movement = (!movement);
                    for (ClusterNode node : clusternodes) {
                        node.setForce(new Vector2(0, 0));
                        node.setVel( new Vector2(0, 0));
                    }
                }
            }
            public void keyReleased(KeyEvent e) {
            }
            public void keyTyped(KeyEvent e) {
            }
        });

        timer.start();
    }

    public Display( Main main )
    {
        super(true);

        this.main = main;
        timer = new Timer(main.delta, this);

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
            if( showEdges ) {
                g.drawLine((int) edge.getSource().getPos().x, (int) edge.getSource().getPos().y, (int) edge.getDest().getPos().x, (int) edge.getDest().getPos().y);
            }
        }
    }

    @Override
    public void actionPerformed(ActionEvent e)
    {
        //get delta in miliseconds
        delta = System.currentTimeMillis() - lastLoopTime;
        lastLoopTime = System.currentTimeMillis();
        //get correct delta
        delta = (delta / 1000);
        //delta = 0.040f;

        calculatePos();

        this.repaint();
    }

    private void calculatePos()
    {
        Vector2[] oldpos = new Vector2[clusternodes.size()];

        //we will use a midpoint calculation to numerically solve the differential equation

        calculateForces();
        //calculate velocity and position for all nodes.
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            oldpos[i] = clusternodes.get(i).getPos();
            clusternodes.get(i).setVel( new Vector2((clusternodes.get(i).getVel().x + (clusternodes.get(i).getForce().x * delta)), (clusternodes.get(i).getVel().y + (clusternodes.get(i).getForce().y * delta ))));
            clusternodes.get(i).setPos( new Vector2((clusternodes.get(i).getPos().x + (clusternodes.get(i).getVel().x * (delta/2))), (clusternodes.get(i).getPos().y + (clusternodes.get(i).getVel().y * (delta/2) ))));
        }
        calculateForces();
        for( int i = 0; i < clusternodes.size(); i++ )
        {
            clusternodes.get(i).setVel( new Vector2((clusternodes.get(i).getVel().x + (clusternodes.get(i).getForce().x * delta)), (clusternodes.get(i).getVel().y + (clusternodes.get(i).getForce().y * delta ))));
            clusternodes.get(i).setPos( new Vector2((oldpos[i].x + (clusternodes.get(i).getVel().x * delta)), (oldpos[i].y + (clusternodes.get(i).getVel().y * delta ))));
        }
    }

    private void calculateForces()
    {
        for (ClusterNode node : clusternodes) {
            node.setForce(new Vector2(0, 0));
        }
        //apply new forces, we clear them first since nodes can occur for multiple edges and the forces accumulate
        for (ClusterEdge edge : clusteredges) {
            edge.ApplyForces();
        }

        for( ClusterNode node : clusternodes )
        {
            node.ApplyForces();
        }
    }

    private class MouseHandler extends MouseAdapter {

        @Override
        public void mousePressed(MouseEvent e) {
            super.mousePressed(e);
        }
    }
}

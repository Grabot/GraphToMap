package Tue.load;

import Tue.Main;
import Tue.objects.Cluster;
import Tue.objects.ClusterEdge;
import Tue.objects.ClusterNode;

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

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusterEdges = new ArrayList<ClusterEdge>();

    long lastLoopTime;
    float delta;

    private boolean showEdges = true;
    private boolean movement = true;

    private Renderer render;
    private Simulation simulation;

    public void create()
    {
        lastLoopTime = System.currentTimeMillis();

        render = new Renderer(clusters, clusterEdges);
        simulation = new Simulation(render, clusters, clusterEdges, main.width, main.height);

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
                    for (ClusterNode node : clusters) {
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

        this.clusters = main.clusters;
        this.clusterEdges = main.clusterEdges;

        this.setOpaque(false);
        this.addMouseListener(new MouseHandler());
    }

    @Override
    protected void paintComponent(Graphics g)
    {
        super.paintComponent(g);
        render.draw(g, showEdges );
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

        simulation.update( delta );

        this.repaint();
    }

    private class MouseHandler extends MouseAdapter {

        @Override
        public void mousePressed(MouseEvent e) {
            super.mousePressed(e);
        }
    }
}

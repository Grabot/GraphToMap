package Tue.load.Display;

import Tue.Main;
import Tue.load.Forces.Force;
import Tue.load.PointPlacement;
import Tue.load.Vector2;
import Tue.objects.*;

import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.event.MouseInputAdapter;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;

/**
 * Created by s138362 on 16-3-2016.
 */
public class Display extends JPanel implements ActionListener
{
    private final Timer timer;

    private ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    private ArrayList<ClusterEdge> clusterEdges = new ArrayList<ClusterEdge>();
    private ArrayList<Node> nodes = new ArrayList<Node>();
    private ArrayList<Edge> edges = new ArrayList<Edge>();

    long lastLoopTime;
    float delta;

    private boolean showEdges = false;
    private boolean showDelaunay = false;
    private boolean showSites = false;
    private boolean movement = true;
    private boolean showData = false;

    private Renderer render;
    private Simulation simulation;

    private Force forces;
    private PointPlacement points;
    private int width;
    private int height;
    private double[][] clusterDistance;
    private double[][] pairD;

    public double graphScaling = 0;

    public double zoom = 1;
    public double zoominverse = 1;
    public double movementX = 0;
    public double movementY = 0;

    private double clickPointX = 0;
    private double clickPointY = 0;

    private double currentX = 0;
    private double currentY = 0;

    public void create()
    {
        lastLoopTime = System.currentTimeMillis();

        render = new Renderer(this, clusters, clusterEdges, graphScaling);
        simulation = new Simulation(this, render, clusters, clusterEdges, width, height, forces, pairD, clusterDistance, points, nodes, edges, graphScaling );

        JFrame f = new JFrame("Graph To Map");
        f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f.add(this);
        f.pack();
        f.setLocation(400, 100);
        //f.setLocationRelativeTo(null);
        f.setSize(width, height);
        f.setVisible(true);

        f.addMouseWheelListener( new MouseWheelListener() {

            @Override
            public void mouseWheelMoved(MouseWheelEvent e)
            {
                if( e.getWheelRotation() == -1 )
                {
                    zoom = (zoom + 0.01);
                    zoominverse = (zoominverse - 0.004);
                }
                else if( e.getWheelRotation() == 1 )
                {
                    zoom = (zoom - 0.01);
                    zoominverse = (zoominverse + 0.004);
                }
                else
                {

                }
            }

        });
        f.addKeyListener(new KeyListener() {
            public void keyPressed(KeyEvent e) {
                if( e.getKeyCode() == KeyEvent.VK_E )
                {
                    showEdges = (!showEdges);
                }
                if( e.getKeyCode() == KeyEvent.VK_D)
                {
                    showDelaunay = (!showDelaunay);
                }
                if( e.getKeyCode() == KeyEvent.VK_S)
                {
                    showSites = (!showSites);
                }
                if( e.getKeyCode() == KeyEvent.VK_X)
                {
                    showData = (!showData);
                }
                if( e.getKeyCode() == KeyEvent.VK_Z )
                {
                    zoom = (zoom+0.01);
                    zoominverse = (zoominverse - 0.004);
                }
                if( e.getKeyCode() == KeyEvent.VK_A ) {
                    zoom = (zoom - 0.01);
                    zoominverse = (zoominverse + 0.004);
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

        timer = new Timer(main.delta, this);

        this.clusters = main.clusters;
        this.clusterEdges = main.clusterEdges;
        this.nodes = main.nodes;
        this.edges = main.edges;
        this.forces = main.forces;
        this.width = main.width;
        this.height = main.height;
        this.pairD = main.pairD;
        this.clusterDistance = main.clusterDistance;
        this.points = main.points;
        this.graphScaling = main.graphScaling;

        this.setOpaque(false);
        this.addMouseListener(new MouseHandler());
        this.addMouseMotionListener( new MouseHandler());
    }

    @Override
    protected void paintComponent(Graphics g)
    {
        super.paintComponent(g);
        render.draw(g, showEdges, showDelaunay, showSites, showData );
    }

    @Override
    public void actionPerformed(ActionEvent e)
    {
        //get delta in miliseconds
        delta = System.currentTimeMillis() - lastLoopTime;
        lastLoopTime = System.currentTimeMillis();
        //get correct delta
        //delta = (delta / 1000);
        delta = 0.040f;

        simulation.update( delta, false, false );

        this.repaint();
    }

    private class MouseHandler extends MouseInputAdapter
    {

        @Override
        public void mousePressed(MouseEvent e)
        {
            clickPointX = e.getX();
            clickPointY = e.getY();
        }

        @Override
        public void mouseReleased(MouseEvent e) {
            currentX = movementX;
            currentY = movementY;
        }

        @Override
        public void mouseDragged(MouseEvent e)
        {
            movementX = -((clickPointX - e.getX())-currentX);
            movementY = -((clickPointY - e.getY())-currentY);
        }

    }


    public BufferedImage createImage() {

        int w = this.getWidth();
        int h = this.getHeight();
        BufferedImage bi = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
        Graphics2D g = bi.createGraphics();
        this.paint(g);
        return bi;
    }
}
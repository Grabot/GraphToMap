/*******************************************************************************
 * Copyright (c) 2013 Arlind Nocaj, University of Konstanz.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * For distributors of proprietary software, other licensing is possible on request: arlind.nocaj@gmail.com
 * 
 * This work is based on the publication below, please cite on usage, e.g.,  when publishing an article.
 * Arlind Nocaj, Ulrik Brandes, "Computing Voronoi Treemaps: Faster, Simpler, and Resolution-independent", Computer Graphics Forum, vol. 31, no. 3, June 2012, pp. 855-864
 ******************************************************************************/
package Tue.load.Geometry;

import Tue.load.voronoitreemap.datastructure.OpenList;
import Tue.load.voronoitreemap.diagram.PowerDiagram;
import Tue.load.voronoitreemap.j2d.Point2D;
import Tue.load.voronoitreemap.j2d.PolygonSimple;
import Tue.load.voronoitreemap.j2d.Site;
import Tue.objects.Cluster;
import Tue.objects.Node;

import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.Random;

/**
 * Core class for generating Voronoi Treemaps. position and weight of sites is
 * changed on each iteration to get the wanted area for a cell.
 * 
 * @author Arlind Nocaj
 */
public class VoronoiCore {

	protected PolygonSimple clipPolygon;
	protected OpenList sites;
	protected PowerDiagram diagram;
	private int currentIteration;
	protected double currentAreaError = 1.0;

	// level in the hierarchy, level=0 is the first layer
	private int level;
	private Point2D center;
	private double scale;
	private AffineTransform transform;
	private double currentErrorMax;

	public OpenList getSiteList() {
		return sites;
	}

	public void setSiteList( OpenList sites )
	{
		this.sites = sites;
	}

	/**
	 * The resulting Voronoi cells are clipped with this polygon
	 * 
	 * @param polygon
	 *            clipping polygon
	 */
	public void setClipPolygon(PolygonSimple polygon) {
		clipPolygon = polygon;
		if (diagram != null)
			diagram.setClipPoly(polygon);
	}

	private void init() {
		diagram = new PowerDiagram();
	}

	public VoronoiCore() {
		sites = new OpenList();
		init();
	}

	public VoronoiCore(PolygonSimple clipPolygon) {
		this();
		setClipPolygon(clipPolygon);
	}

	/**
	 * Adds a site to the Voronoi diagram.
	 * 
	 * @param site
	 */
	public void addSite(Site site) {
		sites.add(site);
	}

	public void iterateSimple() {
		// if(currentIteration<=settings.maxIterat){
		moveSites();
		//checkPointsInPolygon(sites);
		// }

		// voroDiagram();//does not seem to be necessary
		// fixNoPolygonSites();

		// adapt weights
		adaptWeightsSimple();
		voroDiagram();

		currentAreaError = computeAreaError(sites);
		currentErrorMax = computeMaxError(sites);
		currentIteration++;
	}

	private void checkPointsInPolygon(OpenList sites) {
		boolean outside = false;
		for (int i = 0; i < sites.size; i++) {
			Site point = sites.array[i];
			if (!clipPolygon.contains(point.x, point.y)) {
				outside = true;
				Point2D p = clipPolygon.getInnerPoint();
				point.setXY(p.x, p.y);
			}
		}
		if (outside)
			fixWeightsIfDominated(sites);
	}

	public double computeAreaError(OpenList sites) {
		double completeArea = clipPolygon.getArea();
		double errorArea = 0;
		for (int z = 0; z < sites.size; z++) {
			Site point = sites.array[z];
			PolygonSimple poly = point.getPolygon();
			double currentArea = (poly == null) ? 0.0 : poly.getArea();
			double wantedArea = completeArea * point.getPercentage();
			errorArea += Math.abs(wantedArea - currentArea)
					/ (completeArea * 2.0);
		}
		return errorArea;
	}

	private double computeMaxError(OpenList sites2) {
		double completeArea = clipPolygon.getArea();
		double maxError = 0;
		for (int z = 0; z < sites.size; z++) {
			Site point = sites.array[z];
			PolygonSimple poly = point.getPolygon();
			double currentArea = (poly == null) ? 0.0 : poly.getArea();
			double wantedArea = completeArea * point.getPercentage();
			double error = Math.abs(wantedArea - currentArea) / (wantedArea);
			maxError = Math.max(error, maxError);
		}
		return maxError;
	}

	public void setOldPoint()
	{
		for (Site point : sites) {
			PolygonSimple poly = point.getPolygon();
			if (poly != null) {
				Point2D centroid = poly.getCentroid();
				double centroidX = centroid.getX();
				double centroidY = centroid.getY();
				if (clipPolygon.contains(centroidX, centroidY)) {
					point.setOldXY(point.getX(), point.getY());
				}
			}
		}
	}

	public void moveSitesBackNormal(ArrayList<Node> nodes )
	{
		for( Node n : nodes )
		{
			Site s = n.getSite();
			s.setXY(n.getPos().getX(), n.getPos().getY());
		}
	}

	public void moveSitesBackCluster(ArrayList<Cluster> clusters)
	{
		for( Cluster c : clusters )
		{
			Site s = c.getSite();
			s.setXY(c.getPos().getX(), c.getPos().getY());
		}
	}

	public void moveSites() {
		for (Site point : sites) {
			PolygonSimple poly = point.getPolygon();
			if (poly != null) {
				Point2D centroid = poly.getCentroid();
				double centroidX = centroid.getX();
				double centroidY = centroid.getY();
				if (clipPolygon.contains(centroidX, centroidY)) {
					point.setOldXY(point.getX(), point.getY());
					point.setXY(centroidX, centroidY);
				}
			}
		}
	}

	public void adaptWeightsSimple() {
		Site[] array = sites.array;
		int size = sites.size;
		Random rand = new Random(5);
		double averageDistance = getGlobalAvgNeighbourDistance(sites);
		// double averageWeight=getAvgWeight(sites);
		// averageDistance+=averageWeight;
		double error = computeAreaError(sites);
		for (int z = 0; z < size; z++) {
			Site point = array[z];
			PolygonSimple poly = point.getPolygon();

			// if(poly==null)
			// System.out.println(point.getWeight()+"\t"+error);
			double completeArea = clipPolygon.getArea();
			double currentArea = (poly == null) ? 0.0 : poly.getArea();
			double wantedArea = completeArea * point.getPercentage();

			double increase = wantedArea / currentArea;
			if (currentArea == 0.0)
				increase = 2.0;

			double weight = point.getWeight();

			double step = 0;
			double errorTransform = (-(error - 1) * (error - 1) + 1);

			// errorTransform=error;
			// errorTransform=Math.max(errorTransform, settings.errorThreshold);
			// if(currentIteration>settings.maxIterat)
			// errorTransform*=rand.nextDouble();

			step = 1.0 * averageDistance * errorTransform;
			// step=2*averageDistance*error;
			double epsilon = 0.01;
			if (increase < (1.0 - epsilon))
				weight -= step;
			else if (increase > (1.0 + epsilon))
				weight += step;

			point.setWeight(weight);

			// debug purpose
			point.setLastIncrease(increase);

		}
	}

	private void fixWeightsIfDominated(OpenList sites) {

		for (Site s : sites) {
			double weight = s.getWeight();
			if (Double.isNaN(weight)) {
				s.setWeight(0.00000000001);
			}
		}

		for (Site s : sites) {
			for (Site q : sites) {
				if (s != q) {
					double distance = s.distance(q) * 0.999;
					if (Math.sqrt(s.getWeight()) >= distance) {
						double weight = distance * distance;
						q.setWeight(weight);
					}
				}
			}
		}
	}

	private double getGlobalAvgNeighbourDistance(OpenList sites) {
		double avg = 0;
		int num = 0;
		for (Site point : sites)
			if (point.getNeighbours() != null)
				for (Site neighbour : point.getNeighbours()) {
					double distance = neighbour.distance(point);
					avg += distance;
					num++;
				}
		avg /= num;
		return avg;
	}

	/**
	 * Computes the diagram and sets the results
	 */
	public void voroDiagram() {
		PowerDiagram diagram = new PowerDiagram();
		diagram.setSites(sites);
		diagram.setClipPoly(clipPolygon);
		diagram.computeDiagram();
	}

	public void doIterate( float error ) {

		voroDiagram();

		while (true) {

			iterateSimple();

			if( computeAreaError(sites) < error )
			{
				break;
			}
		}

	}

	public void setSites(OpenList sites) {
		this.sites = sites;
	}

	public OpenList getSites() {
		return sites;
	}


	public void normalizeSites(OpenList sites) {
		double sum = 0;
		Site[] array = sites.array;
		int size = sites.size;
		for (int z = 0; z < size; z++) {
			Site s = array[z];
			sum += s.getPercentage();
		}
		for (int z = 0; z < size; z++) {
			Site s = array[z];
			s.setPercentage(s.getPercentage() / sum);
		}

	}


}

package gdx.tue;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.*;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.math.Vector2;
import gdx.tue.objects.Edge;
import gdx.tue.objects.Node;
import gdx.tue.parser.DotParser;
import gdx.tue.parser.DotScanner;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

public class Main extends ApplicationAdapter {

	private OrthographicCamera camera;
	private SpriteBatch batch;
	private Texture texture;
	private Sprite sprite;

	private ShapeRenderer shapeRenderer;

	Color greenClear = new Color(0, 1, 0, 0);

	private ArrayList<Node> nodes = new ArrayList<Node>();
	private ArrayList<Edge> edges = new ArrayList<Edge>();

	@Override
	public void create ()
	{
		parserInput("datasets/sample.gv");

		nodes.get(0).addPos( new Vector2(-50, 50));
		nodes.get(1).addPos( new Vector2(-500, -200));
		nodes.get(2).addPos( new Vector2(200, 50));
		nodes.get(3).addPos( new Vector2(300, 20));
		nodes.get(4).addPos( new Vector2(0, 0));
		nodes.get(5).addPos( new Vector2(-10, 80));

		float w = Gdx.graphics.getWidth();
		float h = Gdx.graphics.getHeight();

		camera = new OrthographicCamera(w, h);
		batch = new SpriteBatch();

		shapeRenderer = new ShapeRenderer();
	}

	@Override
	public void render ()
	{
		Gdx.gl.glClearColor(0, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

		batch.setProjectionMatrix(camera.combined);
		shapeRenderer.setProjectionMatrix(camera.combined);

		batch.begin();
		batch.end();

		drawNodes();
		drawEdges();

		shapeRenderer.setColor(greenClear);
	}

	private void drawNodes()
	{
		for( int i = 0; i < nodes.size(); i++ )
		{
			shapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
			shapeRenderer.circle(nodes.get(i).getPosX(), nodes.get(i).getPosY(), 5);
			shapeRenderer.end();
		}
	}

	private void drawEdges()
	{
		for( int i = 0; i < edges.size(); i++ )
		{
			shapeRenderer.begin(ShapeRenderer.ShapeType.Line);
			shapeRenderer.line( edges.get(i).getFrom().getPosX(), edges.get(i).getFrom().getPosY(), edges.get(i).getDest().getPosX(), edges.get(i).getDest().getPosY() );
			shapeRenderer.end();
		}
	}

	private void parserInput(String input)
	{

		FileReader reader = null;
		try
		{
			reader = new FileReader(input);
		}
		catch (FileNotFoundException e)
		{
			e.printStackTrace();
		}

		DotScanner scanner = new DotScanner(reader);
		DotParser parser = new DotParser(scanner, nodes, edges);

		try
		{
			parser.graph();
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}

		nodes = parser.getNodes();
		edges = parser.getEdges();

		/*
		for( int i = 0; i < nodes.size(); i++ )
		{
			System.out.println("node: " + nodes.get(i).getIndex() + " with label: " + nodes.get(i).getLabel() + " it has cluster: " + nodes.get(i).getCluster() );
		}

		for( int i = 0; i < edges.size(); i++ )
		{
			System.out.println("edge: " + i + " edge from " + edges.get(i).getFrom().getName() + " to " + edges.get(i).getDest().getName() );
		}
		*/
	}
}


package gdx.tue;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import gdx.tue.objects.Edge;
import gdx.tue.objects.Node;
import gdx.tue.parser.DotParser;
import gdx.tue.parser.DotScanner;
import gdx.tue.parser.Token;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.ArrayList;

public class Main extends ApplicationAdapter {
	SpriteBatch batch;
	Texture img;

	public ArrayList<Node> nodes = new ArrayList<Node>();
	public ArrayList<Edge> edges = new ArrayList<Edge>();

	@Override
	public void create () {

		FileReader reader = null;
		try
		{
			reader = new FileReader("datasets/sample.gv");
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

		for( int i = 0; i < nodes.size(); i++ )
		{
			System.out.println("node: " + nodes.get(i).getIndex() + " with label: " + nodes.get(i).getLabel() + " it has cluster: " + nodes.get(i).getCluster() );
		}

		for( int i = 0; i < edges.size(); i++ )
		{
			System.out.println("edge: " + i + " edge from " + edges.get(i).getFrom().getName() + " to " + edges.get(i).getDest().getName() );
		}

		batch = new SpriteBatch();
		img = new Texture("badlogic.jpg");
	}

	@Override
	public void render () {
		Gdx.gl.glClearColor(1, 0, 0, 1);
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);
		batch.begin();
		batch.draw(img, 0, 0);
		batch.end();
	}
}

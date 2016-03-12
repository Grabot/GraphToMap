package Tue;

/**
 * Created by s138362 on 12-3-2016.
 */
public class Node
{

    private String name;
    private String label;
    private int index;
    public Node(String name, int index)
    {
        this.name = name;
        this.index = index;
    }

    public void addLabel(String label)
    {
        this.label = label;
    }

    public int getIndex() {
        return index;
    }

    public String getName()
    {
        return name;
    }

    public String getLabel()
    {
        return label;
    }
}

package Tue;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class Main {

    public static void main(String[] args)
    {
        System.out.println("Hello World");

        File file = new File("datasets/sample.gv");
        Scanner s = null;
        try
        {
            s = new Scanner(file);
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }

        DotParser parser = new DotParser(s);
    }
}

package Tue.parser;

import java.io.IOException;

/**
 * Created by s138362 on 12-3-2016.
 */
public interface TokenStream {
    public Token nextToken() throws IOException;
}
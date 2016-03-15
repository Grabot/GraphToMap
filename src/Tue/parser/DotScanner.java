package Tue.parser;

import java.io.IOException;
import java.io.Reader;
import java.io.StreamTokenizer;

/**
 * Created by s138362 on 12-3-2016.
 */
public class DotScanner implements TokenStream {

    protected StreamTokenizer tk;
    public DotScanner(Reader input)
    {
        tk = new StreamTokenizer(input);
        setSyntax(tk);
    }

    protected void setSyntax(StreamTokenizer tk)
    {
        tk.resetSyntax();
        tk.eolIsSignificant(false);
        tk.slashStarComments(true);
        tk.slashSlashComments(true);
        tk.whitespaceChars(0, ' ');
        tk.wordChars(35, 255);
        tk.ordinaryChar('[');
        tk.ordinaryChar(']');
        tk.ordinaryChar('{');
        tk.ordinaryChar('}');
        tk.ordinaryChar(',');
        tk.ordinaryChar('-');
        tk.ordinaryChar('>');
        tk.ordinaryChar('/');
        tk.ordinaryChar('*');
        tk.quoteChar('"');
        tk.whitespaceChars(';', ';');
        tk.ordinaryChar('=');
    }

    public void pushback(){
        tk.pushBack();
    }

    @Override
    public Token nextToken() throws IOException
    {
        tk.nextToken();

        if(tk.ttype == tk.TT_WORD || tk.ttype == '"')
        {
            return new Token(tk.ttype,tk.sval,tk.lineno());
        }
        else if(tk.ttype == tk.TT_NUMBER)
        {
            return  new Token(tk.ttype,tk.toString(),tk.lineno());
        }
        else if(tk.ttype == tk.TT_EOF || tk.ttype == tk.TT_EOL)
        {
            return new Token(tk.ttype,"",tk.lineno());
        }
        else
        {
            return new Token(tk.ttype,Character.toString((char)tk.ttype),tk.lineno());
        }

    }
}

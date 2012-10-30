package ex3;

import java.awt.image.*;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;
import javax.swing.*;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.MessageBuffers;


public class MapRender extends JFrame{

    public MapRender(){
        this.setSize(600, 600);
        this.setVisible(true);
        BufferedImage image = new BufferedImage(600, 600, BufferedImage.TYPE_INT_RGB);

    }


}



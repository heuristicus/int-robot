/*
package ex4;

import java.awt.Color;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import javax.swing.JFrame;
import javax.swing.JPanel;


public class FaceDetectGUI {

    static class Window extends JFrame {

	public static final int WINDOW_OFFSET_X = 100;
	public static final int WINDOW_OFFSET_Y = 100;

//	public static final int WINDOW_SIZE_X = 512;
//	public static final int WINDOW_SIZE_Y = 512;
	
	public Window(JPanel panel, int windowSizeX, int windowSizeY) {
            setTitle("LaserPlotWindow");
            setSize(windowSizeX, windowSizeY);
            setBounds(WINDOW_OFFSET_X, WINDOW_OFFSET_Y, windowSizeX, windowSizeY);
            setResizable(true);
            setDefaultCloseOperation(EXIT_ON_CLOSE);
            setContentPane(panel);

            this.addKeyListener(new KeyListener() {
                @Override
                public void keyTyped(KeyEvent e) {}

                @Override
                public void keyPressed(KeyEvent e) {
                    if (KeyEvent.VK_ESCAPE == e.getKeyCode()) {
                        System.exit(0);
                    }
                }

                @Override
                public void keyReleased(KeyEvent e) {}
            });
	}
    }

    public static final Color BACKGROUND_COLOUR = Color.black;

    JPanel panel;


    static class Panel extends JPanel {
        int width;
        int height;

        public Panel(int width, int height) {
            this.width = width;
            this.height = height;
        }
        
//        @Override
//    	public void paint(Graphics g) {
//            g.setColor(BACKGROUND_COLOUR);
//            g.fillRect(0, 0, sizeX*2, sizeY*2);
//            g.setColor(PLOT_POINT_COLOR);
//            float reading;
//            int y;
//
//            for (int x = 0; x < latestReadings.length; x++) {
//                reading = latestReadings[x];
//                y = sizeY-((int)(reading * 100.0f));
//
//                if (reading != 0) {
//                    g.fillOval(x, y, PLOT_POINT_WIDTH, PLOT_POINT_HEIGHT);
//                } else { // We draw zeros differently
//                    g.setColor(ZERO_POINT_COLOR);
//                    g.fillOval(x, y, ZERO_POINT_WIDTH, ZERO_POINT_HEIGHT);
//                    g.setColor(PLOT_POINT_COLOR);
//                }
//            }
//    	}
    }
}
*/
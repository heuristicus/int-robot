package util.laserplot;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import javax.swing.JFrame;
import javax.swing.JPanel;

@SuppressWarnings("serial")
public class LaserPlotWindow extends JFrame {

	public static final int WINDOW_OFFSET_X = 100;
	public static final int WINDOW_OFFSET_Y = 100;

//	public static final int WINDOW_SIZE_X = 512;
//	public static final int WINDOW_SIZE_Y = 512;
	
	public LaserPlotWindow(JPanel panel, int windowSizeX, int windowSizeY) {
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
                    if (e.VK_ESCAPE == e.getKeyCode()) {
                        System.exit(0);
                    }
                }

                @Override
                public void keyReleased(KeyEvent e) {}
            });
	}
}

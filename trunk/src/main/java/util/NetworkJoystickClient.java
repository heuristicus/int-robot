package util;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import javax.swing.JFrame;
import javax.swing.JPanel;


public class NetworkJoystickClient {

    public static final String IP = "localhost";
    public static final int PORT = 12585;

    public static final double DEFAULT_SPEED = 0.3;
    public static final double DEFAULT_ANGLE = 0.3;

    private static PrintWriter out = null;

    public static void main(String[] args) {
        Socket socket = null;

        try {
            socket = new Socket(IP, PORT);
            System.out.println("Socket initialised, getting PrintWriter");
            out = new PrintWriter(socket.getOutputStream());
            System.out.println("PrintWriter initialised, reading");

            KeyListener listener = new KeyListener(){
                @Override
                public void keyTyped(KeyEvent e) {}
                @Override
                public void keyPressed(KeyEvent e) {
                    System.out.println("Char: "+e.getKeyChar());
                    out.println("Char: "+e.getKeyChar());
                    
                    switch (e.getKeyCode()) {
                        case KeyEvent.VK_W:
                            out.write("MoveX:" + DEFAULT_SPEED+"\n");
                            break;
                        case KeyEvent.VK_S:
                            out.write("MoveX:-" + DEFAULT_SPEED+"\n");
                            break;
                        case KeyEvent.VK_A:
                            out.write("RotateZ:-" + DEFAULT_ANGLE+"\n");
                            break;
                        case KeyEvent.VK_D:
                            out.write("RotateZ:" + DEFAULT_ANGLE+"\n");
                            break;
                    }

                    out.flush();
                }
                @Override
                public void keyReleased(KeyEvent e) {}

            };
            
            JFrame window = getWindow(listener);
            window.setVisible(true);
        } catch (IOException e) {
            System.err.println("IOException. Oops");
            e.printStackTrace();
        }

        System.out.println("Client main closed.");
    }


    public static final int WINDOW_OFFSET_X = 800;
    public static final int WINDOW_OFFSET_Y = 800;

    public static final int WINDOW_SIZE_X = 100;
    public static final int WINDOW_SIZE_Y = 100;

    public static JFrame getWindow(KeyListener listener) {
        JFrame frame = new JFrame();
        frame.setTitle("NetworkJoystickClient");
        frame.setSize(WINDOW_SIZE_X, WINDOW_SIZE_Y);
        frame.setBounds(WINDOW_OFFSET_X, WINDOW_OFFSET_Y, WINDOW_SIZE_X, WINDOW_SIZE_Y);
        frame.setResizable(true);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        JPanel panel = new JPanel();
        frame.setContentPane(panel);
        frame.addKeyListener(listener);
        return frame;
    }


}

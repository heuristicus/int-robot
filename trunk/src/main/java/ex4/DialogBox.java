package ex4;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

class DialogBox extends JDialog implements ActionListener, Runnable {

    enum response {
        YES, NO, NORESPONSE
    }
    private JButton jButton_Yes = null;
    private JButton jButton_No = null;
    private response userResponse = response.NORESPONSE;
    private Thread thread = null;
    private int seconds = 0;
    private final int max = 15;//max number of seconds 
    private String title;
    
    private String message;
    private JLabel jLabel;

    public DialogBox(String title, String message) {
        setModal(true);
        this.message = message;

        setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
        this.title = title;
        setTitle(title);
        jButton_Yes = new JButton("Yes");
        jButton_Yes.addActionListener(this);
        
        jButton_No = new JButton("No");
        jButton_No.addActionListener(this);

        jLabel = new JLabel(this.message);
        jLabel.setHorizontalAlignment(JLabel.CENTER);

        Font font = new Font("Dialog", Font.PLAIN, 24);
        jLabel.setFont(font);
        JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.CENTER));
        Container cont = getContentPane();
        //cont.setLayout();
        cont.add(jLabel, BorderLayout.CENTER);
        buttonPanel.add(jButton_Yes);
        buttonPanel.add(jButton_No);
        cont.add(buttonPanel, BorderLayout.SOUTH);
        pack();
        thread = new Thread(this);
        thread.start();
        setSize(600, 300);
        setLocationRelativeTo(null);
        setVisible(true);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if (e.getSource() == jButton_Yes) {
            userResponse = response.YES;
        }
        if (e.getSource() == jButton_No) {
            userResponse = response.NO;
        }
        setVisible(false);
    }

    @Override
    public void run() {
        while (seconds < max) {
            seconds++;
            String secs = " (" + (max - seconds) + "s)";
            setTitle(title + secs);
            jLabel.setText(message + "\r\n" + secs);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException exc) {};
        }
        setVisible(false);
    }

    public response getUserResponse() {
        return userResponse;
    }

    public static void main(String[] args) {//testing
        DialogBox cd = new DialogBox("Meeting", "Would you like to attend a meeting?");
//        System.out.println(cd.getUserResponse().name());
        cd = null;
        System.exit(0);
    }
}//end


package driverstation;

import java.awt.BorderLayout;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JDialog;
import javax.swing.JLabel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class Keypad extends JDialog implements KeyListener {
	private JLabel label;
	private JLabel picLabel;
    private NetworkTable table;
    private State state;
    private StringBuffer inputBuffer;
    
    static ImageIcon cube;
    static ImageIcon cone;
    static ImageIcon none;
    static {
		try {
			cube = new ImageIcon(ImageIO.read(new File("images/cube.png")));
			cone = new ImageIcon(ImageIO.read(new File("images/cone.png")));
			none = new ImageIcon(ImageIO.read(new File("images/none.png")));
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    
	public Keypad() {
	    NetworkTableInstance inst = NetworkTableInstance.getDefault();
	    inst.startClient4("keypad");
	    inst.setServerTeam(1731);
	    inst.startDSClient();
		table = inst.getTable("KeyPad");
		//setLayout(new FlowLayout());
		label = new JLabel();
		label.setFont(new Font("Arial Black", Font.BOLD, 32));
		add(label, BorderLayout.SOUTH);
		picLabel = new JLabel(none);
		add(picLabel, BorderLayout.CENTER);
		state = State.INPUT;
		inputBuffer = new StringBuffer("");
		addWindowListener(new WindowAdapter() { 
		    @Override
		    public void windowClosing(WindowEvent e) {
		    	dispose();
		        System.exit(0);
		    }
		});
		setSize(700, 500);
		setVisible(true);
	}
	
    public static void main(String[] args) {
    	Keypad keypad = new Keypad();
    	keypad.addKeyListener(keypad);
    }

    enum KeypadControl {
    	GET_CONE    (111), // "/"
    	GET_CUBE    (106), // "*"
    	CONE_LEFT   (100), // "4"
    	CUBE_MIDDLE (101), // "5"
    	CONE_RIGHT  (102), // "6"
    	DEPLOY_LOW   (97), // "1"
    	DEPLOY_MID   (98), // "2"
    	DEPLOY_HIGH  (99), // "3"
    	CLEAR_ENTRY  (96); // "0"
    	
    	private final int value;
    	private static Map map = new HashMap<>();
    	
    	private KeypadControl(int value) {
    		this.value = value;
    	}
    	
    	static {
    		for(KeypadControl control : KeypadControl.values()) {
    			map.put(control.value, control);
    		}
    	}
    	
    	public static KeypadControl valueOf(int control) {
    		return (KeypadControl) map.get(control);
    	}
    	
    	public int getValue() {
    		return value;
    	}
    }
    
    enum State {
        INPUT,
        PENDING;
    }

    Object stateTransitionTable[][] = {
    	// CURRENT      INPUT                     OPERATION            ICON    NEXT
        {State.INPUT,   KeypadControl.GET_CONE,   "lightUpBot",        cone,   State.INPUT},
        {State.INPUT,   KeypadControl.GET_CUBE,   "lightUpBot",        cube,   State.INPUT},
        {State.INPUT,   KeypadControl.CONE_LEFT,   null,               cone,   State.PENDING},
        {State.INPUT,   KeypadControl.CUBE_MIDDLE, null,               cube,   State.PENDING},
        {State.INPUT,   KeypadControl.CONE_RIGHT,  null,               cone,   State.PENDING},
        {State.INPUT,   KeypadControl.CLEAR_ENTRY, "clearInputBuffer", none,   State.INPUT},
        {State.PENDING, KeypadControl.CLEAR_ENTRY, "clearInputBuffer", none,   State.INPUT},
        {State.PENDING, KeypadControl.DEPLOY_LOW,  "armJoystick",      null,   State.INPUT},
        {State.PENDING, KeypadControl.DEPLOY_MID,  "armJoystick",      null,   State.INPUT},
        {State.PENDING, KeypadControl.DEPLOY_HIGH, "armJoystick",      null,   State.INPUT},
    };
    
    void lightUpBot() {}
    void clearInputBuffer() {}
    void armJoystick() {}
    
	@Override
	public void keyPressed(KeyEvent e) {
        int keyCode = e.getKeyCode();
        if(KeypadControl.map.containsKey(keyCode)) {
        	KeypadControl control = KeypadControl.valueOf(keyCode);
        	String controlName = control.name();
        	switch(state) {
        	case INPUT:
            	switch(control) {
            	case GET_CONE: picLabel.setIcon(cone);
            		inputBuffer = new StringBuffer(controlName);
            		break;
            	case GET_CUBE: picLabel.setIcon(cube);
            		inputBuffer = new StringBuffer(controlName);
            		break;
            	case CLEAR_ENTRY: picLabel.setIcon(none);
            		inputBuffer = new StringBuffer("CLEAR");
            		label.setText("");
            		break;
            	case CONE_LEFT:
            	case CUBE_MIDDLE:
            	case CONE_RIGHT:
            		//
            		// TODO ERROR-CHECK HERE!!!
            		//      what if we have a cone and the driver presses CUBE_MIDDLE?
            		//
            		inputBuffer = new StringBuffer(controlName);
            		state = State.PENDING;
            		break;
            	default:
            		break;
            	}
        		break;
        	case PENDING:
            	switch(control) {
            	case DEPLOY_LOW: 
            	case DEPLOY_MID: 
            	case DEPLOY_HIGH:
            		inputBuffer.append("; " + controlName);
            		state = State.INPUT;
            		break;
            	case CLEAR_ENTRY: picLabel.setIcon(none);
            		inputBuffer = new StringBuffer("");
            		label.setText("");
            		state = State.INPUT;
            		break;
            	default:
            		break;
            	}
        		break;
        	}
        	String command = inputBuffer.toString();
        	if(command.length() > 0) {
		        label.setText(command);
		        if(state != State.PENDING) {
			        table.putValue("driver entry", NetworkTableValue.makeString(command));
			        System.out.println("sent " + command + " to network tables");
		        }
        	}
        }
	}

	@Override
	public void keyReleased(KeyEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void keyTyped(KeyEvent arg0) {
		// TODO Auto-generated method stub
		
	}
}

package driverstation;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.HashMap;
import java.util.Map;

import javax.swing.ImageIcon;
import javax.swing.JDialog;
import javax.swing.JLabel;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;

public class Keypad extends JDialog implements KeyListener {
	private static final long serialVersionUID = 1L;
	private JLabel commandLabel;
	private JLabel feedbackLabel;
	private JLabel picLabel;
    private NetworkTable table;
    private State state;
    private StringBuffer inputBuffer;
    private boolean sent;
    private ImageIcon cube;
    private ImageIcon cone;
    private ImageIcon none;
    private ImageIcon lock;
    
	public Keypad() {
		cone = new ImageIcon(Keypad.class.getResource("/images/cone.png"));
		cube = new ImageIcon(Keypad.class.getResource("/images/cube.png"));
		none = new ImageIcon(Keypad.class.getResource("/images/none.png"));
		lock = new ImageIcon(Keypad.class.getResource("/images/lock.png"));
	    NetworkTableInstance inst = NetworkTableInstance.getDefault();
	    inst.startClient4("keypad");
	    inst.setServerTeam(1731);
	    inst.startDSClient();
	    sent = false;
		table = inst.getTable("KeyPad");
		feedbackLabel = new JLabel();
		feedbackLabel.setFont(new Font("Arial Black", Font.BOLD, 24));
		feedbackLabel.setForeground(Color.RED);
		add(feedbackLabel, BorderLayout.NORTH);
		commandLabel = new JLabel();
		commandLabel.setFont(new Font("Arial Black", Font.BOLD, 32));
		commandLabel.setForeground(Color.BLUE);
		add(commandLabel, BorderLayout.SOUTH);
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
		setTitle("ALEXANDER G. COWBELL -- TEAM 1731 -- KEYPAD ENTRY PROGRAM");
		setSize(700, 500);
		setVisible(true);
	}
	
    public static void main(String[] args) {
    	Keypad keypad = new Keypad();
    	keypad.addKeyListener(keypad);
    }

    enum KeypadControl {
    	CONE        (111), // "/"
    	CUBE        (106), // "*"
    	SPARE_A       (8), // "<--"
    	GRID_1      (103), // "7"
    	GRID_2      (104), // "8"
    	GRID_3      (105), // "9"
    	SPARE_B     (109), // "-"
    	LEFT        (100), // "4"
    	MIDDLE      (101), // "5"
    	RIGHT       (102), // "6"
    	SPARE_C     (107), // "+"
    	SCORE_LOW    (97), // "1"
    	SCORE_MED    (98), // "2"
    	SCORE_HIGH   (99), // "3"
    	SPARE_D      (10), // "Ent"
    	CLEAR_ENTRY  (96), // "0"
    	SPARE_E     (110); // "."
    	
    	private final int value;
    	private static Map<Integer, KeypadControl> map = new HashMap<>();
    	
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
        PENDING1,
        PENDING2,
    	DONE
    }

    Object stateTransitionTable[][] = {
    	// CURRENT         INPUT                     OPERATION            ICON    NEXT
        {State.INPUT,      KeypadControl.CONE,       "lightUpBot",        cone,   State.INPUT},
        {State.INPUT,      KeypadControl.CUBE,       "lightUpBot",        cube,   State.INPUT},
        {State.INPUT,      KeypadControl.GRID_1,      null,               null,   State.PENDING1},
        {State.INPUT,      KeypadControl.GRID_2,      null,               null,   State.PENDING1},
        {State.INPUT,      KeypadControl.GRID_3,      null,               null,   State.PENDING1},
        {State.PENDING1,   KeypadControl.LEFT,        null,               null,   State.PENDING2},
        {State.PENDING1,   KeypadControl.MIDDLE,      null,               null,   State.PENDING2},
        {State.PENDING1,   KeypadControl.RIGHT,       null,               null,   State.PENDING2},
        {State.INPUT,      KeypadControl.CLEAR_ENTRY, "clearInputBuffer", none,   State.INPUT},
        {State.PENDING2,   KeypadControl.CLEAR_ENTRY, "clearInputBuffer", none,   State.INPUT},
        {State.PENDING2,   KeypadControl.SCORE_LOW,   "armJoystick",      null,   State.DONE},
        {State.PENDING2,   KeypadControl.SCORE_MED,   "armJoystick",      null,   State.DONE},
        {State.PENDING2,   KeypadControl.SCORE_HIGH,  "armJoystick",      null,   State.DONE},
    };
    
    void lightUpBot() {}
    void clearInputBuffer() {}
    void armJoystick() {}
    
	@Override
	public void keyPressed(KeyEvent e) {
        int keyCode = e.getKeyCode();
        // System.out.println(keyCode);
        if(keyCode == 144) {
        	picLabel.setIcon(lock);
        	feedbackLabel.setText("Press NumLock; then press CLEAR.");
        	return;
        }
        feedbackLabel.setText("");
        if(KeypadControl.map.containsKey(keyCode)) {
        	KeypadControl control = KeypadControl.valueOf(keyCode);
        	String controlName = control.name();
        	switch(state) {
        	case INPUT:
            	switch(control) {
            	case CONE:
            		picLabel.setIcon(cone);
            		inputBuffer = new StringBuffer(controlName);
            		break;
            	case CUBE:
            		picLabel.setIcon(cube);
            		inputBuffer = new StringBuffer(controlName);
            		break;
            	case CLEAR_ENTRY:
            		clear();
            		break;
            	case GRID_1:
            	case GRID_2:
            	case GRID_3:
            		if(inputBuffer.toString().toLowerCase().contains("cube") || inputBuffer.toString().toLowerCase().contains("cone")) {
            			inputBuffer.append("; " + controlName);
            			state = State.PENDING1;
            		}
            	}
            	break;
        	case PENDING1:
            	switch(control) {
            	case CLEAR_ENTRY:
            		clear();
            		break;
            	case LEFT:
            	case MIDDLE:
            	case RIGHT:
            		inputBuffer.append("; " + controlName);
            		state = State.PENDING2;
            	}
            	break;
           	case PENDING2:
           		switch(control) {
            	case CLEAR_ENTRY:
            		clear();
            		break;
            	case SCORE_MED:
            	case SCORE_HIGH:
            		String buf = inputBuffer.toString().toLowerCase();
            		if((buf.contains("cube") && buf.contains("middle")) ||
            			buf.contains("cone") && !buf.contains("middle")) {
            			// input is OK
            		}
            		else {
            			feedbackLabel.setText("TRY AGAIN!");
            			break;
            		}
            	case SCORE_LOW: 
            		inputBuffer.append("; " + controlName);
            		state = state.DONE;
            		sent = false;
            	}
           		break;
           	case DONE:
           		switch(control) {
            	case CLEAR_ENTRY:
            		clear();
            		break;
           		}
        	}
        	String command = inputBuffer.toString();
        	if(command.length() > 0) {
		        commandLabel.setText(command);
		        //if(!sent) {
			        table.putValue("driver entry", NetworkTableValue.makeString(command));
			        sent = true;
			        System.out.println("sent " + command + " to network tables");
		        //}
        	}
        }
	}

	private void clear() {
		picLabel.setIcon(none);
		inputBuffer = new StringBuffer("CLEAR");
		commandLabel.setText("");
		state = State.INPUT;
		sent = false;
	}
	
	@Override
	public void keyReleased(KeyEvent arg0) {
	}

	@Override
	public void keyTyped(KeyEvent arg0) {
	}
}
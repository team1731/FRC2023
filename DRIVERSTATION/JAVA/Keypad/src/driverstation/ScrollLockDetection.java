package driverstation;

import java.awt.AWTEvent;
import java.awt.Robot;
import java.awt.Toolkit;
import java.awt.event.AWTEventListener;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.concurrent.ExecutionException;
import java.util.function.Consumer;
import java.util.stream.Stream;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.SwingWorker;

public class ScrollLockDetection {
    private static final int SCROLL_LOCK = KeyEvent.VK_SCROLL_LOCK;
    private JFrame frame;

    public ScrollLockDetection() {
        frame = new JFrame();
        frame.setSize(400, 400);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.addWindowFocusListener(new WindowAdapter() {

            @Override
            public void windowGainedFocus(WindowEvent e) {
                showScrollLockStatus();
            }
        });
        registerGlobalScrollLockListener();
        frame.setVisible(true);

    }

    private void registerGlobalScrollLockListener() {
        Toolkit.getDefaultToolkit().addAWTEventListener(event -> {
            if (event instanceof KeyEvent) {
                KeyEvent keyEvent = (KeyEvent) event;
                if (keyEvent.getID() == KeyEvent.KEY_RELEASED && keyEvent.getKeyCode() == KeyEvent.VK_SCROLL_LOCK) {
                    showScrollLockStatus();
                }
            }
        }, AWTEvent.KEY_EVENT_MASK);
    }

    private void showScrollLockStatus() {
        ScrollLockDetector scrollLockDetector = new ScrollLockDetector(b -> {
            System.out.println("Scroll lock ON: " + b);
        });
        scrollLockDetector.execute();
    }

    class ScrollLockDetector extends SwingWorker<Boolean, Void> {
        private Consumer<Boolean> consumer;

        public ScrollLockDetector(Consumer<Boolean> consumer) {
            this.consumer = consumer;
        }

        @Override
        protected Boolean doInBackground() throws Exception {
            //First we have to remove all global key listeners so the robot does not fire them
            Toolkit toolkit = Toolkit.getDefaultToolkit();
            AWTEventListener[] globalKeyListeners = toolkit.getAWTEventListeners(AWTEvent.KEY_EVENT_MASK);
            while (toolkit.getAWTEventListeners(AWTEvent.KEY_EVENT_MASK).length > 0)
                toolkit.removeAWTEventListener(toolkit.getAWTEventListeners(AWTEvent.KEY_EVENT_MASK)[0]);

            Robot robot = new Robot();
            robot.keyPress(SCROLL_LOCK);
            robot.keyRelease(SCROLL_LOCK);
            Thread.sleep(3);
            robot.keyPress(SCROLL_LOCK);
            robot.keyRelease(SCROLL_LOCK);
            Thread.sleep(3);
            //Re-add the global key listeners
            Stream.of(globalKeyListeners).forEach(listener -> toolkit.addAWTEventListener(listener, AWTEvent.KEY_EVENT_MASK));
            return toolkit.getLockingKeyState(SCROLL_LOCK);
        }

        @Override
        protected void done() {
            try {
                Boolean isScrollLockOn = get();
                consumer.accept(isScrollLockOn);
            } catch (InterruptedException | ExecutionException e) {
                e.printStackTrace();
            }
        }
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            new ScrollLockDetection();
        });
    }
}

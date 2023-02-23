package driverstation;

import java.awt.Toolkit;
import java.awt.event.KeyEvent;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.NativeHookException;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;

public class GlobalKeyListenerExample implements NativeKeyListener {
    private static boolean scrollLock;

    @Override
    public void nativeKeyPressed(NativeKeyEvent e) {
    }

    @Override
    public void nativeKeyReleased(NativeKeyEvent e) {
        if (e.getKeyCode() == NativeKeyEvent.VC_SCROLL_LOCK) {
            scrollLock = !scrollLock;
            System.out.println("Scroll lock is:" + (scrollLock ? "ON" : "OFF"));
        }
    }

    @Override
    public void nativeKeyTyped(NativeKeyEvent e) {
    }

    public static void main(String[] args) {
        try {
            scrollLock = Toolkit.getDefaultToolkit().getLockingKeyState(KeyEvent.VK_SCROLL_LOCK);
            System.out.println("Initial state of scrollock: " + (scrollLock ? "ON" : "OFF"));
            GlobalScreen.registerNativeHook();
            Logger logger = Logger.getLogger(GlobalScreen.class.getPackage().getName());
            logger.setLevel(Level.WARNING);

            // Don't forget to disable the parent handlers.
            logger.setUseParentHandlers(false);
            // Don't forget to disable the parent handlers.
        } catch (NativeHookException ex) {
            System.err.println("There was a problem registering the native hook.");
            System.err.println(ex.getMessage());

            System.exit(1);
        }
        GlobalScreen.addNativeKeyListener(new GlobalKeyListenerExample());
    }
}
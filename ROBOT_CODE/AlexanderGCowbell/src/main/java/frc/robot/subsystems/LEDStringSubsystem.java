/*-------------------------------------------------------*/
/* LED SUBSYSTEM 2023                                    */
/* Code for controlling WS2812B LED RGB Strip using      */
/* RoboRio as a controller                               */
/*-------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.LedOption;

public class LEDStringSubsystem extends SubsystemBase{
    //WITH NEW LED STRIP, COLORS ARE IN RBG not RGB
    private static final int[] YELLOW = {255, 0, 120};
    private static final int[] PURPLE = {120, 150, 0};
    private static final int[] WHITE = {255, 255, 255};
    private static final int[] BLUE = {64, 0, 0};
    private static final int[] BLACK = {0, 0, 0};
    private static final int[] RED = {255, 0, 0};
    private Timer mTimer;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int length;
    private boolean blink;
    private LedOption currentColor;
    private double startBlink;
    private boolean switched;

    public LEDStringSubsystem() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(OpConstants.kPWM_LedSting);
        mTimer = new Timer();
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(65);
        length = m_ledBuffer.getLength();
        m_led.setLength(length);
    
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    
      @Override
      public void periodic() {
        //This method will be called once per scheduler run
        double elapsed = (mTimer.get() - startBlink);
        if (elapsed < 0.1){
          return;
        }
        if (blink) {
          if (!switched){
            setColor(LedOption.BLACK);
            switched = true;
          } else {
            setColor(currentColor);
            switched = false;
          }
          startBlink = mTimer.get();
        }
        else{
          setColor(currentColor);
        }
      }
      public void init() {
        // initialization stuff
        setColor(OpConstants.LedOption.INIT);
        mTimer.start();
      }
    
      void setFullColor(int r, int g, int b) {
        for(int i=0; i < m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
  
      } 

      /**
       * 5 LED blocks of Yellow/Blue for team colors 
       */
      void setTeamColorBlocks(){
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
          if(i%5 >= 0 && i%5 <= 4%5 && (i/5)%2 != 0){
            m_ledBuffer.setRGB(i, BLUE[0], BLUE[1], BLUE[2]);
          }else{
            m_ledBuffer.setRGB(i, YELLOW[0], YELLOW[1], YELLOW[2]);
          }
        }
      }


      /**
       * Blue to Yellow Gradient for team colors
       */
      /*void setTeamColorGradient(){
        int r=160, g=160, b=0;
        for(int i = 0; i < m_ledBuffer.getLength(); i++){
          m_ledBuffer.setRGB(i, r, g, b);
          r-=160/m_ledBuffer.getLength();
          g-=160/m_ledBuffer.getLength();
          b+=160/m_ledBuffer.getLength();
        }
      }*/

      public void setBlink(boolean blink){
        this.blink = blink; 
        this.startBlink = mTimer.get();
      }

      public void setColor(OpConstants.LedOption color) {
        // Fill the buffer with selection
        switch (color) {
          case INIT:
          setTeamColorBlocks();
          if (!blink){
              this.currentColor = LedOption.WHITE;
            }
          case WHITE:
            setFullColor(WHITE[0], WHITE[1], WHITE[2]);
            if (!blink){
              this.currentColor = LedOption.WHITE;
            }
            break;
          case YELLOW:
            setFullColor(YELLOW[0], YELLOW[1], YELLOW[2]);
            if (!blink){
              this.currentColor = LedOption.YELLOW;
            }  
            break;
          case PURPLE:
            setFullColor(PURPLE[0], PURPLE[1], PURPLE[2]);
            if (!blink){
              this.currentColor = LedOption.PURPLE;
            }  
            break;
          case BLUE:
            setFullColor(BLUE[0], BLUE[1], BLUE[2]);
            if (!blink){
              this.currentColor = LedOption.BLUE;
            }  
          case BLACK:
            setFullColor(BLACK[0], BLACK[1], BLACK[2]);
            break;
          case RED:
            setFullColor(RED[0], RED[1], RED[2]);
            break;
        }
      }
}
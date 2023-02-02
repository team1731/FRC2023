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

public class LEDStringSubsystem extends SubsystemBase{
    private Timer mTimer;
    // Var to store last hue of first pixel
    private int m_rainbowFirstPixelHue;
    private int count;
    private double elapsed;
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private OpConstants.LedOption mLedOption = OpConstants.LedOption.TEAM;
    private double delayNum;
    private boolean delay;
    private int length;

    public LEDStringSubsystem() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(OpConstants.kPWM_LedSting);
        mTimer = new Timer();
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(60);
        length  = m_ledBuffer.getLength();
        m_led.setLength(length);
    
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        count = 0;
      }
    
      @Override
      public void periodic() {
        // This method will be called once per scheduler run
        if (delay){
          if (mTimer.get() - elapsed > delayNum) {
            option(mLedOption);
            elapsed = mTimer.get();
          }
        } else {
          option(mLedOption);
        }
      }
    
      public void init() {
        // initialization stuff
        option(OpConstants.LedOption.TEAM);
        mTimer.start();
        elapsed = mTimer.get();
      }
    
      void setFullColor(int r, int g, int b) {
        for(int i=0; i<m_ledBuffer.getLength(); i++) {
          m_ledBuffer.setRGB(i, r,g,b);
          //     m_led.setData(m_ledBuffer);
        }
             m_led.setData(m_ledBuffer);
      } 
      int ii = 45;
      void coolShoot(){
        for (int i = 30; i < 59; i++){
          ii++;
          if (ii > 59){
            ii=30;
          }
          m_ledBuffer.setRGB(ii,0,0,0);
          m_ledBuffer.setRGB(((ii*-1)+59), 20,0,0);
          m_ledBuffer.setRGB(i,255,0,0);
          m_ledBuffer.setRGB(((i*-1)+59), 255,0,0);
          m_led.setData(m_ledBuffer);
        }
      }
      
      void shoot(){
        delay = false;
        delayNum = 0.00;
        ii++;
        if (ii > 59){
          ii=30;
        }
        m_ledBuffer.setRGB(ii,0,0,255);
        m_ledBuffer.setRGB(((ii*-1)+59), 0,0,255);
        m_ledBuffer.setRGB(count + 30,0,255,0);
        m_ledBuffer.setRGB((((count + 30)*-1)+59), 0,255,0);
        m_led.setData(m_ledBuffer);
        count++;
        if (count > 29){
          count = 0;
        }
      }
    
      int in = 15;
      void coolIntake(){
        for (int i = 59; i > 30; i--){
          in++;
          if (in > 28){
            in=0;
          }
          m_ledBuffer.setRGB(in,255,0,0);
          m_ledBuffer.setRGB(((in*-1)+59), 255,0,0);
          m_ledBuffer.setRGB(i,255,125,0);
          m_ledBuffer.setRGB(((i*-1)+59), 255,125,0);
          m_led.setData(m_ledBuffer);
        }
      }
    
      void intake(){
        delay = true;
        delayNum = 0.02;
        in++;
        if (in > 28){
          in=0;
        }
        m_ledBuffer.setRGB(in,255,0,0);
        m_ledBuffer.setRGB(((in*-1)+59), 25,0,0);
        m_ledBuffer.setRGB(count,255,125,0);
        m_ledBuffer.setRGB(((count*-1)+59), 255,125,0);
        m_led.setData(m_ledBuffer);
        count--;
        if (count < 30){
          count = 58;
        }
      }
    
      void intakeBall(){
        delay = true;
        delayNum = 0.02;
        in++;
        if (in > 28){
          in=0;
        }
        m_ledBuffer.setRGB(in,0,0,255);
        m_ledBuffer.setRGB(((in*-1)+59), 0,0,255);
        m_ledBuffer.setRGB(count,200,125,0);
        m_ledBuffer.setRGB(((count*-1)+59), 200,125,0);
        m_led.setData(m_ledBuffer);
      count--;
      if (count < 31){
        count = 59;
      }
    }
    
      void ballCount(int ball){
        count++;
      if(count>12*ball){
            count=0;
      }
      if (count<12*ball){
        if (count<12){
          m_ledBuffer.setRGB(count, 255,0,0);
        }else if(count<24){
          m_ledBuffer.setRGB(count, 0,0,255);
        }else if(count<36){
          m_ledBuffer.setRGB(count, 255,0,125);
        }else if(count<48){
          m_ledBuffer.setRGB(count, 255,125,0);
        }else if(count<60){
          m_ledBuffer.setRGB(count, 0,0,255);
        }
      }
    
        m_led.setData(m_ledBuffer);
      }
    
      boolean loopNum = true;
    
      void fullBroken(int length){
      delay = false;
      if (loopNum == true){
        count++;
        if (count > 58){
          loopNum = false;
          count = 0;
        }
        if ((count/length)%2 == 0){
          m_ledBuffer.setRGB(count ,0,255,0);
        } else {
          m_ledBuffer.setRGB(count ,0,0,0);
        }
      }
      m_led.setData(m_ledBuffer);
      if (loopNum == false){
        count++;
        if (count > 58){
          loopNum = true;
          count = 0;
        }
        if ((count/length)%2 == 0){
          m_ledBuffer.setRGB(count ,0,0,0);
        } else {
          m_ledBuffer.setRGB(count ,0,255,0);
        }
      }
      m_led.setData(m_ledBuffer);
    }
    
      void climb(){
        setFullColor(64,100,25);
      }
    
      void full(){
        delayNum = 0.05;
        delay = true;
        count++;
        if (count > 2){
          count = 0;
        }
        if (count == 1){
        m_ledBuffer.setRGB(0,0,0,0);
        m_ledBuffer.setRGB(1,0,0,0);
        m_ledBuffer.setRGB(2,0,0,0);
        m_ledBuffer.setRGB(3,0,0,0);
        m_ledBuffer.setRGB(4,0,0,0);
        m_ledBuffer.setRGB(5,0,255,0);
        m_ledBuffer.setRGB(6,0,255,0);
        m_ledBuffer.setRGB(7,0,255,0);
        m_ledBuffer.setRGB(8,0,255,0);
        m_ledBuffer.setRGB(9,0,255,0);
        m_ledBuffer.setRGB(10,0,0,0);
        m_ledBuffer.setRGB(11,0,0,0);
        m_ledBuffer.setRGB(12,0,0,0);
        m_ledBuffer.setRGB(13,0,0,0);
        m_ledBuffer.setRGB(14,0,0,0);
        m_ledBuffer.setRGB(15,0,255,0);
        m_ledBuffer.setRGB(16,0,255,0);
        m_ledBuffer.setRGB(17,0,255,0);
        m_ledBuffer.setRGB(18,0,255,0);
        m_ledBuffer.setRGB(19,0,255,0);
        m_ledBuffer.setRGB(20,0,0,0);
        m_ledBuffer.setRGB(21,0,0,0);
        m_ledBuffer.setRGB(22,0,0,0);
        m_ledBuffer.setRGB(23,0,0,0);
        m_ledBuffer.setRGB(24,0,0,0);
        m_ledBuffer.setRGB(25,0,255,0);
        m_ledBuffer.setRGB(26,0,255,0);
        m_ledBuffer.setRGB(27,0,255,0);
        m_ledBuffer.setRGB(28,0,255,0);
        m_ledBuffer.setRGB(29,0,255,0);
        m_ledBuffer.setRGB(30,0,0,0);
        m_ledBuffer.setRGB(31,0,0,0);
        m_ledBuffer.setRGB(32,0,0,0);
        m_ledBuffer.setRGB(33,0,0,0);
        m_ledBuffer.setRGB(34,0,0,0);
        m_ledBuffer.setRGB(35,0,255,0);
        m_ledBuffer.setRGB(36,0,255,0);
        m_ledBuffer.setRGB(37,0,255,0);
        m_ledBuffer.setRGB(38,0,255,0);
        m_ledBuffer.setRGB(39,0,255,0);
        m_ledBuffer.setRGB(40,0,0,0);
        m_ledBuffer.setRGB(41,0,0,0);
        m_ledBuffer.setRGB(42,0,0,0);
        m_ledBuffer.setRGB(43,0,0,0);
        m_ledBuffer.setRGB(44,0,0,0);
        m_ledBuffer.setRGB(45,0,255,0);
        m_ledBuffer.setRGB(46,0,255,0);
        m_ledBuffer.setRGB(47,0,255,0);
        m_ledBuffer.setRGB(48,0,255,0);
        m_ledBuffer.setRGB(49,0,255,0);
        m_ledBuffer.setRGB(50,0,0,0);
        m_ledBuffer.setRGB(51,0,0,0);
        m_ledBuffer.setRGB(52,0,0,0);
        m_ledBuffer.setRGB(53,0,0,0);
        m_ledBuffer.setRGB(54,0,0,0);
        m_ledBuffer.setRGB(55,0,255,0);
        m_ledBuffer.setRGB(56,0,255,0);
        m_ledBuffer.setRGB(57,0,255,0);
        m_ledBuffer.setRGB(58,0,255,0);
        m_ledBuffer.setRGB(59,0,255,0);
        }
        if (count == 2){
          m_ledBuffer.setRGB(0,0,255,0);
          m_ledBuffer.setRGB(1,0,255,0);
          m_ledBuffer.setRGB(2,0,255,0);
          m_ledBuffer.setRGB(3,0,255,0);
          m_ledBuffer.setRGB(4,0,255,0);
          m_ledBuffer.setRGB(5,0,0,0);
          m_ledBuffer.setRGB(6,0,0,0);
          m_ledBuffer.setRGB(7,0,0,0);
          m_ledBuffer.setRGB(8,0,0,0);
          m_ledBuffer.setRGB(9,0,0,0);
          m_ledBuffer.setRGB(10,0,255,0);
          m_ledBuffer.setRGB(11,0,255,0);
          m_ledBuffer.setRGB(12,0,255,0);
          m_ledBuffer.setRGB(13,0,255,0);
          m_ledBuffer.setRGB(14,0,255,0);
          m_ledBuffer.setRGB(15,0,0,0);
          m_ledBuffer.setRGB(16,0,0,0);
          m_ledBuffer.setRGB(17,0,0,0);
          m_ledBuffer.setRGB(18,0,0,0);
          m_ledBuffer.setRGB(19,0,0,0);
          m_ledBuffer.setRGB(20,0,255,0);
          m_ledBuffer.setRGB(21,0,255,0);
          m_ledBuffer.setRGB(22,0,255,0);
          m_ledBuffer.setRGB(23,0,255,0);
          m_ledBuffer.setRGB(24,0,255,0);
          m_ledBuffer.setRGB(25,0,0,0);
          m_ledBuffer.setRGB(26,0,0,0);
          m_ledBuffer.setRGB(27,0,0,0);
          m_ledBuffer.setRGB(28,0,0,0);
          m_ledBuffer.setRGB(29,0,0,0);
          m_ledBuffer.setRGB(30,0,255,0);
          m_ledBuffer.setRGB(31,0,255,0);
          m_ledBuffer.setRGB(32,0,255,0);
          m_ledBuffer.setRGB(33,0,255,0);
          m_ledBuffer.setRGB(34,0,255,0);
          m_ledBuffer.setRGB(35,0,0,0);
          m_ledBuffer.setRGB(36,0,0,0);
          m_ledBuffer.setRGB(37,0,0,0);
          m_ledBuffer.setRGB(38,0,0,0);
          m_ledBuffer.setRGB(39,0,0,0);
          m_ledBuffer.setRGB(40,0,255,0);
          m_ledBuffer.setRGB(41,0,255,0);
          m_ledBuffer.setRGB(42,0,255,0);
          m_ledBuffer.setRGB(43,0,255,0);
          m_ledBuffer.setRGB(44,0,255,0);
          m_ledBuffer.setRGB(45,0,0,0);
          m_ledBuffer.setRGB(46,0,0,0);
          m_ledBuffer.setRGB(47,0,0,0);
          m_ledBuffer.setRGB(48,0,0,0);
          m_ledBuffer.setRGB(49,0,0,0);
          m_ledBuffer.setRGB(50,0,255,0);
          m_ledBuffer.setRGB(51,0,255,0);
          m_ledBuffer.setRGB(52,0,255,0);
          m_ledBuffer.setRGB(53,0,255,0);
          m_ledBuffer.setRGB(54,0,255,0);
          m_ledBuffer.setRGB(55,0,0,0);
          m_ledBuffer.setRGB(56,0,0,0);
          m_ledBuffer.setRGB(57,0,0,0);
          m_ledBuffer.setRGB(58,0,0,0);
          m_ledBuffer.setRGB(59,0,0,0);
        }
      }
    
    
      
    
      private void fixedColor(int r, int g, int b) {
        // For every pixel
        int window = count + 4;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          if ((i >= count) && (i < window)) { 
            m_ledBuffer.setRGB(i, 120, 0, 200);
          } else {
            m_ledBuffer.setRGB(i, r, g, b);
          }
        }
        count++;
        count %= m_ledBuffer.getLength();
      }
    
      private void teamColors(int winSize) {
        int bufLen = m_ledBuffer.getLength();
        int winMin;
        int winMax;
    
        if (count < winSize) {
          winMin = 0;
        } else {
          winMin = (count - winSize) % bufLen;
        }  
        if (count < winSize) {
          winMax = count;
        } else {
          winMax = winMin + winSize;
          if (winMax > bufLen) winMax = bufLen;
        }
    
      // For every pixel
        for (var i = 0; i < bufLen; i++) {
          if ((i >= winMin) && (i < winMax)) { 
            m_ledBuffer.setRGB(i, 255, 255, 0);
          } else {
            m_ledBuffer.setRGB(i, 0, 0, 200);
          }
        }
        count++;
        count %= (bufLen + winSize);
      }
    
      private void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
          //m_ledBuffer.s
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }
    
      public void option(OpConstants.LedOption select) {
        if (length < 60){
          return;
        }
        synchronized(mLedOption){
          mLedOption = select;
        }
        // Fill the buffer with selection
        switch (mLedOption) {
          case TEAM:
            teamColors(16);
            break;
          case RED:
            fixedColor(255, 0, 0);
            break;
          case BLUE:
            fixedColor(0, 0, 255);
            break;
          case GREEN:
            fixedColor(0, 200, 0);
            break;
          case YELLOW:
            fixedColor(255, 255, 0);
            break;
          case ORANGE:
            fixedColor(255, 30, 0);
            break;
          case PURPLE:
            fixedColor(150, 0, 150);
            break;
          case RAINBOW:
            rainbow(); // Fill the buffer with a rainbow
            break;
          case FULL:
            full();
            break;
          case CLIMB:
            climb();
            break;
          case SHOOT:
            shoot();
            break;
          case INTAKE:
            intake();
            break;
          case INTAKEBALL:
            intakeBall();
            break;
          case BALLONE:
            ballCount(1);
            break;
          case BALLTWO:
            ballCount(2);
            break;
          case BALLTHREE:
            ballCount(3);
            break;
          case BALLFOUR:
            ballCount(4);
            break;
          default:
            teamColors(16);
            break;
        }
        // Set the LEDs
        m_led.setData(m_ledBuffer);
      }
}

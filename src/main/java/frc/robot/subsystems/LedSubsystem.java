package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LED.modes;

import edu.wpi.first.wpilibj.Filesystem;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class LedSubsystem extends SubsystemBase {

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int counter = 0;
  private int row = 0;

  private int m_pacer = 1;
  private modes m_mode;

  private int m_rainbowFirstPixelHue;

  BufferedImage badapple;
  BufferedImage heatgradient;
  BufferedImage whitedotlines; 

  BufferedImage currentimage; 

  public LedSubsystem() {
    m_led = new AddressableLED(Constants.LED.LED_PWM);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LED.LED_LENGTH);

    m_mode = modes.Rainbow;

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    badapple = imageLoad("images/badapple.png");
    heatgradient = imageLoad("images/heatgradient.png");
    whitedotlines = imageLoad("images/whitedotlines.png");

  }

  private double sin_wave(double val, int set_point) {
    return Math.sin(Math.PI * 2 * val / 70) * 70 + set_point;
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void red() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 0, 0);
    }
  }

  private void orange() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 255, 50, 0);
    }
  }

  private void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 128, 0);
    }
  }

  private void blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 128);
    }
  }

  private void yellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 128, 0);
    }
  }

  private void purple() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 0, 128);
    }
  }

  private void off() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void breathing_monochrome(int hue) {
    int sat = 0;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      sat = (int) sin_wave((double) i + counter, 200);
      m_ledBuffer.setHSV(i, hue, 255, sat);
    }
    counter++;
  }

  private void flashing_orange() {
    int phase = (counter / 2) % 2;
    int red = phase * 255;
    int green = phase * 50;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, 0);
    }
    counter++;
  }
  
  private void flashing_green() {
    int phase = (counter / 2) % 2;
    int red = phase * 0;
    int green = phase * 255;
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, 0);
    }
    counter++;
  }

  private BufferedImage imageLoad(String imagePath) {
    try {
      return ImageIO.read(new File(Filesystem.getDeployDirectory(), imagePath));
    } catch (IOException e) {
      e.printStackTrace();
      // make robust 
    }
    return null;
  }

  private void imageLoop() {
    if (currentimage == null) {
      return;
    }

    int width = Math.min(m_ledBuffer.getLength(), currentimage.getWidth());

    for (var i = 0; i < width; i++) {
      int pixel = currentimage.getRGB(i, row);
      int r = (pixel >> 16) & 0xff;
      int g = (pixel >> 8) & 0xff;
      int b = (pixel >> 0) & 0xff;
      m_ledBuffer.setRGB(i, r, g, b);
    }

    if (++row >= currentimage.getHeight()) {
      row = 0;
    }
  }

/**
 * <p>Set the LED mode to a specific color</p>
 * @param mode the mode to set the LED to
 * @param optionalOverride whether to override the current mode, defaults to false
 */
  public void setMode(modes mode, boolean... optionalOverride) {
    // TODO: Override does not do anything
    // optionalOverride defaults to false
    if (m_mode != mode) {
      if (optionalOverride.length > 0) {
        m_mode = mode;
        counter = 0;
        return;
      }
      m_mode = mode;
      counter = 0;
    }
    //System.out.println("Set LED to: " + mode);
  }

  public modes getMode() {
    return m_mode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_pacer == 3) {
      switch (m_mode) {
        case Green:
          green();
          break;
        case Red:
          red();
          break;
        case Orange:
          orange();
          break;
        case Blue:
          blue();
          break;
        case Yellow:
          yellow();
          break;
        case Purple:
          purple();
          break;
        case Off:
          off();
          break; 
        case BreathingYellow:
          breathing_monochrome(30);
          break;
        case BreathingMagenta:
          breathing_monochrome(150);
          break;
        case FlashingOrange:
          flashing_orange();
          break;
        case FlashingGreen:
          flashing_green();
          break;
        case badApple:
          currentimage = badapple;
          imageLoop();
          break;
        case heatGradient:
          currentimage = heatgradient;
          imageLoop();
          break;
        case whiteDotLines:
          currentimage = whitedotlines;
          imageLoop();
          break;
        default:
          rainbow();
      }
      m_pacer = -1;
    }
    // Set the LEDs
    m_led.setData(m_ledBuffer);
    m_pacer++;
  }
}

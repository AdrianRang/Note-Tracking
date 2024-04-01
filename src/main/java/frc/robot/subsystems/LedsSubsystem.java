package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.team3526.constants.CTRECANDevice;
import lib.team3526.led.LEDStrip;

public class LedsSubsystem extends SubsystemBase {
    private final CANdle leds;
    private final CANdleConfiguration config;

    public LedsSubsystem(CTRECANDevice device) {
        this.leds = new CANdle(device.getDeviceID(), device.getCanbus());
        this.config = new CANdleConfiguration();
        this.config.stripType = Constants.CANdle.kLEDStripType;
        this.config.brightnessScalar = Constants.CANdle.kLEDBrightness;
    }

    /**
     * Sets the color of the LEDs to the specified hex color
     * @param hex The hex color to set the LEDs to in String format
     */
    public void setLeds(String hex) {
        int[] rgb = hexToRGB(hex);
        leds.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    /**
     * Sets the color of the LEDs to the specified RGB values
     * @param r The red value to set the LEDs to
     * @param g The green value to set the LEDs to
     * @param b The blue value to set the LEDs to
     */
    public void setLeds(int r, int g, int b) {
        leds.setLEDs(r, g, b);
    }

    /**
     * Sets the color of a segment of the LEDs to the specified hex color
     * @param strip The LEDStrip object representing the segment of LEDs to set
     * @param hex The hex color to set the LEDs to in String format
     */
    public void setLedSegment(LEDStrip strip, String hex) {
        int[] rgb = hexToRGB(hex);
        leds.setLEDs(rgb[0], rgb[1], rgb[2], 255, strip.getInitialIndex(), strip.getLength());
    }

    /**
     * Sets the color of a segment of the LEDs to the specified RGB values
     * @param strip The LEDStrip object representing the segment of LEDs to set
     * @param r The red value to set the LEDs to
     * @param g The green value to set the LEDs to
     * @param b The blue value to set the LEDs to
     */
    public void setLedSegment(LEDStrip strip, int r, int g, int b) {
        leds.setLEDs(r, g, b, 255, strip.getInitialIndex(), strip.getLength());
    }

    /**
     * Blinks the LEDs with the specified hex color
     * @param hex The hex color to blink the LEDs with in String format
     */
    public void blinkLeds(String hex) {
        int[] rgb = hexToRGB(hex);
        blinkLeds(rgb[0], rgb[1], rgb[2]);
    }

    /**
     * Blinks the LEDs with the specified RGB values
     * @param r The red value to blink the LEDs with
     * @param g The green value to blink the LEDs with
     * @param b The blue value to blink the LEDs with
     */
    public void blinkLeds(int r, int g, int b) {
        leds.animate(new StrobeAnimation(r, g, b));
    }

    /**
     * Blinks the LEDs with the specified hex color
     * @param strip The LEDStrip object representing the segment of LEDs to blink
     * @param hex The hex color to blink the LEDs with in String format
     * @param speed The speed of the blink
     */
    public void blinkLedSegment(LEDStrip strip, String hex, int speed) {
        int[] rgb = hexToRGB(hex);
        leds.animate(new StrobeAnimation(rgb[0], rgb[1], rgb[2], 255, speed, strip.getLength(), strip.getInitialIndex()));
    }

    /**
     * Blinks the LEDs with the specified RGB values
     * @param strip The LEDStrip object representing the segment of LEDs to blink
     * @param r The red value to blink the LEDs with
     * @param g The green value to blink the LEDs with
     * @param b The blue value to blink the LEDs with
     * @param speed The speed of the blink
     */
    public void blinkLedSegment(LEDStrip strip, int r, int g, int b, int speed) {
        leds.animate(new StrobeAnimation(r, g, b, 255, speed, strip.getLength(), strip.getInitialIndex()));
    }

    public void turnOff() {
        leds.setLEDs(0, 0, 0);
    }

    /**
     * Converts a hex color to an RGB array (The hex must be in the format #RRGGBB)
     * @param hex The hex color to convert in String format
     * @return An array of 3 integers representing the RGB values
     */
    private int[] hexToRGB(String hex) {
        int[] rgb = new int[3];
        rgb[0] = Integer.parseInt(hex.substring(1, 3), 16);
        rgb[1] = Integer.parseInt(hex.substring(3, 5), 16);
        rgb[2] = Integer.parseInt(hex.substring(5, 7), 16);
        return rgb;
    }
}

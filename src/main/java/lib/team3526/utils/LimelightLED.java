package lib.team3526.utils;

import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightLED {
    /**
     * Blink the LEDs on the limelight for a given duration
     * @param limelightName
     */
    public static final void blinkLeds(String limelightName) {
        blinkLeds(limelightName, 1);
    }

    /**
     * Blink the LEDs on the limelight for a given duration
     * @param limelightName
     * @param duration
     */
    public static final void blinkLeds(String limelightName, double duration) {
        new Thread(() -> {
            try {
                LimelightHelpers.setLEDMode_ForceBlink(Constants.Vision.kLimelightName);
                Thread.sleep((int)duration * 1000);
                LimelightHelpers.setLEDMode_ForceOff(Constants.Vision.kLimelightName);
            } catch(Exception e) {}
        }).start();
    }
}

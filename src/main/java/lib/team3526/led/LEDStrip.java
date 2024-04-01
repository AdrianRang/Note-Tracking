package lib.team3526.led;

public class LEDStrip {
    private final int initialIndex;
    private final int finalIndex;
    private final int length;
    
    /**
     * Creates a new LEDStrip object (Sums 8 to the initial and final index to account for the 8 LEDs on the candle)
     * @param initialIndex the first index of the LED strip
     * @param finalIndex the last index of the LED strip
     */
    public LEDStrip(int initialIndex, int finalIndex) {
        this.initialIndex = initialIndex+8;
        this.finalIndex = finalIndex+8;
        this.length = finalIndex - initialIndex + 1;
    }

    public int getInitialIndex() {
        return initialIndex;
    }

    public int getFinalIndex() {
        return finalIndex;
    }

    public int getLength() {
        return length;
    }

}

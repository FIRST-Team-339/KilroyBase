package frc.HardwareInterfaces;

/**
 * A class for the LV-MaxSonar-EZ line of ultrasonic sensors.
 * This specific sensor reads in inches by default.
 * 
 * To identify, these boards are generally a green color
 * 
 * @author Ryan McGee
 *
 */
public class LVMaxSonarEZ extends UltraSonic
{
/**
 * The default scaling factor for inches
 */
private final double DEFAULT_SCALING_FACTOR = .13;

/**
 * Sets up the ultrasonic and sets the default scaling factor.
 * 
 * @param channel
 *            The analog port the ultrasonic is located.
 */
public LVMaxSonarEZ (int channel)
{
    super(channel);
    super.setScalingFactor(DEFAULT_SCALING_FACTOR);
}


}

package frc.HardwareInterfaces;

/**
 * A class for the MB1200 XL-MaxSonar-EZ line of ultrasonic sensors.
 * This specific sensor reads in Inches by default.
 * 
 * To identify, these boards are generally a green color with a small
 * silver box attached to the back.
 * 
 * @author Ryan McGee
 *
 */
public class XLMaxSonarEZ extends UltraSonic
{

/**
 * The working scaling factor for this line of ultrasonics,
 * converting to inches.
 */
private final double DEFAULT_SCALING_FACTOR = 0.1;

/**
 * Sets up the ultrasonic and sets the default scaling factor
 * for this specific model.
 * 
 * @param channel
 *            the analog channel
 */
public XLMaxSonarEZ (int channel)
{
    super(channel);
    super.setScalingFactor(DEFAULT_SCALING_FACTOR);
}

}

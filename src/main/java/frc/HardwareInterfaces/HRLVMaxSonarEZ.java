package frc.HardwareInterfaces;

/**
 * A class for the HRLV-MaxSonar-EZ line of ultrasonic sensors.
 * This specific sensor reads in millimeters by default, but the
 * default scaling factor converts to inches.
 * 
 * To identify, these boards are generally a black color
 * 
 * @author Ryan McGee
 *
 */
public class HRLVMaxSonarEZ extends UltraSonic
{

/**
 * The default scaling factor for millimeters
 */
private final double DEFAULT_SCALING_FACTOR = .05;

/**
 * Sets up the ultrasonic and sets the default scaling factor.
 * 
 * @param channel
 *            The analog port the ultrasonic is located.
 */
public HRLVMaxSonarEZ (int channel)
{
    super(channel);
    super.setScalingFactor(DEFAULT_SCALING_FACTOR);

}

}

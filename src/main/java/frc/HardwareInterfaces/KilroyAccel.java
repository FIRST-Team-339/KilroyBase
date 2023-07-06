package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

/**
 * Accelerometer class that utilizes and builds upon the accelerometer built into the roborio.
 * 
 * @author Ryan McGee
 *
 */
public class KilroyAccel extends BuiltInAccelerometer
{
	/**
	 * Create the accelerometer object, and initialize the position / velocity background thread
	 */
	public KilroyAccel()
	{
		super(Range.k4G);
		this.startCalculationThread();
	}

	/**
	 * Creates the acceleromter object, and initializes the position / velocity background thread
	 * 
	 * @param accelRange The range on the acceleromter: 1G is equal to 32.2 feet per second, 
	 * 			and this decides the max value and sensitivity of the accelerometer.
	 */
	public KilroyAccel(Range accelRange)
	{
		super(accelRange);
	}

	/**
	 * List of different axis' available on the built in accelerometer.
	 * @author Ryan McGees
	 */
	public enum Axis
	{
		X, Y, Z
	}

	/**
	 *  Describes pairs of axis as the three planes, for use in vectors.
	 * @author Ryan McGee
	 */
	public enum AxisPlane
	{
		XY, YZ, ZX
	}

	/**
	 * Begins a thread that continually calculates the velocity and position since last reset.
	 * 
	 * This does so by adding the difference between updates to an original value. 
	 * This is basically a watered down integral of the acceleration, so there will be error.
	 */
	public void startCalculationThread()
	{
		if (calculationThreadStarted == true)
			return;

		this.calculationThread = (new Thread()
		{
			// Store the last values in order to find deltas
			double lastAccelX, lastAccelY, lastAccelZ;
			double lastVelX, lastVelY, lastVelZ;

			// Begin the thread here
			@Override
			public void run()
			{
				while (stopCalculationThread == false)
				{
					// Finding / storing accel values (feet per second squared)
					double x = getX() * GS_TO_FT_PER_SECOND_SQRD, y = getY() * GS_TO_FT_PER_SECOND_SQRD,
							z = getZ() * GS_TO_FT_PER_SECOND_SQRD;

					double deltaAccelX = x - lastAccelX;
					double deltaAccelY = y - lastAccelY;
					double deltaAccelZ = z - lastAccelZ;
					this.lastAccelX = x;
					this.lastAccelY = y;
					this.lastAccelZ = z;

					currentAccelX = x;
					currentAccelY = y;
					currentAccelZ = z;

					// Finding velocity (feet per second)
					currentVelX += deltaAccelX;
					currentVelY += deltaAccelY;
					currentVelZ += deltaAccelZ;

					// Finding position (inches)
					currentPosX += (currentVelX - lastVelX) * 12;
					currentPosY += (currentVelY - lastVelY) * 12;
					currentPosZ += (currentVelZ - lastVelZ) * 12;
				}
				// Right before the thread dies, make it so we can re-enable it
				// later on
				calculationThreadStarted = false;
			}

		});

		this.calculationThread.start();

		calculationThreadStarted = true;
	}

	/**
	 * Tells the thread handling calculations for position / velocity / acceleration to end.
	 */
	public void stopThread()
	{
		this.stopCalculationThread = true;
	}

	/**
	 * Gets the current acceleration on the given axis
	 * @param axis Which axis to pull the acceleration from 
	 * @return the acceleration, in Feet per Second Squared
	 */
	public double getAcceleration(Axis axis)
	{
		switch (axis)
		{
		case X:
			return this.currentAccelX;
		case Y:
			return this.currentAccelY;
		case Z:
			return this.currentAccelZ;
		default:
			return 0;
		}
	}

	/**
	 * Gets the current velocity on a given axis
	 * @param axis Which axis to pull the velocity from 
	 * @return the acceleration, in Feet per Second
	 */
	public double getVelocity(Axis axis)
	{
		switch (axis)
		{
		case X:
			return this.currentVelX;
		case Y:
			return this.currentVelY;
		case Z:
			return this.currentVelZ;
		default:
			return 0;
		}
	}

	/**
	 * Gets the current position on a given axis, relative to the latest reset
	 * @param axis Which axis to pull the position from
	 * @return the position (displacement), in inches
	 */
	public double getPosition(Axis axis)
	{
		switch (axis)
		{
		case X:
			return this.currentPosX;
		case Y:
			return this.currentPosY;
		case Z:
			return this.currentPosZ;
		default:
			return 0;
		}
	}

	/**
	 * @param plane which axis' should the magnitude reflect
	 * @return The magnitude for the give plane (speed)
	 */
	public double getAccelMagnitude(AxisPlane plane)
	{
		switch (plane)
		{
		case XY:
			return Math.sqrt(Math.pow(currentAccelX, 2) + Math.pow(currentAccelY, 2));
		case YZ:
			return Math.sqrt(Math.pow(currentAccelY, 2) + Math.pow(currentAccelZ, 2));
		case ZX:
			return Math.sqrt(Math.pow(currentAccelZ, 2) + Math.pow(currentAccelX, 2));
		default:
			return 0;
		}
	}

	/**
	 * @param plane which axis' should the magnitude reflect
	 * @return the magnitude for the given plane (speed)
	 */
	public double getVelocityMagnitude(AxisPlane plane)
	{
		switch (plane)
		{
		case XY:
			return Math.sqrt(Math.pow(currentVelX, 2) + Math.pow(currentVelY, 2));
		case YZ:
			return Math.sqrt(Math.pow(currentVelY, 2) + Math.pow(currentVelZ, 2));
		case ZX:
			return Math.sqrt(Math.pow(currentVelZ, 2) + Math.pow(currentVelX, 2));
		default:
			return 0;
		}
	}

	/**
	 * @param plane which axis' should the magnitude reflect
	 * @return the magnitude for the given plane (speed)
	 */
	public double getPositionMagnitude(AxisPlane plane)
	{
		switch (plane)
		{
		case XY:
			return Math.sqrt(Math.pow(currentPosX, 2) + Math.pow(currentPosY, 2));
		case YZ:
			return Math.sqrt(Math.pow(currentPosY, 2) + Math.pow(currentPosZ, 2));
		case ZX:
			return Math.sqrt(Math.pow(currentPosZ, 2) + Math.pow(currentPosX, 2));
		default:
			return 0;
		}
	}

	/**
	 * @param plane which axis' should the direction reflect
	 * @return the direction of the given plane (angle)
	 */
	public double getAccelDirection(AxisPlane plane)
	{
		switch (plane)
		{
		case XY:
			return Math.atan2(currentAccelY, currentAccelX);
		case YZ:
			return Math.atan2(currentAccelZ, currentAccelY);
		case ZX:
			return Math.atan2(currentAccelX, currentAccelZ);
		default:
			return 0;
		}
	}

	/**
	 * @param plane which axis' should the direction reflect
	 * @return the direction of the given plane (angle)
	 */
	public double getVelocityDirection(AxisPlane plane)
	{
		switch (plane)
		{
		case XY:
			return Math.atan2(currentVelY, currentVelX);
		case YZ:
			return Math.atan2(currentVelZ, currentVelY);
		case ZX:
			return Math.atan2(currentVelX, currentVelZ);
		default:
			return 0;
		}
	}

	/**
	 * @param plane which axis' should the direction reflect
	 * @return the direction of the given plane (angle)
	 */
	public double getPositionDirection(AxisPlane plane)
	{
		switch (plane)
		{
		case XY:
			return Math.atan2(currentPosY, currentPosX);
		case YZ:
			return Math.atan2(currentPosZ, currentPosY);
		case ZX:
			return Math.atan2(currentPosX, currentPosZ);
		default:
			return 0;
		}
	}

	// ================Variables================

	private Thread calculationThread;

	private boolean calculationThreadStarted = false;

	private volatile boolean stopCalculationThread = false;

	private volatile double currentAccelX, currentAccelY, currentAccelZ;

	private volatile double currentVelX, currentVelY, currentVelZ;

	private volatile double currentPosX, currentPosY, currentPosZ;

	// ================Constants================

	private final double GS_TO_FT_PER_SECOND_SQRD = 32.2;

}

package frc.HardwareInterfaces.Transmission;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Contains necessary functions that must be included in each transmission type
 * class created.
 *
 * @author Ryan McGee
 * @written 7/17/2017
 */
public abstract class TransmissionBase
{

/**
 * Creates the TransmissionBase object with a 2 wheel drive system. Should only
 * be created by sub classes, hence the abstract label.
 *
 * @param leftMotor
 * @param rightMotor
 */
public TransmissionBase (MotorController leftMotor,
        MotorController rightMotor)
{
    this.motors = new MotorController[2];
    this.motors[0] = leftMotor;
    this.motors[1] = rightMotor;
}

/**
 * Creates the TransmissionBase object. Should only be called by any subclasses,
 * hence the abstract label.
 *
 * @param leftRear
 * @param rightRear
 * @param leftFront
 * @param rightFront
 */
public TransmissionBase (MotorController leftRear,
        MotorController rightRear, MotorController leftFront,
        MotorController rightFront)
{
    this.motors = new MotorController[4];
    this.motors[0] = leftRear;
    this.motors[1] = rightRear;
    this.motors[2] = leftFront;
    this.motors[3] = rightFront;
}

/**
 * Turns off the deadband for use in auto.
 */
public void disableDeadband ()
{
    currentJoystickDeadband = 0;
}

/**
 * Removes one from the current gear of the robot, allowing the user to drive
 * slower.
 */
public void downShift ()
{
    if (currentGear > 0)
        currentGear--;
}

/**
 * Drives the transmission based on a Tank drive system, but without gear ratios
 * or joystick deadbands. Use for autonomous purposes.
 *
 * @param leftVal
 *                     The left value of the robot, in percentage (-1.0 to 1.0)
 * @param rightVal
 *                     The right value of the robot, in percentage (-1.0 to 1.0)
 */
public void driveRaw (double leftVal, double rightVal)
{
    for (int i = 0; i < this.motors.length; i++)
        if (i % 2 == 0)
            this.motors[i].set(leftVal);
        else
            this.motors[i].set(rightVal);
}

/**
 * Drives the robot based on raw inputs, for autonomous uses. Also, can use a
 * correction PID loop for rotation, if that is enabled. (Functionality
 * overridden in MecanumTransmission class)
 *
 * @param magnitude
 *                      How fast the robot will travel (0.0 to 1.0)
 * @param direction
 *                      In which direction, laterally, will the robot travel
 *                      (degrees, -180 to 180. 0 is forward.)
 * @param rotation
 *                      How much the robot should be turning (left,(-1.0) to
 *                      right,(1.0)
 */
public void driveRaw (double magnitude, double direction,
        double rotation)
{
    this.stop();
    // If this object is an omni-directional drive and this method is called,
    // it will be overridden by the superclass. This prevents a tank style
    // transmission from forcing the motors against each other, as it will
    // by default to do nothing.
}

/**
 * Turns on the deadband for use in teleop.
 */
public void enableDeadband ()
{
    currentJoystickDeadband = inputJoystickDeadband;
}

/**
 * @return all speed controllers attached to this transmission, as an array.
 */
public MotorController[] getAllMotorControllers ()
{
    return this.motors;
}

/**
 * @return The gear number that is active
 */
public int getCurrentGear ()
{
    return this.currentGear;
}

/**
 * @return The percentage corresponding to the current gear
 */
public double getCurrentGearRatio ()
{
    return gearRatios[currentGear];
}

/**
 * @param position
 *                     which corner the motor is in
 * @return the motor controller object
 */
public MotorController getMotorController (MotorPosition position)
{
    switch (position)
        {
        // Left is the same as left rear
        case LEFT:
        case LEFT_REAR:
            return this.motors[0];
        // Right is the same as right rear
        case RIGHT:
        case RIGHT_REAR:
            return this.motors[1];
        case LEFT_FRONT:
            return this.motors[2];
        case RIGHT_FRONT:
            return this.motors[3];
        default:
            return null;
        }

}

/**
 * @return The type of transmission of a class extending TransmissionBase.
 */
public TransmissionType getType ()
{
    return type;
}

/**
 * Uses the formula for mapping one set of values to the other: y = mx + b
 *
 * m = 1 / (1 - deadband) b = deadband * -m x = joystick input y = motor output
 *
 * Therefore, motor output = (1 / (1 - deadband)) * joystick input + (1 - (1 /
 * (1 - deadband)))
 *
 * If this equation does not make much sense, try graphing it first as the
 * original x = y, and then the scaled output starting at the deadband, and use
 * the slope formula.
 *
 * @param input
 * @return The scaled value, if between -1 and -deadband or deadband and 1, or 0
 *         if between -deadband and deadband.
 */
public double scaleJoystickForDeadband (double input)
{
    double deadbandSlope = 1.0 / (1.0 - currentJoystickDeadband);
    double constant = -this.currentJoystickDeadband * deadbandSlope;

    if (input > this.currentJoystickDeadband)
        return (deadbandSlope * input) + constant;
    else
        if (input < -this.currentJoystickDeadband)
            return -((-deadbandSlope * input) + constant);

    // Set to 0 if it is between the deadbands.
    return 0.0;
}

/**
 * Sets every gear ratio. Make sure that the lowest gear starts at 0, and the
 * highest gear is at the max, to make sure the up-shifting and down-shifting
 * works properly.
 *
 * @param ratios
 *                   Percent multiplied by the transmission.drive functions
 */
public void setAllGearPercentages (double... ratios)
{
    this.gearRatios = ratios;
}

/**
 * TODO Test gear system Sets the current gear for the robot. This will change
 * the maximum speed of the robot for precise aiming/driving.
 *
 * @param gear
 *                 The requested gear number. If outside the range, it will do
 *                 nothing.
 */
public void setGear (int gear)
{
    if (gear >= 0 && gear < gearRatios.length)
        this.currentGear = gear;
}

/**
 * Sets the percent multiplied by Transmission.
 *
 * @param gear
 *                  Which gear should be changed: 0 is lowest, increasing.
 * @param value
 *                  Percent decimal form: between 0 and 1.0
 */
public void setGearPercentage (int gear, double value)
{
    if (value < 1 && value > 0 && gear < gearRatios.length && gear >= 0)
        {
        gearRatios[gear] = value;
        }
}

/**
 * TODO test deadbands Sets the minimum value the joysticks must output in order
 * for the robot to start moving.
 *
 * @param deadband
 *                     Percentage value, ranging from 0.0 to 1.0, in decimals.
 */
public void setJoystickDeadband (double deadband)
{
    this.inputJoystickDeadband = deadband;
    this.enableDeadband();
}

/**
 * Sets the maximum gear to the value input.
 *
 * @param value
 *                  Percent (0.0 to 1.0)
 */
public void setMaxGearPercentage (double value)
{
    this.gearRatios[gearRatios.length - 1] = value;
}

/**
 * Sets the robot to the maximum gear available
 *
 */
public void setToMaxGear ()
{
    this.currentGear = gearRatios.length - 1;
}

/**
 * Shift gears using a up-shift and down-shift button. Also makes sure that
 * holding the button will not trigger multiple shifts.
 *
 * @param upShiftButton
 *                            The button that should change to the next higher
 *                            gear
 * @param downShiftButton
 *                            The button that should change to the next lowest
 *                            gear
 */
public void shiftGears (boolean upShiftButton, boolean downShiftButton)
{
    // Makes sure that if the button is held down, it doesn't constantly
    // cycle through gears.
    if (downShiftButton && !downShiftButtonStatus)
        {
        downShift();

        }
    else
        if (upShiftButton && !upShiftButtonStatus)
            {
            upShift();

            }

    upShiftButtonStatus = upShiftButton;
    downShiftButtonStatus = downShiftButton;
}

/**
 * Tells the robot to cut all power to the motors.
 */
public void stop ()
{
    for (MotorController sc : motors)
        sc.set(0);
}

/**
 * Adds one to the current gear of the robot, allowing the user to drive faster.
 */

/*
 * ====================================================================== To
 * (Hopefully) fix McGee's code, uncomment the gearArrayInit method. Comment out
 * the declaration that declares gearRatios with set values, and uncomment the
 * declaration with no set values. THIS HAS NOT BEEN TESTED!!! There is a
 * workaround in teleopDrive method, but this is a possible permanent fix.
 * =====================================================================
 */
// public void gearArrayInit(int numberOfGears)
// {
// double[] gearRatios;
// gearRatios = new double[numberOfGears];
// }

public void upShift ()
{
    if (currentGear < gearRatios.length - 1)
        currentGear++;
}

// =========================================

/**
 * Describes which corner a motor is in when identifying it.
 *
 * @author Ryan McGee
 */
public enum MotorPosition
    {
    /** the left side (if two wheel drive) or left rear (if four wheel drive) */
    LEFT,
    /**
     * the right side (if two wheel drive) or right rear (if four wheel drive)
     */
    RIGHT,
    /** the left front wheel */
    LEFT_FRONT,
    /** the left rear wheel */
    LEFT_REAR,
    /** the right front wheel */
    RIGHT_FRONT,
    /** the right rear wheel */
    RIGHT_REAR,
    /** all motor positions, not used in getMotorController. */
    ALL
    }

/**
 * The current types of transmissions available.
 *
 * @author Ryan McGee
 *
 */
public enum TransmissionType
    {
    /**
     * Tank-style drive system with a left drive, and a right drive.
     */
    TANK,
    /**
     * Omni-Directional Style of drive train Mecanum / Holonomic or Swerve Drive
     * where each motor is independent, and enables the robot to move laterally,
     * forwards/backwards, and rotate.
     */
    OMNI_DIR
    }

// ================VARIABLES================
// The current stored transmission type
TransmissionType type = null;

private double[] gearRatios =
    {.6, .8, 1};
// public static double[] gearRatios;

private final MotorController[] motors;

// Will default to the highest gear available
private int currentGear = 0;

// private int numberOfGears = 2;

// private int maxGears = 2;

private boolean upShiftButtonStatus = false;

private boolean downShiftButtonStatus = false;

private double inputJoystickDeadband = DEFAULT_JOYSTICK_DEADBAND;

// The motors will start turning only once the joystick is past this
// deadband.
private double currentJoystickDeadband = inputJoystickDeadband;

// ================CONSTANTS================
/** the default deadband applied to all joysticks used in drive methods */
public static final double DEFAULT_JOYSTICK_DEADBAND = .2;
// =========================================
}


// ====================================================================
// FILE NAME: KilroyServo.java (Team 339 - Kilroy)
//
// CREATED ON: sometime during 2017 build season
// CREATED BY: Becky BUtton
// MODIFIED ON:2/28/17
// MODIFIED BY: Ashley Espeland
// ABSTRACT:
// Sets up the KilroyServo by inputting the port number and maxDegrees
// includes the code to get the servo angle in degrees(.getAngle()), get
// the max degree(.getMaxdegrees()), set the servo angle in
// degrees(.setAngle()),
// set the max degree value (.setMaxDegrees)
//
package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.Servo;

/**
 * kilroyServo extends Servo and includes the functions
 * involving the Servo
 * 
 * @author Becky Button
 *
 */
public class KilroyServo extends Servo
{

/**
 * Kilroy Servo
 * 
 * @param portNumber
 *            the port the servo is on
 * @param maxDegrees
 *            the maximum degrees the servo can go
 * @author Becky Button
 */
public KilroyServo (final int portNumber, final double maxDegrees)
{
    // sets the port number to the one set in KilroyServo
    super(portNumber);
    // sets up shorthand where this.maxDegrees is the same as maxDegrees
    this.setMaxDegrees(maxDegrees);
} // end constructor()

/**
 * scales raw value to degrees
 * 
 * @return Angle value in degrees
 *         the degrees that the servo is at currently
 */
//
public double getAngle ()
{
    // gets angle value thats already scaled from 0-1, and then scales it to
    // degrees by multiplying it by the maxDegrees
    return (super.get() * this.getMaxDegrees()); // scaled value for
} // end getAngle()



/**
 * Scales raw set value to degrees
 * 
 * @param degree
 *            you want to set the servo
 */
// sets angle of the servo
public void setAngle (final double degree)
{
    // divides the setAngle value by the maxDegrees to scale it from 0 to 1
    // sets the angle using the set() from KilroyServo
    super.set(degree / this.getMaxDegrees()); // scaled value for servo
} // end setAngle()


/**
 * sets .setMaxDegrees to the variable maxDegrees
 * 
 * @param degree
 *            the degree position you want the servo at
 * 
 * @author Ashley Espeland
 */
public void setMaxDegrees (final double degree)
{
    // sets .setMaxDegrees to the variable maxDegrees
    this.maxDegrees = degree;
}

/**
 * returns the max degrees as defined in the variable maxDegrees
 * 
 * @return maxDegrees
 * 
 * @author Ashley Espeland
 */
private double getMaxDegrees ()
{
    // returns maxDegrees for .getMaxDegrees()
    return maxDegrees;
}

// sets maxDegrees equal to 0
private double maxDegrees = 0; // max degrees servo can go
} // end class KilroyServo

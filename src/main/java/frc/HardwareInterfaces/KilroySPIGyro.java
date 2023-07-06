
// ====================================================================
// FILE NAME: KilroyGyro.java (Team 339 - Kilroy)
//
// CREATED ON: sometime during 2017 build season
// CREATED BY: Becky BUtton
// MODIFIED ON:2/28/17
// MODIFIED BY: Ashley Espeland
// ABSTRACT:
// sets up and declares a gyro, includes various functions
// checks to see if we have a gyro and if we do, we declare it, if we
// dont then we print that to the screen, and if neither we return null
// calibration, reset, getAngle (in degrees), and getRate functions
// are included
//

package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * sets up the gyro and includes the functions
 *
 * @author Becky Button
 *
 */
public class KilroySPIGyro extends ADXRS450_Gyro
{
private final ADXRS450_Gyro gyro;

public boolean hasGyro;

/**
 * Creates the Gyro object. If we input that the gyro is not connected, then do
 * not create it to avoid errors, and set the methods to avoid
 * nullPointerExceptions.
 *
 * @param hasGyro
 *                    Whether or not the gyro is connected.
 */
public KilroySPIGyro (boolean hasGyro)
{
    this.hasGyro = hasGyro;
    if (hasGyro)
        {
        this.gyro = new ADXRS450_Gyro();
        }
    else
        {
        System.out.println("***Gyro is NOT enabled!***");
        this.gyro = null;
        }
}

/**
 * calibration of the gyro
 */
public void calibrate ()
{
    // if we do not have a gyro
    if (this.hasGyro() == false)
        {
        System.out.println("***Gyro is NOT enabled!***");
        return;
        }
    // then calibrate gyro
    this.gyro.calibrate();
}

/**
 * resets the gyro
 */

public void reset ()
{
    // if we do not have a gyro
    if (this.hasGyro() == false)
        {
        System.out.println("***Gyro is NOT enabled!***");
        return;
        }
    // then reset the gyro
    this.gyro.reset();
}

/**
 * will return the gyro angle in degrees
 *
 * @return
 *         return gyro angle
 */

public double getAngle ()
{
    // if we don't have a gyro
    if (this.hasGyro() == false)
        {
        System.out.println("***Gyro is NOT enabled!***");
        return 0;
        }
    // return the angle of the gyro in degrees
    return this.gyro.getAngle();

}

public boolean isGyroConnected ()
{
    return this.gyro.isConnected();
}

/**
 * returns the rate of rotation for the gyro
 *
 * @return
 *         return rate of rotation
 */

public double getRate ()
{
    // if we don't have a gyro
    if (this.hasGyro() == false)
        {
        System.out.println("***Gyro is NOT enabled!***");
        return 0;
        }
    // return the rate of rotation of the gyro
    return this.gyro.getRate();
}

/**
 * @return Whether or not the gyro is enabled in the code.
 */
public boolean hasGyro ()
{
    return this.hasGyro;
}

@Override
/**
 * Free the gyro object from memory, since it is called through a JNI.
 */
public void close()
{
    if (this.hasGyro() == false)
        {
        System.out.println("***Gyro is NOT enabled!***");
        return;
        }

    //Intentionally do nothing, as wpilib has not yet immplemented a close() function.
    //Therefore calling this next line *might* result in a recursive loop. -McGee

    //this.gyro.close();
}

}

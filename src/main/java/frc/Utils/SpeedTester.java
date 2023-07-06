// ====================================================================
// FILE NAME: SpeedTester.java (Team 339 - Kilroy)
//
// CREATED ON: Jun 18, 2011
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file contains a class which measures the speed that Kilroy
// travels at. We normally do this just before we pack up the
// robot to be shipped.
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.Utils;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

// -------------------------------------------------------
/**
 * This class determines the speed that Kilroy is traveling at.
 * To use this class, declare an instance of this object while
 * passing in the encoder that you want to be measured and
 * the timer that will be used to measure time, and then
 * call periodically in teleopPeriodic() the method watchJoystick()
 * passing in the value of the joystick that you want to use to
 * denote when we are calculating the speed. When the joystick is
 * let go, the speed calculations will be written to the
 * laptop in the output window.
 * 
 * Translation by Ashley Espeland
 * declare the speed tester in hardware and only use one at a time. In the
 * declaration you pass in the encoder you are using and the timer
 * that you are using for the speed test. Make sure the timer you
 * are using is actually a thing...
 * To actually use the speed tester code, use the watch joystick method
 * in teleop by printing it out. You must pass in two joysticks, but you can
 * pass in the
 * same joystick twice, which I would highly recommend.
 * To speed test accurately for a set percentage of power, scale the
 * joystick to a certain joystick value and then set the speed to the testing
 * power that you want, of course a deadband zone is recommended, so
 * youll have to put in an else statement where you set the motors to zero
 * if they are not in the desired range.
 *
 * what the speed tester does is print out our speed in ft-in per
 * second, but only once so if you dont have a riolog running youll want
 * to have someone on the disable button to shut it down before the print
 * statement
 *
 * @class SpeedTester
 * @author Bob Brown
 * @written Jun 18, 2011
 *          -------------------------------------------------------
 */
public class SpeedTester
{
// -------------------------------------------------------

/**
 * -------------------------------------------------------
 *
 * @description denotes that we have started this test
 * @author Bob Brown
 * @written Jun 18, 2011
 *          -------------------------------------------------------
 */
private boolean startedDistanceCheck = false;

/**
 * -------------------------------------------------------
 *
 * @description denotes that we have gotten the complete distance
 *              we needed
 * @author Bob Brown
 * @written Jun 18, 2011
 *          -------------------------------------------------------
 */
private boolean distanceCounting = false;

/**
 * -------------------------------------------------------
 *
 * @description encoder to be used to measure distance traveled
 * @author Bob Brown
 * @written Jun 18, 2011
 *          -------------------------------------------------------
 */
private Encoder encoderToMeasure = null;

/**
 * -------------------------------------------------------
 *
 * @description encoder to be used to measure distance traveled
 * @author Bob Brown
 * @written Jun 18, 2011
 *          -------------------------------------------------------
 */
private Timer speedTesterTimer = null;

/**
 * constructor
 *
 * @method SpeedTester
 * @param encoderToMeasure
 *            - Encoder that will be used to measure
 *            distance
 * @param speedTesterTimer
 *            - timer that will be used to measure time
 * @author Bob Brown
 * @written Jun 18, 2011
 *          -------------------------------------------------------
 */
public SpeedTester (final Encoder encoderToMeasure,
        final Timer speedTesterTimer)
{
    this.encoderToMeasure = encoderToMeasure;
    this.speedTesterTimer = speedTesterTimer;
} // end SpeedTester

// -------------------------------------------------------
/**
 * gets the joystick value that the caller passes in
 * and determines whether or not we are now watching
 * the distance we are traveling so that we will
 * eventually measure our speed (in ft/sec)
 *
 * @method get
 * @param joystickYvalue
 *            - Y axis value to be used to measure
 *            whether or not we are measuring speed
 * @return double - speed that the motors traveled at
 *         in inches per sec
 * @author Bob Brown
 * @written Jun 18, 2011
 *          -------------------------------------------------------
 */
// NOTE ONLY USE ONE OF THESE AT A TIME OR ELSE IT WONT WORK
public double watchJoystick (double joystickYvalue,
        double secondJoystickYvalue)
{
    // -------------------------------------
    // if we haven't started the speed check before
    // and the joystick is full forward
    // denote that we have started and start the timer
    // -------------------------------------
    if (this.startedDistanceCheck == false)
        {
        if (joystickYvalue < -0.9 || secondJoystickYvalue < -0.9)
            {
            this.startedDistanceCheck = true;
            this.speedTesterTimer.reset();
            this.speedTesterTimer.start();
            } // if
        } // if
    else if (this.distanceCounting == false)
        {
        if (this.speedTesterTimer.get() > 1.0)
            {
            this.speedTesterTimer.reset();
            this.encoderToMeasure.reset();
            this.distanceCounting = true;
            } // if
        } // if
    else if (joystickYvalue > -0.9 || secondJoystickYvalue > -0.9)
        {
        int ftTraveled;
        double traveled;
        double traveledPerSec;
        int inchesFurtherTraveled;

        this.distanceCounting = false;
        this.startedDistanceCheck = false;
        this.speedTesterTimer.stop();
        traveled = this.encoderToMeasure.getDistance();
        // System.out.println(
        // "Total Dist = "
        // + traveled);
        // System.out.println("Time = " + this.speedTesterTimer.get());
        traveledPerSec = traveled / this.speedTesterTimer.get();
        ftTraveled = (int) (traveledPerSec / 12);
        inchesFurtherTraveled = (int) (traveledPerSec
                - (ftTraveled * 12));
        System.out.println("ft'inches\"/sec = " + ftTraveled + "'"
                + inchesFurtherTraveled + "\"");
        this.speedTesterTimer.reset();

        return traveledPerSec;
        } // if

    return 0; // return 0 if we are not watching or have not completed
    // watchJoystick
} // end watchJoystick

} // end class

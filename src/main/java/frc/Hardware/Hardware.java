// ====================================================================
// FILE NAME: Hardware.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 2, 2011
// CREATED BY: Bob Brown
// MODIFIED ON: June 24, 2019
// MODIFIED BY: Ryan McGee
// ABSTRACT:
// This file contains all of the global definitions for the
// hardware objects in the system
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.Hardware;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * ------------------------------------------------------- puts all of the
 * hardware declarations into one place. In addition, it makes them available to
 * both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 *
 * @written Jan 2, 2011 -------------------------------------------------------
 */

public class Hardware {

    enum Identifier {
        CurrentYear, PrevYear
    };

    public static Identifier robotIdentity = Identifier.CurrentYear;

    public static void initialize() {
        if (robotIdentity == Identifier.CurrentYear) {
            // ==============DIO INIT=============

            // ============ANALOG INIT============

            // ==============CAN INIT=============
            // Motor Controllers

            // Encoders

            // ==============RIO INIT==============

            // =============OTHER INIT============

        } else if (robotIdentity == Identifier.PrevYear) {
            // ==============DIO INIT=============

            // ============ANALOG INIT============

            // ==============CAN INIT=============

            // ==============RIO INIT=============

            // =============OTHER INIT============

        }
    }

    // **********************************************************
    // CAN DEVICES
    // **********************************************************


    // **********************************************************
    // DIGITAL I/O
    // **********************************************************


    // **********************************************************
    // ANALOG I/O
    // **********************************************************

    // **********************************************************
    // PNEUMATIC DEVICES
    // **********************************************************

    // **********************************************************
    // roboRIO CONNECTIONS CLASSES
    // **********************************************************

    public static PowerDistribution pdp = new PowerDistribution();


    // **********************************************************
    // DRIVER STATION CLASSES
    // **********************************************************

    public static DriverStation driverStation = DriverStation.getInstance();

    public static Joystick leftDriver = new Joystick(0);
    public static Joystick rightDriver = new Joystick(1);
    public static Joystick leftOperator = new Joystick(2);
    public static Joystick rightOperator = new Joystick(3);

    // **********************************************************
    // Kilroy's Ancillary classes
    // **********************************************************

    // ------------------------------------
    // Utility classes
    // ------------------------------------

    // ------------------------------------
    // Drive system
    // ------------------------------------

    // ------------------------------------------
    // Vision stuff
    // ----------------------------

    // -------------------
    // Subassemblies
    // -------------------

} // end class

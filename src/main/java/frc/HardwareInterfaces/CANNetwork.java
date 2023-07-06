package frc.HardwareInterfaces;

import edu.wpi.first.wpilibj.PowerDistribution;

// import java.util.ArrayList;

/**
 * NOTE: THIS CODE IS NOT COMPLETE! THERE ARE STILL ERRORS
 * 
 * @author Daniel Resio, Becky Button, and Cole Ramos
 */
//
//// Line 14 had a static in it, but was removed, don't change back unless
// necessary

public class CANNetwork
{

private PowerDistribution pdp = null;

/**
 * 
 * @param pdp
 *            The power distribution panel in the network.
 */
public CANNetwork (PowerDistribution pdp)
{
    this.pdp = pdp;
}



/**
 * Prints out the current currently flowing through the provided pin.
 * 
 * @param pinNumber
 *            The PDP port to print out the current for.
 * @author Becky Button
 */
public void pinCurrentPrint (int pinNumber)
{

    System.out.println(
            "Port " + String.format("%1$02d", pinNumber) + " Amps "
                    + this.pdp.getCurrent(pinNumber));

}


// currently causing a null pointer exception on testboard, but not on
// Nessie- Cole Ramos, 16 Jan 2017
public double pinValue (int pinNumber)
{
    // int pinInt = (int) this.pdp.getCurrent(pinNumber);
    return this.pdp.getCurrent(pinNumber);
}


/**
 * prints out the currents of all ports on the PDP that are being used
 * -has not been tested, is found commented out in print statements
 * in teleop
 * 
 * @author Cole Ramos
 * 
 **/

public void printAllPDPChannels ()
{


    for (int i = 0; i <= 15; i++)

        {
        if (this.pinValue(i) != 0.0)
            {
            pinCurrentPrint(i);

            }
        }
}
//
// public ArrayList<CANObject> canObjects =
// new ArrayList<CANObject>();
//
// private int talonNum = 0;
//
/// **
// * @param newObjects
// * This is an array of CAN objects in use. It is set once in the
// * constructor.
// */
//
//
// public CANNetwork(){
//
// }
//
// public CANNetwork (final ArrayList<CANObject> newObjects)
// {
// //private void "CANNetwork.canObjects = newObjects;" <-- originial
// //Need to make work with static variables
// }
//
/// **
// * This function finds the CAN object given an ID.
// *
// * @param id
// * This is the id for a CAN object.
// * @return This will return the found CAN object. Or null if we can't find the
// * CAN object. Make sure to check for null before
// * you use the return.
// */
// public CANObject getCAN (final int id)
// {
// for (int i = 0; i < canObjects.size(); i++)
// {
// CANObject tempObj = canObjects.get(i);
// // if CAN object id at location i in the array matches provided id,
// // then return that CANObject
// // else do nothing, continue loop to next spot in array, canObjects
// if (tempObj.getCanId() == i)
// {
// return tempObj;
// }
//
// }
// return null;
// }
//// If true, activates debug print statements throughout the class
// private boolean useDebug = true;
//
/// **
// * Prints values of the significant variables/methods in CANNetwork if
// useDebug
// * is true
// */
// private void DebugCANNetwork ()
// {
// //If true, runs DebugCANNetwork
// if (useDebug == true)
// {
// //Prints the size of the CANOBject array list
// System.out.println("The size of the CANObject array list is "
// + canObjects.size());
// }
// }
//
/// **
// * This function finds the CAN object given an ID.
// *
// * @param id
// * This is the id for a CAN object.
// * @return This will return the found CAN object. Or null if we can't find the
// * CAN object. Make sure to check for null before
// * you use the return.
// */
// public CANObject getCANWithId (final int id)
// {
//
// for (int i = 0; i < canObjects.size(); i++)
// {
// CANObject tempObj = canObjects.get(i);
// // if CAN object id at location i in the array matches provided id,
// // then return that CANObject
// // else do nothing, continue loop to next spot in array, canObjects
// if (tempObj.getCanId() == i)
// {
// return tempObj;
// }
//
// }
// //return a null object because we can't find the CANDevice you're looking for
// return null;
// }
}

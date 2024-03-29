// ====================================================================
// FILE NAME: KilroyCamera.java (Team 339 - Kilroy)
//
// CREATED ON: Oct 16, 2014
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This class is used to hold an AxisCamera instance or null if we
// say that we do not have a camera. If we say that we do not have
// a camera, all of the methods dealing with the camera values return
// bogus values/don't acutally set anything when asked to write a
// value. All of the methods are identical to the AxisCamera methods,
// except for the constructor, those dealing with having a camera,
// and getCameraInstance().
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================

package frc.HardwareInterfaces;

import java.io.FileNotFoundException;
import java.io.IOException;
import edu.wpi.first.cscore.AxisCamera;

/**
 * A camera container/interface that holds an instance of the AxisCamera
 * class and a boolean used to determine if we have a camera on the robot.
 * This class has methods that are the same as the AxisCamera classes (it
 * does not include all of methods, only the ones that we use at this
 * point). If we do not have a camera, the class does nothing when the
 * camera methods are called, returning inaccurate values if a get was called
 *
 * @author Nathanial Lydick
 * @written Oct 16, 2014
 */
public class KilroyCamera
{
// holds the camera instance, or null if we have no camera
private final AxisCamera camera;

// Holds the network addres of the KilroyCamera
private String cameraAddress;

// the return value for integers when we have no camera
public static final int noCameraIntReturnValue = -1;

private static int imageNumber = 0;

private static int txtNumber = 0;

// The focal length of the camera, you can find it in the camera documentation
private double focalLength = 2.8;

// The horizontal field of view of the camera in degrees.
private double horizFieldOfView = 67;

private double vertFieldOfView = 34;
// claimed to be a = 2 arctan(d/2f)
// d = size of sensor in direction of interest
// f = focal length of camera
// a = field of view

// --------------------------------------------------------------
// All methods below this point simply call the methods for the
// Axis camera, if we have one, else it does nothing or it returns
// a bogus value (sometimes an impossible value, such as the ints)
// --------------------------------------------------------------

// Constructor, passes in whether we have a camera, and uses the
// default AxisCamera.getInstance() to get the camera instance
// if we say that we have one
// -------------------------------------------------------
/**
 * Sets the Camera boolean, in case a change is necessary
 *
 * @method KilroyCamera() - constructor
 * @param address
 *                      - The network address of the camera
 * @author Nathan Lydick
 * @written Oct 16, 2014
 * @updated_by Alex Kneipp
 * @updated_on 7/10/2019
 *          -------------------------------------------------------
 */

public KilroyCamera (String address)
{
    this.cameraAddress = address;
    if (address == null)
    {
        System.out.println("ATTENTION: the AXIS CAMERA is DISABLED :(");
        this.camera = null;
    }
    else
    {
        this.camera = new AxisCamera("Kilroy Axis", address);
    }
}

/**
 * ---------------------------------------------
 * Clear all images previously taken. This will
 * actually remove the entire directory, because
 * we always remake the directory before populating
 * any image
 */
public void clearAllImages ()
{
    try
        {
        // rm -rf removes a directory
        Runtime.getRuntime()
                .exec("/bin/rm -rf /home/lvuser/images");
        // saves new file
        }  // end try
    catch (final IOException e)
        {
        e.printStackTrace();
        } // catch any errors

} // end clearAllImages

// public boolean freshImage ()
// {
// if (this.haveCamera)
// return this.camera.isFreshImage();
// // returns true so that if we ever try to wait for an image, we will
// // just continue
// return true;
// }

public int getBrightness ()
{
    if (this.camera != null)
        return this.camera.getBrightness();
    // returns a value that it shouldn't be
    return noCameraIntReturnValue;
}

// -------------------------------------------------------
/**
 * Returns the AxisCamera (final), in case direct access is necessary
 * For the most part, this should not be used, and if a new camera
 * method is needed, it should just be added to this class
 *
 * @method getCameraInstance()
 * @return AxisCamera - the camera instance (or null)
 * @author Nathan Lydick
 * @written Oct 16, 2014
 *          -------------------------------------------------------
 */
public final AxisCamera getCameraInstance ()
{
    return this.camera;
}

// public int getColorLevel ()
// {
// if (this.haveCamera)
// return this.camera.getColorLevel();
// // returns a value that it shouldn't be
// return noCameraIntReturnValue;
// }

// public int getCompression ()
// {
// if (this.haveCamera)
// return this.camera.getCompression();
// // returns a value that it shouldn't be
// return noCameraIntReturnValue;
// }

// public AxisCamera.ExposureControl getExposureControl ()
// {
// if (this.haveCamera)
// return this.camera.getExposureControl();
// // returns automatic
// return AxisCamera.ExposureControl.kAutomatic;
// }

// -------------------------------------------------------
/**
 * Returns whether we have a camera
 *
 * @method gethaveCamera()
 * @return boolean - returns whether we have a camera
 * @author Nathan Lydick
 * @written Oct 16, 2014
 * @updated_by Alex Kneipp
 * @updated_on 7/10/2019
 *          -------------------------------------------------------
 */
public boolean gethaveCamera ()
{
    return this.camera != null;
}

// -------------------------------------------------------
/**
 * Returns the AxisCamera (final), in case direct access is necessary
 * For the most part, this should not be used, and if a new camera
 * method is needed, it should just be added to this class
 *
 * @method getCameraInstance()
 * @return ColorImage - Camera.getImage() or a 0x0 hsl image
 * @author Nathan Lydick
 * @throws NIVisionException
 *                               - throws exception when fails
 * @written Oct 16, 2014
 *          -------------------------------------------------------
 */
// public ColorImage getImage () throws NIVisionException
// {
// if (this.haveCamera)
// return this.camera.getImage();
// // returns a 0x0 image
// return new HSLImage();
// }

// public int getMaxFPS ()
// {
// if (this.haveCamera)
// return this.camera.getMaxFPS();
// // returns a value that it shouldn't be
// return noCameraIntReturnValue;
// }

// --------------------------------------------------------------
// The above methods simply call the methods for the
// Axis camera, if we have one, else it does nothing or it returns
// a bogus value (sometimes an impossible value, such as the ints
// --------------------------------------------------------------
// public AxisCamera.Resolution getResolution ()
// {
// if (this.haveCamera)
// return this.camera.getResolution();
// // returns the smallest resolution and our default
// return AxisCamera.Resolution.k160x120;
// }

// public AxisCamera.Rotation getRotation ()
// {
// if (this.haveCamera)
// return this.camera.getRotation();
// // returns 0 rotation
// return AxisCamera.Rotation.k0;
// }

// public AxisCamera.WhiteBalance getWhiteBalance ()
// {
// if (this.haveCamera)
// return this.camera.getWhiteBalance();
// // returns automatic
// return AxisCamera.WhiteBalance.kAutomatic;
// }

/**
 * @return
 *         The focal length of the camera this object refers to, in millimeter.
 */
public double getFocalLength ()
{
    return this.focalLength;
}

/**
 * Saves the focal length of the of the camera this object refers to in this
 * class
 */
public void setFocalLength (double focalLength)
{
    this.focalLength = focalLength;
}

/**
 *
 * @return
 *         The horizontal field of view of the camera, in degrees
 */
public double getHorizFieldOfView ()
{
    return this.horizFieldOfView;
}

/**
 * Saves the field of view of the hardware camera into this class.
 *
 * @param FOV
 *                THe horizontal field of view, in degrees.
 */
public void setVertFieldOfView (double FOV)
{
    this.vertFieldOfView = FOV;
}


public double getVertFieldOfView ()
{
    return this.vertFieldOfView;
}

/**
 * Saves the field of view of the hardware camera into this class.
 *
 * @param FOV
 *                THe horizontal field of view, in degrees.
 */
public void setHorizFieldOfView (double FOV)
{
    this.horizFieldOfView = FOV;
}


// this is called
/**
 *
 * @return
 *         The horizontal resolution of the axis camera this object refers to.
 */
// public double getHorizontalResolution ()
// {
// switch (this.getResolution())
// {
// case k640x480:
// return 640.0;
// case k480x360:
// return 480.0;
// case k320x240:
// return 320.0;
// case k240x180:
// return 240.0;
// case k176x144:
// return 176.0;
// default:
// case k160x120:
// return 160.0;
// }
// }

// /**
// *
// * @return
// * The vertical resolution of the camera this object this refers to.
// */
// public double getVerticalResolution ()
// {
// switch (this.getResolution())
// {
// case k640x480:
// return 480.0;
// case k480x360:
// return 360.0;
// case k320x240:
// return 240.0;
// case k240x180:
// return 180.0;
// case k176x144:
// return 144.0;
// default:
// case k160x120:
// return 120.0;
// }
// }

/**
 * Takes an image from the camera and stores it in the specified file path.
 * This will override old images of the same name.
 *
 * @param fileName
 *                     no extensions necessary
 * @updated_by Alex Kneipp
 * @updated_on 7/10/2019
 */


public void saveImage (String fileName)
{
    // System.out.println("\n\n\nthe command is : " );

    // Pre-creates the directory for images in all cases
    try
        {
        Runtime.getRuntime()
                .exec("/bin/mkdir -p /home/lvuser/images");
        // saves new file
        }
    catch (final IOException e)
        {
        e.printStackTrace();
        }
    // Erases any pre-existing files that have previously been created
    try
        {
        Runtime.getRuntime()
                .exec("/bin/rm -rf /home/lvuser/images/" + fileName
                        + ".jpg");
        // saves new file
        }
    catch (final IOException e)
        {
        e.printStackTrace();
        }
    // Creates new image, keep a log of the creation of the file
    try
        {
        Runtime.getRuntime()
                .exec("/usr/bin/wget http://" + this.cameraAddress
                        + "/jpg/image.jpg "
                        + "-O /home/lvuser/images/" + fileName
                        + ".jpg >> /home/lvuser/images/log.txt");
        // saves new file
        }
    catch (final IOException e)
        {
        e.printStackTrace();
        }
}

/**
 * calls up saveImage method and labels pictures Image0 through Image9
 * when more than 10 pictures are taken, the label "loops" back to 0
 * this is done through the modulus function, which divides the imageNumber
 * by ten and uses the remainder as the image number
 *
 * author@ Ashley Espeland
 * date@ 1/31016
 */
public void saveImagesSafely ()
{
    // calls saveImage and assigns image number
    this.saveImage("Image" + ((imageNumber % 25) + 1));
    // increments imageNumber
    imageNumber++;

}

public void saveTextSafely (String text)
{
    // Pre-creates the directory for files in all cases
    try
        {
        Runtime.getRuntime()
                .exec("/bin/mkdir -p /home/lvuser/TextFiles");
        // echo the text to the shell and pipe it into a new file
        Runtime.getRuntime()
                .exec("echo " + text + " > "
                        + "/home/lvuser/TextFiles/RoboRIO_Out_"
                        + txtNumber + ".txt"); // @TEST
        txtNumber++;
        }
    catch (FileNotFoundException e)// Shouldn't actually happen.
        {
        System.out.println("Failed to save the text file!");
        }
    catch (final IOException e)
        {
        System.out.println("Failed to make text directory!");
        }
}



// -------------------------------------------------------
/**
 * Sets the Camera boolean, in case a change is necessary
 *
 * @method setHaveCamera() - constructor
 * @param value
 *                  - the new value as to whether we have a camera
 * @author Nathan Lydick
 * @written Oct 16, 2014
 * @updated_by Alex Kneipp
 * @updated_on 7/10/2019
 * @deprecated on 7/10/2019
 *  Since the haveCamera boolean is no longer used, this function is a nop but included for compatibility
 *          -------------------------------------------------------
 */
@Deprecated
public void setHaveCamera (boolean value)
{
    //Avoiding warnings for useless statements
    if(value)
        return;
}

// public void writeBrightness (int brightness)
// {
// if (this.haveCamera)
// {
// this.camera.writeBrightness(brightness);
// }
// else
// {
// // returns nothing

// }
// }

// public void writeColorLevel (int value)
// {
// if (this.haveCamera)
// {
// this.camera.writeColorLevel(value);
// }
// else
// {
// // returns nothing

// }
// }

// public void writeCompression (int value)
// {
// if (this.haveCamera)
// {
// this.camera.writeCompression(value);
// }
// else
// {
// // returns nothing

// }
// }

// public void writeExposureControl (AxisCamera.ExposureControl value)
// {
// if (this.haveCamera)
// {
// this.camera.writeExposureControl(value);
// }
// else
// {
// // returns nothing

// }
// }

// public void writeExposure(int value)
// {
// if (this.haveCamera)
// {
// this.camera.writeExposure
// }
// }

// public void writeMaxFPS (int value)
// {
// if (this.haveCamera)
// {
// this.camera.writeMaxFPS(value);
// }
// else
// {
// // returns nothing

// }
// }

// public void writeResolution (AxisCamera.Resolution value)
// {
// if (this.haveCamera)
// {
// this.camera.writeResolution(value);
// }
// else
// {
// // returns nothing

// }
// }

// public void writeRotation (AxisCamera.Rotation value)
// {
// if (this.haveCamera)
// {
// this.camera.writeRotation(value);
// }
// else
// {
// // returns nothing

// }
// }

// public void writeWhiteBalance (AxisCamera.WhiteBalance whiteBalance)
// {
// if (this.haveCamera)
// {
// this.camera.writeWhiteBalance(whiteBalance);
// }
// else
// {
// // returns nothing

// }
// }


/**
 * Takes a single image
 *
 * @param joystickButton
 *                           joystick.button
 * @author Becky Button
 */
public void takeSinglePicture (boolean rawButton)
{
    if (rawButton == true && previousPictureState == false)
        {
        saveImagesSafely();
        }
    previousPictureState = rawButton;
}

private static boolean previousPictureState;

}

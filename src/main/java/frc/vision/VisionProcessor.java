package frc.vision;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Date;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Relay.Value;



/**
 * This class contains vision code that uses OpenCV and the auto-generated code
 * from the program GRIP. To properly set up GRIP to work with this class, make
 * sure to set the package name, the class name to AutoGenVision.java and the
 * directory to the correct package.
 *
 * NOTE: The GRIP project MUST end with a "filter contours" modifier
 *
 * @author Ryan McGee
 * @written 6/22/17
 *
 */
public class VisionProcessor extends AutoGenVision
{

/**
 * A class that holds several statistics about particles.
 *
 * The measures include: area: The area, in pixels of the blob boundingRect: The
 * rectangle around the blob center: the point of the center of the blob
 *
 * @author Ryan McGee
 *
 */
public class ParticleReport implements Comparator<ParticleReport>,
        Comparable<ParticleReport>
{
/**
 * The area of the bounding rectangle around the blob
 */
public double area = 0.0;

/**
 * The rectangle around the blob
 */
public Rect boundingRect = new Rect(new Point(0, 0), new Point(0, 0));

/**
 * the center of the bounding rectangle around the blob
 */
public Point center = new Point(0, 0);

/**
 * Compares the areas of the two ParticleReports
 *
 * @param r1
 *               The first particle report in the comparison
 * @param r2
 *               The second particle report in the comparison
 * @return the area of r1 - the area of r2; positive value if r1.area > r2.area,
 *         0 if they are equal, and a negative value if r1.area < r2.area
 */
@Override
public int compare (ParticleReport r1, ParticleReport r2)
{
    return (int) (r1.area - r2.area);
} // end compare()

/**
 * Compares the area of <b>this</b> ParticleReport to the area of the inputted
 * particle report
 *
 * @param r
 *              The particle report whose area is being compared to <b>this</b>
 *              particle report's
 *
 * @return the area of r - the area of <b>this</b>; positive value if r.area >
 *         this.area, 0 if they are equal, and a negative value if r.area <
 *         this.area
 */
@Override
public int compareTo (ParticleReport r)
{
    return (int) (r.area - this.area);
} // end compareTo()
} // end class VisionProcessor

/**
 * The user must set which camera is connected for correct field of views and
 * focal lengths.
 *
 * @author Ryan McGee
 *
 */
public enum CameraModel
    {
    /**
     * The USB camera supplied by FIRST, model Lifecam HD-3000
     */
    LIFECAM,
    /**
     * The OLD model of the IP camera supplied by FIRST
     */
    AXIS_M1011,
    /**
     * The NEW model of the IP camera supplied by FIRST
     */
    AXIS_M1013
    } // end enum CameraModel

/**
 * A list of the different kind of images, for accessing the images directly.
 *
 * @author Ryan McGee
 */
public enum ImageType
    {
    /**
     * An image straight from the camera.
     */
    RAW,
    /**
     * An image that has gone through post processing.
     */
    PROCESSED
    } // end enum ImageType

// In order to calculate the horizontal / vertical field of view,
// you can use the formula: a = 2arctan(d/2f) where 'a' is the angle,
// 'd' is the size of the sensor (in millimeters and in the direction
// needed), and f is the focal length (again in millimeters).
// source:
// https://photo.stackexchange.com/questions/21536/how-can-i-calculate-vertical-field-of-view-from-horizontal-field-of-view.

// Remember to use all info available. If the datasheet says 47 degrees
// horizontal and the calculated answer is
// different, remember that video cuts off a certain amount of data and
// recalculate presuming only that portion
// of the sensor is used.

// ========M1011 SPECS========
// horizontal field of view for the M1011 camera, in degrees
private final int M1011_HORIZ_FOV = 47;

// vertical field of view for the M1011 camera, in degrees
private final int M1011_VERT_FOV = 36;

// ========M1013 SPECS========
// horizontal field of view for the M1013 camera, in degrees
private final int M1013_HORIZ_FOV = 67;

// vertical field of view for the M1013 camera, in degrees
private final int M1013_VERT_FOV = 51;

// ========LIFECAM SPECS========
/*
 * There is not enough information on the technical data sheet to find this
 * info. They must instead be calculated manually.
 */

// the first part of the path for images
private final String IMAGE_BASIC_PATH = "/home/lvuser";

// the file path where we save the images we take
private final String SAVE_IMAGE_PATH = IMAGE_BASIC_PATH + "/images";

private Mat image = new Mat(); // The stored "image" (in a matrix format)

private ParticleReport[] particleReports = new ParticleReport[0];

// the horizontal field of view of the current camera, in degrees
private final int horizontalFieldOfView;

// the vertical field of view of the current camera, in degrees
private final int verticalFieldOfView;

// the model of the current camera
private final CameraModel cameraModel;

private final VideoCamera camera;

// brightness the camera is set to under setDefaultCameraSettings
private int DEFAULT_CAMERA_BRIGHTNESS = 50;

// ========OBJECTS FOR TAKE LIT IMAGE========
// relay that controls the ringLight (turns it on or off)

// private DigitalOutput tempRingLight = null;

private Relay tempRingLight = null;

// timer used in the takeLitPicture function to delay taking an image until
// after the ringLight turned on
private final Timer pictureTimer = new Timer();

/**
 * Creates the object and starts the camera server
 *
 * @param ip
 *                   the IP of the the axis camera (usually 10.3.39.11)
 * @param camera
 *                   the brand / model of the camera
 */
public VisionProcessor (String ip, CameraModel camera)
{
    // Adds the camera to the cscore CameraServer, in order to grab the
    // stream.
    this.camera = CameraServer.getInstance()
            .addAxisCamera("axis-camera", ip);
    // originally named Vision Camera



    // Based on the selected camera type, set the field of views and focal
    // length.
    this.cameraModel = camera;
    switch (this.cameraModel)
        {
        case AXIS_M1011:
            this.horizontalFieldOfView = M1011_HORIZ_FOV;
            this.verticalFieldOfView = M1011_VERT_FOV;
            break;
        case AXIS_M1013:
            this.horizontalFieldOfView = M1013_HORIZ_FOV;
            this.verticalFieldOfView = M1013_VERT_FOV;
            break;

        default: // Data will default to one to avoid any "divide by zero"
                 // errors.
            this.horizontalFieldOfView = 1;
            this.verticalFieldOfView = 1;
        } // end switch

} // end VisionProcessor()

/**
 * Creates the object and starts the camera server
 *
 * @param ip
 *                           the IP of the the axis camera (usually 10.3.39.11)
 * @param camera
 *                           the brand / model of the camera
 * @param ringlightRelay
 *                           camera ringlight to pick up retro-reflective tape:
 *                           this
 *                           is the janky fix
 *
 */

// public VisionProcessor (String ip, CameraModel camera,
// DigitalOutput ringlightRelay)
// {
// // Adds the camera to the cscore CameraServer, in order to grab the
// // stream.
// this.camera = CameraServer.getInstance()
// .addAxisCamera("axis-camera", ip);



// // Based on the selected camera type, set the field of views and focal
// // length.
// this.cameraModel = camera;
// switch (this.cameraModel)
// {
// case AXIS_M1011:
// this.horizontalFieldOfView = M1011_HORIZ_FOV;
// this.verticalFieldOfView = M1011_VERT_FOV;
// break;
// case AXIS_M1013:
// this.horizontalFieldOfView = M1013_HORIZ_FOV;
// this.verticalFieldOfView = M1013_VERT_FOV;
// break;

// default: // Data will default to one to avoid any "divide by zero"
// // errors.
// this.horizontalFieldOfView = 1;
// this.verticalFieldOfView = 1;
// } // end switch

// this.pictureTimer.reset();
// this.tempRingLight = ringlightRelay;
// } // end VisionProcessor()



/**
 * Creates the object and starts the camera server
 *
 * @param ip
 *                           the IP of the the axis camera (usually 10.3.39.11)
 * @param camera
 *                           the brand / model of the camera
 * @param ringlightRelay
 *                           - actual relay not just DigitalOutput
 *                           camera ringlight to pick up retro-reflective tape:
 *                           this
 *                           is the janky fix
 *
 */

public VisionProcessor (String ip, CameraModel camera,
        Relay ringlightRelay)
{
    // Adds the camera to the cscore CameraServer, in order to grab the
    // stream.
    this.camera = CameraServer.getInstance()
            .addAxisCamera("axis-camera", ip);



    // Based on the selected camera type, set the field of views and focal
    // length.
    this.cameraModel = camera;
    switch (this.cameraModel)
        {
        case AXIS_M1011:
            this.horizontalFieldOfView = M1011_HORIZ_FOV;
            this.verticalFieldOfView = M1011_VERT_FOV;
            break;
        case AXIS_M1013:
            this.horizontalFieldOfView = M1013_HORIZ_FOV;
            this.verticalFieldOfView = M1013_VERT_FOV;
            break;

        default: // Data will default to one to avoid any "divide by zero"
                 // errors.
            this.horizontalFieldOfView = 1;
            this.verticalFieldOfView = 1;
        } // end switch

    this.pictureTimer.reset();
    this.tempRingLight = ringlightRelay;


} // end VisionProcessor()


/**
 * Creates the object and starts the camera server
 *
 * @param usbPort
 *                    The USB camera port number. '0' for default.
 * @param camera
 *                    the brand / model of the camera
 */
public VisionProcessor (int usbPort, CameraModel camera)
{
    // Adds the camera to the cscore CameraServer, in order to grab the
    // stream.
    this.camera = CameraServer.getInstance()
            .startAutomaticCapture("axis-camera", usbPort);

    // Based on the selected camera type, set the field of views and focal
    // length.
    this.cameraModel = camera;
    switch (this.cameraModel)
        {
        // case LIFECAM: //Not enough information to properly find this data.
        // see above.
        // this.horizontalFieldOfView =
        // this.verticalFieldOfView =
        // this.focalLength =
        // break;
        default: // Data will default to one to avoid any "divide by zero"
                 // errors.
            this.horizontalFieldOfView = 1;
            this.verticalFieldOfView = 1;
        } // end switch

} // end VisionProcessor()

/**
 * Creates the object and starts the camera server
 *
 * @param usbPort
 *                           The USB camera port number. '0' for default.
 * @param camera
 *                           the brand / model of the camera
 * @param ringlightRelay
 *                           camera ringlight to pick up retro-reflective tape
 */
// public VisionProcessor (int usbPort, CameraModel camera,
// DigitalOutput ringlightRelay)
// {
// // Adds the camera to the cscore CameraServer, in order to grab the
// // stream.
// this.camera = CameraServer.getInstance()
// .startAutomaticCapture("axis-camera", usbPort);

// // Based on the selected camera type, set the field of views and focal
// // length.
// this.cameraModel = camera;
// switch (this.cameraModel)
// {
// // case LIFECAM: //Not enough information to properly find this data.
// // see above.
// // this.horizontalFieldOfView =
// // this.verticalFieldOfView =
// // this.focalLength =
// // break;
// default: // Data will default to one to avoid any "divide by zero"
// // errors.
// this.horizontalFieldOfView = 1;
// this.verticalFieldOfView = 1;
// } // end switch
// this.tempRingLight = ringlightRelay;
// this.pictureTimer.reset();
// } // end VisionProcessor()

// ==========================END INIT===================================

/**
 * The method that processes the image and inputs it into the particle reports
 */
public void processImage ()
{
    // Gets the error code while getting the new image from the camera.
    // If the error code is not 0, then there is no error.
    long errorCode = CameraServer.getInstance()
            .getVideo("axis-camera").grabFrame(image);



    if (image.empty() == true)
        {
        System.out.println("Image is Empty! Unable to process image!");
        return;
        } // end if

    if (errorCode == 0)
        {
        System.out.println(
                "There was an error grabbing the image. See below:");
        System.out.println(
                CameraServer.getVideo().getError());
        } // end if

    // The process image function found in the AutoGenVision class.
    super.process(image);
    // If this throws an error, make sure the GRIP project ends with a
    // filterContours function.
    this.createParticleReports(super.filterContoursOutput());
    // Sort the particles from largest to smallest
    Arrays.sort(particleReports);
    // for (int i = 0; i < particleReports.length; i++)
    // {
    // System.out.println(i + " " + particleReports[i].area);
    // } // end for
} // end processImage()

/**
 * Sets the camera image settings for use in image processing.
 *
 * @param exposure
 *                         How much light will hit the sensor, in percentage.
 * @param whiteBalence
 *                         The white balence of the camera. Constants are found
 *                         in
 *                         the VideoCamera class
 * @param brightness
 *                         How bright the image is in post processing, in
 *                         percentage.
 */
public void setCameraSettings (int exposure, int whiteBalence,
        int brightness)
{
    this.camera.setBrightness(brightness);
    this.camera.setExposureManual(exposure);
    this.camera.setWhiteBalanceManual(whiteBalence);
} // end setCameraSettings()

/**
 * Sets the camera back to default settings for switching between vision
 * processing and driver assisting mode.
 */
public void setDefaultCameraSettings ()
{
    this.camera.setExposureAuto();
    this.camera.setBrightness(DEFAULT_CAMERA_BRIGHTNESS);
    this.camera.setWhiteBalanceAuto();
} // end setDefaultCameraSettings()

/**
 * Saves an image to the roborio. This has a max of 26 images per type, before
 * it starts to overwrite. The image will be saved to the SAVE_IMAGE_PATH
 * defined above.
 *
 * @param type
 *                 What kind of image will be saved. If it is ImageType.RAW,
 *                 then an
 *                 image will be saved directly from the camera. if
 *                 ImageType.PROCESSED is chosen, then the robot will save the
 *                 image
 *                 after it has gone through the filters.
 */

public void saveImage (ImageType type)
{
    String fileName = "";

    // Create the path the images will be saved in. If the path already
    // exists, do nothing.

    try
        {
        // system command that creates the path the image will be saved in

        Runtime.getRuntime()
                .exec("mkdir -p " + SAVE_IMAGE_PATH + timeStamp);
        // "echo $(ls -1rtd images*| head -n$(($(ls -1d images*| wc -l)-" +
        // numImageFolders+"))) \\> temp.txt"
        // ls -l | find -name "images*" | tail -n +6 | xargs rm -r

        Runtime.getRuntime()
                .exec("ls -ld " + IMAGE_BASIC_PATH + "; sleep 30");

        // Runtime.getRuntime().exec(
        // "ls -lt | find -name \"images*\" | tail -n +6 | xargs rm -r");
        } // end try
    catch (IOException e)
        {
        e.printStackTrace();
        } // catch

    // grab the image
    Mat tempImage = new Mat();


    // maybe check to see if we have an image first before using this
    // test if we are getting an image, and then try putting it in a loop
    // until we get one
    CameraServer.getVideo("axis-camera")
            .grabFrame(tempImage);

    if (tempImage.empty() == true)
        {
        // redo taking photo
        CameraServer.getVideo("axis-camera")
                .grabFrame(tempImage);
        // this is a temporary test {Craig}

        if (tempImage.empty() == true)
            {
            System.out.println("this photo is empty");
            }

        else
            {
            System.out.println("Photo taken");

            }

        }

    // Choses which type of image will be saved: raw or processed.
    switch (type)
        {
        case RAW:
            // Creating the file name. Only 30 images will be saved before
            // overwrite.
            if (rawImageNum > this.maxRawImagesAllowedToCollect)
                rawImageNum = 0;
            fileName = "raw_image_" + rawImageNum++ + ".png";
            break;
        case PROCESSED:
            // Creating the file name. Only 30 images will be saved before
            // overwrite.
            if (processedImageNum > this.maxProcessedImagesAllowedToCollect)
                processedImageNum = 0;
            fileName = "proc_image_" + processedImageNum++ + ".png";
            // Only process the image if it is chosen as the image type.
            super.process(tempImage);
            tempImage = super.rgbThresholdOutput();
            break;
        default:
            // Should not run, but will if another imageType is added and
            // chosen.
            System.out.println(
                    "Failed to save image: Image type not recognized.");
            break;
        } // switch
          // Save the image to the folder specified with the name specified



    Imgcodecs.imwrite(SAVE_IMAGE_PATH + timeStamp + "/" + fileName,
            tempImage);
    System.out.println("Image: " + SAVE_IMAGE_PATH + timeStamp + "/");
    // else
    // System.out.println("saveImage: Image was empty; was not saved");
} // end saveImage()

private int rawImageNum = 0;

private int processedImageNum = 0;

/**
 * Saves an image once (and only once), no matter how long the button is pressed
 * down on the joystick.
 *
 * @param button
 *                   Whether or not the button is pressed
 * @param type
 *                   What kind of image should be saved to the RoboRIO's
 *                   storage.
 */
public void saveImageSafely (boolean button, ImageType type)
{
    if (button == true && saveImageButtonState == false)
        this.saveImage(type);
    saveImageButtonState = button;
} // end saveImageSafely()

private boolean saveImageButtonState = false;

/**
 * Takes a lit picture with the camera when the user presses a button
 *
 * @param button
 *                   1 (or multiple) joystick button values; recommended to use
 *                   2
 *                   example parameter: joystick.getRawButton(x) &&
 *                   joystick.getRawButton(y)
 *
 */
public void takeLitPicture (boolean button)
{
    // Start the timer if the button is pressed
    if (button == true && pictureTakenByButton == false
            && takePictureByButton == false)
        {
        this.takePictureByButton = true;
        this.pictureTimer.start();
        } // end if

    // if the button isn't pressed, reset the other booleans
    if (button == false && pictureTakenByButton == true)
        {
        takePictureByButton = false;
        pictureTakenByButton = false;
        } // end if

    // if both buttons are pressed, turn on the relay
    if (this.takePictureByButton == true)
        {

        // turns on the ring light
        if (this.pictureTimer.get() <= TAKE_PICTURE_DELAY / 2.0)
            {

            this.setRelayValue(Value.kOn);
            }
        // if the timer expires, save the picture , reset booleans, turns off
        // the ring light
        if (this.pictureTimer.get() >= TAKE_PICTURE_DELAY)
            {
            this.saveImageSafely(true, ImageType.RAW);

            this.pictureTakenByButton = true;
            this.takePictureByButton = false;

            this.saveImageSafely(false, ImageType.RAW);

            // this.setRelayValue(Value.kOff);
            this.pictureTimer.stop();
            this.pictureTimer.reset();
            } // end if
        } // end if
} // end takeLitPicture()

private boolean takePictureByButton = false;

private boolean pictureTakenByButton = false;

private final double TAKE_PICTURE_DELAY = 1.0;

/**
 * Gets the value of the ring light relay
 *
 * @return the value of the camera ring light relay; see the get() function in
 *         the Relay class for more information
 */
public boolean getRelayValue ()
{
    if (this.tempRingLight.get() == Value.kOn)
        {
        return true;
        }
    else
        {
        return false;
        }
} // end getRelayValue()

/**
 * Gets the value of the ring light relay as a Digital Output
 *
 * @return the value of the camera ring light relay; see the get() function in
 *         the Digital Output class for more information
 */
public boolean getDigitalOutputValue ()
{
    if (this.tempRingLight.get() == Value.kOn)
        {
        return true;
        }
    else
        {
        return false;
        }
} // end getDigitalOutputValue ()

/**
 * Set the ring light to a value
 *
 * @param ringLightValue
 *                           use kForward or kReverse to set the ring light
 */
public void setRelayValue (Value ringLightValue)
{

    this.tempRingLight.set(ringLightValue);


} // end setRelayValue()

/**
 * Set the ring light to a value
 *
 * @param ringLightValue
 *                           use true to turn the relay on
 */
public void setDigitalOutputValue (Value ringLightValue)
{
    this.tempRingLight.set(ringLightValue);
} // end setDigitalOutputValue

/**
 * Turns on the ring light
 *
 * @param button
 *                   1 (or multiple) joystick buttons
 */
public void turnRingLightOn (boolean button)
{
    // if(button == true)
    // {
    // this.setRelayValue(Value.kForward);
    // }
} // end turnRingLightOn()

// =====================USER ACCESSABLE METHODS========================
/*
 * Any methods that will allow the user to directly access raw data outside the
 * class will be stored below.
 */

/**
 * @return the list of blobs generated after processing the image, in descending
 *         order of size.
 */
public ParticleReport[] getParticleReports ()
{
    return particleReports;
} // getParticleReports()

/**
 * @return Whether or not the camera can see any retro-reflective tape
 */
public boolean hasBlobs ()
{
    // @ANE
    if (this.getParticleReports().length >= 1)
        {
        return true;
        }

    return false;
} // end hasBlobs()

/**
 * Gets a report of the index the user requests.
 *
 * @param n
 *              The index of the size requested. 0 is the largest, and gradually
 *              gets smaller until the end of the array is reached.
 * @return The blob thats the Nth largest in the particleReports array.
 */
public ParticleReport getNthSizeBlob (int n)
{
    return particleReports[n];
} // end getNthSizeBlob()

// ======================POST PROCESSING METHODS========================
/*
 * Any methods that DO NOT require direct OpenCV access and are NOT game
 * specific can be placed below.
 */

/**
 * Takes the base OpenCV list of contours and changes the output to be easier to
 * work with.
 *
 * @param contours
 *                     The input from the base OpenCV contours output
 */
private void createParticleReports (List<MatOfPoint> contours)
{
    ParticleReport[] reports = new ParticleReport[contours.size()];

    // creates and properly sets the values of each individual element
    // in the reports arrays
    for (int i = 0; i < reports.length; i++)
        {
        reports[i] = new ParticleReport();
        Rect r = Imgproc.boundingRect(contours.get(i));
        reports[i].area = r.area();
        reports[i].center = new Point(r.x + (r.width / 2.0),
                r.y + (r.height / 2.0));
        reports[i].boundingRect = r;
        } // end for

    this.particleReports = reports;
} // end createParticleReports()

/**
 * TODO TEST THIS AND COMMENT SOME OF THE CALCULATIONS
 *
 * Calculates the angle the target is at from the center line. The formula can
 * be cut into two easier sections, one for the focal length and one for the
 * angle.
 *
 * Focal length (in pixels): Resolution / 2 x tan(FOV / 2) Angle (in radians):
 * arctan(distanceFromCenter / focalLength)
 *
 * @param target
 *                   The input: takes the Y axis from the center point.
 * @return the angle, in degrees. If the target is above the center line, it
 *         will show positive. If it is below the line, it will show negative.
 */
public double getPitchAngleDegrees (ParticleReport target)
{
    int distFromCenterLine = (int) Math
            .abs((image.size().height / 2.0) - target.center.y);

    // The focal length is dependent on the resolution of the image, since
    // units must remain in pixels, and the field of view must not change.
    double focalLengthPixels = image.size().height
            / (2.0 * Math.tan(verticalFieldOfView / 2.0));

    // Conditions for the return statement based on the position of the
    // target.
    if ((image.size().height / 2.0) - target.center.y > 0.0)
        return Math.toDegrees(
                Math.atan(distFromCenterLine / focalLengthPixels));

    return -Math.toDegrees(
            Math.atan(distFromCenterLine / focalLengthPixels));
} // end getPitchAngleDegrees

/**
 * TODO TEST THIS AND COMMENT SOME OF THE CALCULATIONS
 *
 * Calculates the angle the target is at from the center line. The formula can
 * be cut into two easier sections, one for the focal length and one for the
 * angle.
 *
 * Focal length (in pixels): Resolution / 2 x tan(FOV / 2) Angle (in radians):
 * arctan(distanceFromCenter / focalLength)
 *
 * @param target
 *                   The input: takes the X axis from the center point.
 * @return the angle, in degrees. If the target is to the right of the center
 *         line, it will show positive. If it is to the left, it will show
 *         negative.
 */
public double getYawAngleDegrees (ParticleReport target)
{
    int distFromCenterLine = (int) Math
            .abs((image.size().width / 2.0) - target.center.x);

    // The focal length is dependent on the resolution of the image, since
    // units must remain in pixels, and the field of view must not change.
    double focalLengthPixels = image.size().width
            / (2 * Math.tan(horizontalFieldOfView / 2.0));

    // Conditions for the return statement based on the position of the
    // target.
    if ((image.size().width / 2.0) - target.center.x < 0)
        return Math.toDegrees(
                Math.atan(distFromCenterLine / focalLengthPixels));

    return -Math.toDegrees(
            Math.atan(distFromCenterLine / focalLengthPixels));
} // end getYawAngleDegrees()

// -------------------------------------
// Max number of processed images allowed on the roboRIO
// -------------------------------------
private final int maxProcessedImagesAllowedToCollect = 29;

// -------------------------------------
// Max number of raw images allowed on the roboRIO
// -------------------------------------
private final int maxRawImagesAllowedToCollect = 25;

// private final int numImageFolders = 5;

private final static String timeStamp = new SimpleDateFormat("MMddHHmm")
        .format(new Date());

} // end class

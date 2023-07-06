package frc.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;


public class NewVisionInterface
{

NetworkTable limelight = NetworkTableInstance.getDefault()
        .getTable("limelight");

// has target
NetworkTableEntry tv = limelight.getEntry("tv");

// horizontal offset
NetworkTableEntry tx = limelight.getEntry("tx");

// vertical offset
NetworkTableEntry ty = limelight.getEntry("ty");

// % area
NetworkTableEntry ta = limelight.getEntry("ta");

// skew
NetworkTableEntry ts = limelight.getEntry("ts");

// latency
NetworkTableEntry tl = limelight.getEntry("tl");

// length of shortest side
NetworkTableEntry tshort = limelight.getEntry("tshort");

// length of longest side
NetworkTableEntry tlong = limelight.getEntry("tlong");

// horizontal side length (The "strongest" avenger)
NetworkTableEntry thor = limelight.getEntry("thor");

// vertical side length
NetworkTableEntry tvert = limelight.getEntry("tvert");

// active pipeline
NetworkTableEntry getpipe = limelight.getEntry("getpipe");

// 3d stuff
NetworkTableEntry camtran = limelight.getEntry("camtran");

NetworkTableEntry Led_Mode = limelight.getEntry("ledMode");

public boolean hasTargets;

public double x;

public double y;

public double area;

public double skew;

public double latency;

public double shortSide;

public double longSide;

public double horizontal;

public double vertical;

public double pipeline;

public double camtan;

public double led_Mode;



// to be called continuously
// updates internal values with those recieved from the network table
public void updateValues ()
{

    try
        {

        hasTargets = hasTargets(tv.getDouble(0));
        x = tx.getDouble(0);
        y = ty.getDouble(0);
        area = ta.getDouble(0);
        skew = ts.getDouble(0);
        latency = tl.getDouble(0);
        shortSide = tshort.getDouble(0);
        longSide = tlong.getDouble(0);
        horizontal = thor.getDouble(0);
        vertical = tvert.getDouble(0);
        pipeline = getpipe.getDouble(0);
        led_Mode = Led_Mode.getDouble(0);
        // filterBlobs();
        publishValues();
        }
    catch (NullPointerException exception)
        {
        System.out.println(exception);
        }

}

public double getXOffSet ()
{
    return x;

}

public double getYOffSet ()
{
    return y;
}

public double getArea ()
{
    return area;
}

public double getSkew ()
{
    return skew;
}

public double getLatency ()
{
    return latency;
}

public double getShortSide ()
{
    return shortSide;
}

public double getLongSide ()
{
    return longSide;
}

public double getHorizontalSide ()
{
    return horizontal;
}

public double getVerticalSide ()
{
    return vertical;
}

public double getPipeline ()
{
    return pipeline;
}

public void publishValues ()
{

    SmartDashboard.putBoolean("hasTargets", hasTargets);
    SmartDashboard.putNumber("x offset", x);
    SmartDashboard.putNumber("y offset", y);
    SmartDashboard.putNumber("skew ", skew);
    SmartDashboard.putNumber("latency ", latency);
    SmartDashboard.putNumber("shortside", shortSide);
    SmartDashboard.putNumber("longSide ", longSide);
    SmartDashboard.putNumber("horizontal ", horizontal);
    SmartDashboard.putNumber("vertical ", vertical);
    SmartDashboard.putNumber("pipeline ", pipeline);
    SmartDashboard.putNumber("ledMode", led_Mode);
    // SmartDashboard.putNumber("distance", getDistanceFromTarget());
}



public boolean hasTargets (double targets)
{
    if (targets != 0)
        {

        return true;
        }

    return false;

}

public enum LedMode
    {
    PIPELINE, OFF, BLINK, ON
    }

public LedMode ledmode = LedMode.PIPELINE;

// @parameters
// 0 pipeline control
// 1 off
// 2 blink
// 3 on
public void setLedMode (LedMode ledMode)
{
    switch (ledmode)
        {
        case PIPELINE:
            limelight.getEntry("ledMode").setNumber(0);
            break;
        case OFF:
            limelight.getEntry("ledMode").setNumber(1);
            break;
        case BLINK:
            limelight.getEntry("ledMode").setNumber(2);
            break;
        case ON:
            limelight.getEntry("ledMode").setNumber(3);
            break;
        }
}

public enum CamMode
    {
    PROCESSOR, CAMERA
    }

public CamMode camMode = CamMode.PROCESSOR;

// TODO make enum
// 0 vision processor
// 1 driver camera
public void setCamMode (int mode)
{
    switch (camMode)
        {
        case PROCESSOR:
            limelight.getEntry("camMode").setNumber(0);
            break;
        case CAMERA:
            limelight.getEntry("camMode").setNumber(1);
            break;
        }
}

// pipeline number
public void setPipeline (int pipe)
{
    limelight.getEntry("pipeling").setNumber(pipe);
}

public void takePicture ()
{
    // TODO
}

public boolean filterPass = true;

public void filterBlobs ()
{

    if (longSide > 50)
        {
        filterPass = false;
        }
    else
        {
        filterPass = true;
        }


}



public double distance = 0;

public double getDistanceFromTarget ()
{
    // alternating piplein to compare the different blob goups?

    // set pipeline()



    // double distance = 0;
    // d = (h2-h1) / tan(a1+a2)
    // distance = ((CAMERA_HEIGHT - TARGET_HEIGHT_LOW)
    // / Math.sin(Math.abs(getYOffSet())))
    // * Math.sin(90 - Math.abs(getYOffSet()));

    distance = 16.15 / Math.tan(Math.toRadians(Math.abs(getYOffSet())));

    if (hasTargets == true)
        {

        return distance;
        }
    else
        {
        return 0;
        }
}


public final double CAMERA_HEIGHT = 44.5;// TODO

final double TARGET_HEIGHT_LOW = 29;// TODO

final double TARGET_HEIGHT_HIGH = 35;// TODO

final double MOUNTING_ANGLE = 0;// TODO
}

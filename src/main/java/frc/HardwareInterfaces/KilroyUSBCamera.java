/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.HardwareInterfaces;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoProperty;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Class for controlling the USB cameras on the robot
 *
 * @Author Alice Marchant
 * @Written Feb 15th, 2020
 */
public class KilroyUSBCamera
    {

    /**
     * constructor
     *
     * @param twoFeeds
     *            - states whether we are using two camera feeds
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(boolean twoFeeds)
        {
            // If there are two feeds, declare and set values for two cameras.
            if (twoFeeds == true)
                {
                this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
                this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);
                this.server = CameraServer.getServer("serve_usb0");
                this.server1 = CameraServer.getServer("serve_usb1");
                setCameraValues(2);
                this.server1.getProperty("compression").set(COMPRESSION);
                }
            // If two feeds is false, only declare and set values for one camera
            else
                {
                this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
                this.server = CameraServer.getServer("serve_usb0");
                setCameraValues(1);
                }
        }

    /**
     * constructor
     *
     * @param twoFeeds
     *            - states whether we are using two camera feeds
     * @param width
     *            - the width
     * @param height
     *            - the height
     * @param FPS
     *            - the fps of the cameras
     * @param compression
     *            - the compression rate
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(boolean twoFeeds, int width, int height, int FPS, int compression)
        {
            // If there are two feeds, declare and set values for two cameras.
            if (twoFeeds == true)
                {
                this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
                this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);
                this.server = CameraServer.getServer("serve_usb0");
                this.server1 = CameraServer.getServer("serve_usb1");
                setCameraValues(width, height, FPS, compression, 2);
                this.server1.getProperty("compression").set(compression);
                }
            // If two feeds is false, only declare and set values for one camera
            else
                {
                this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
                this.server = CameraServer.getServer("serve_usb0");
                setCameraValues(width, height, FPS, compression, 1);
                }
        }

    /**
     * constructor
     *
     * @param button
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(MomentarySwitch button)
        {
            // Declares and sets values for two cameras
            this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
            this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);
            CameraServer.removeServer("serve_usb1");
            this.server = CameraServer.getServer("serve_usb0");
            setCameraValues(2);
            this.button = button;
        }

    /**
     * constructor
     *
     * @param button
     * @param width
     *            - the width
     * @param height
     *            - the height
     * @param FPS
     *            - the fps of the cameras
     * @param compression
     *            - the compression rate
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(MomentarySwitch button, int width, int height, int FPS, int compression)
        {
            // Declares and sets values for two cameras
            this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
            this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);

            CameraServer.removeServer("serve_usb1");
            this.server = CameraServer.getServer("serve_usb0");
            setCameraValues(width, height, FPS, compression, 2);
            this.button = button;
        }

    /**
     * constructor
     *
     * @param switch1
     *            - button
     * @param switch2
     *            - button
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(MomentarySwitch switch1, MomentarySwitch switch2)
        {
            // Declares and sets values for two cameras
            this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
            this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);
            CameraServer.removeServer("serve_usb1");
            this.server = CameraServer.getServer("serve_usb0");
            setCameraValues(2);
            this.switch1 = switch1;
            this.switch2 = switch2;
        }

    /**
     * constructor
     *
     * @param switch1
     *            - button
     * @param switch2
     *            - button
     * @param width
     *            - the width
     * @param height
     *            - the height
     * @param FPS
     *            - the fps of the cameras
     * @param compression
     *            - the compression rate
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(MomentarySwitch switch1, MomentarySwitch switch2, int width, int height, int FPS,
            int compression)
        {
            // Declares and sets values for two cameras
            this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
            this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);
            CameraServer.removeServer("serve_usb1");
            this.server = CameraServer.getServer("serve_usb0");
            setCameraValues(width, height, FPS, compression, 2);
            this.switch1 = switch1;
            this.switch2 = switch2;
        }

    /**
     * constructor
     *
     * @param button1
     * @param button2
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(JoystickButton button1, JoystickButton button2)
        {
            // Declares and sets values for two cameras
            this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
            this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);

            CameraServer.removeServer("serve_usb1");
            this.server = CameraServer.getServer("serve_usb0");
            setCameraValues(2);
            this.button1 = button1;
            this.button2 = button2;
        }

    /**
     * constructor
     *
     * @param button1
     * @param button2
     * @param width
     *            - the width
     * @param height
     *            - the height
     * @param FPS
     *            - the fps of the cameras
     * @param compression
     *            - the compression rate
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public KilroyUSBCamera(JoystickButton button1, JoystickButton button2, int width, int height, int FPS,
            int compression)
        {
            // Declares and sets values for two cameras
            this.cam0 = CameraServer.startAutomaticCapture("usb0", 0);
            this.cam1 = CameraServer.startAutomaticCapture("usb1", 1);
            CameraServer.removeServer("serve_usb1");
            this.server = CameraServer.getServer("serve_usb0");
            setCameraValues(width, height, FPS, compression, 2);
            this.button1 = button1;
            this.button2 = button2;
        }

    /**
     * Gets the compression rate of the server
     *
     * @return the compression rate
     * 
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public int getCompression()
    {
        return this.server.getProperty("compression").get();
    }

    /**
     * Gets the fps of a given camera
     *
     * @param cameraNum
     *            - which camera the method gets the value for
     * @return the fps of the camera
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public double getFPS(int cameraNum)
    {
        // If camNum is zero, get the fps of cam0
        if (cameraNum == 0)
            {
            return this.cam0.getActualFPS();
            }
        // If camNum is one, get the fps of cam1
        else if (cameraNum == 1)
            {
            return this.cam1.getActualFPS();
            }
        // If neither, return 0
        else
            {
            return 0.0;
            }
    }

    /**
     * Sets a given camera to be diplayed to the driver's station
     *
     * @param cameraNum
     *            - which camera to set the source to
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public void setCamera(int cameraNum)
    {
        // If the int passed in is 0, switch source to cam0. If int passed in is 1,
        // switch source to cam1
        if (cameraNum == 0)
            {
            this.server.setSource(this.cam0);
            }
        if (cameraNum == 1)
            {
            this.server.setSource(this.cam1);
            }
    }

    /**
     * Sets the values for fps, resolution, and compression on the camera(s)
     *
     * @param numCameras
     *            - the amount of cameras to set the values for
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public void setCameraValues(int numCameras)
    {
        if (numCameras == 1)
            {
            this.cam0.setResolution(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
            this.cam0.setFPS(CAMERA_FPS);
            this.server.getProperty("compression").set(COMPRESSION);
            }
        else if (numCameras == 2)
            {
            this.cam0.setResolution(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
            this.cam1.setResolution(RESOLUTION_WIDTH, RESOLUTION_HEIGHT);
            this.cam0.setFPS(CAMERA_FPS);
            this.cam1.setFPS(CAMERA_FPS);
            this.server.getProperty("compression").set(COMPRESSION);
            }
    }

    /**
     * Passes in the values for fps, resolution, and compression on the camera(s)
     *
     * @param width
     *            - the width of the resolution
     * @param height
     *            - the height of the resolution
     * @param FPS
     *            - the fps of the camera(s)
     * @param compression
     *            - the compression rate of the server
     * @param numCameras
     *            - the amount of cameras to set the values for
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public void setCameraValues(int width, int height, int FPS, int compression, int numCameras)
    {
        if (numCameras == 1)
            {
            this.cam0.setResolution(width, height);
            this.cam0.setFPS(FPS);
            this.server.getProperty("compression").set(compression);
            }
        else if (numCameras == 2)
            {
            this.cam0.setResolution(width, height);
            this.cam1.setResolution(width, height);
            this.cam0.setFPS(FPS);
            this.cam1.setFPS(FPS);
            this.server.getProperty("compression").set(compression);
            }
    }

    /**
     * Toggles which camera is being displayed on the driver's station
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public void switchCameras()
    {
        // If cam1 is not null, switch cameras based on which is on at the time of
        // calling
        if (this.cam1 != null)
            {
            if (this.cam0.isEnabled())
                {
                this.server.setSource(this.cam1);
                }
            else
                {
                this.server.setSource(this.cam0);
                }
            }
    }

    /**
     * Toggles the cameras with a momentary switch
     *
     * @param button
     *            - button used to switch the cameras
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public void switchCameras(MomentarySwitch button)
    {
        // Whenever the value of the momentary switch switches from false to true or
        // true to false, it calls the switchCameras() method
        if (this.button.isOnCheckNow() && firstCheck == true)
            {
            switchCameras();
            firstCheck = false;
            }
        if (this.button.isOnCheckNow() == false && firstCheck != true)
            {
            switchCameras();
            firstCheck = true;
            }
    }

    /**
     * Use two buttons that each toggle the cameras
     *
     * @param switch1
     *            - button used to toggle the cameras
     * @param switch2
     *            - other button used to toggle the cameras\
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public void switchCameras(MomentarySwitch switch1, MomentarySwitch switch2)
    {
        // Whenever the value of either momentary switch switches from false to true or
        // true to false, it calls the switchCameras() method
        if (this.switch1.isOnCheckNow() && firstCheck == true)
            {
            switchCameras();
            firstCheck = false;
            }
        if (this.switch1.isOnCheckNow() == false && firstCheck != true)
            {
            switchCameras();
            firstCheck = true;
            }
        if (this.switch2.isOnCheckNow() && firstCheck2 == true)
            {
            switchCameras();
            firstCheck2 = false;
            }
        if (this.switch2.isOnCheckNow() == false && firstCheck2 != true)
            {
            switchCameras();
            firstCheck2 = true;
            }
    }

    /**
     * button1 sets camera1 to be displayed to the driver's station, button2 sets
     * camera0 to be displayed to the driver's station
     *
     * @param button1
     *            - button used to switch the source to cam1
     * @param button2
     *            - button used to switch the source to cam0
     *
     * @Author Alice Marchant
     * @Written Feb 15th, 2020
     */
    public void switchCameras(JoystickButton button1, JoystickButton button2)
    {
        // If button1 is pressed, switch source to cam1. If button2 is pressed, switch
        // source to cam0
        if (this.cam1 != null)
            {
            if (this.button1.get())
                {
                server.setSource(this.cam1);
                }
            if (this.button2.get())
                {
                server.setSource(this.cam0);
                }
            }
    }

    /**
     * Sets the limelight as the server source
     */
    public void setLimelight()
    {
        server.setSource(limelight);
        for (VideoProperty x : server.enumerateProperties())
            System.out.println("Name: " + x.getName() + "Val: " + x.get());
    }

    // Variables

    private UsbCamera cam0 = null;

    private UsbCamera cam1 = null;

    private HttpCamera limelight = new HttpCamera("Limelight Preveiw", "http://limelight.local:5800/");

    private VideoSink server;

    private VideoSink server1;

    private MomentarySwitch button = null;

    private MomentarySwitch switch1 = null;

    private MomentarySwitch switch2 = null;

    private JoystickButton button1 = null;

    private JoystickButton button2 = null;

    private boolean firstCheck = true;

    private boolean firstCheck2 = true;

    // Constants

    private int CAMERA_FPS = 20;

    private int COMPRESSION = 30;

    private int RESOLUTION_WIDTH = 340;

    private int RESOLUTION_HEIGHT = 240;
    // end class
    }

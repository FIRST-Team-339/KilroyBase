package frc.Utils.drive;

import frc.HardwareInterfaces.KilroyEncoder;
import frc.HardwareInterfaces.Transmission.TransmissionBase;
import frc.HardwareInterfaces.Transmission.TransmissionBase.MotorPosition;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * An extension of the Drive class that overrides the methods to use PID loops
 * instead of static-correcting methods.
 *
 * @author Ryan McGee
 *
 */
public class DrivePID extends Drive
    {

    /**
     * Create the DrivePID Object with a 4 encoder system
     *
     * @param transmission
     * @param leftFrontEncoder
     * @param rightFrontEncoder
     * @param leftRearEncoder
     * @param rightRearEncoder
     * @param gyro
     */
    public DrivePID(TransmissionBase transmission, KilroyEncoder leftFrontEncoder, KilroyEncoder rightFrontEncoder,
            KilroyEncoder leftRearEncoder, KilroyEncoder rightRearEncoder, ADXRS450_Gyro gyro)
        {
            // Create the Drive class this is extending, as to override the methods
            // to use PID instead of static constants.
            super(transmission, leftFrontEncoder, rightFrontEncoder, leftRearEncoder, rightRearEncoder, gyro);

            initPIDControllers();

            this.encoders = new KilroyEncoder[4];
            encoders[0] = leftRearEncoder;
            encoders[1] = rightRearEncoder;
            encoders[2] = leftFrontEncoder;
            encoders[3] = rightFrontEncoder;
        }

    /**
     * Create the DrivePID Object with a 2 encoder system
     *
     * @param transmission
     * @param leftEncoder
     * @param rightEncoder
     * @param gyro
     */
    public DrivePID(TransmissionBase transmission, KilroyEncoder leftEncoder, KilroyEncoder rightEncoder,
            ADXRS450_Gyro gyro)
        {
            super(transmission, leftEncoder, rightEncoder, gyro);
            initPIDControllers();
            this.encoders = new KilroyEncoder[2];
            this.encoders[0] = leftEncoder;
            this.encoders[1] = rightEncoder;
        }

    /**
     * Removes the PID controllers from the shuffleboard list to make sure that
     * there is no confusion when tuning PID loops on which PID to tune. Instead the
     * custom PIDTuner class will be used to display / input PID values, as it adds
     * in speed, tolerance and acceleration. (You will need to drag those values out
     * separately from the widget)
     */
    private void initPIDControllers()
    {
        LiveWindow.disableTelemetry(driveStraightInchesPID);
        LiveWindow.disableTelemetry(driveStraightPID_enc);
        LiveWindow.disableTelemetry(driveStraightPID_gyro);
        LiveWindow.disableTelemetry(turnDegreesPID_enc);
        LiveWindow.disableTelemetry(turnDegreesPID_gyro);
        LiveWindow.disableTelemetry(brakePID);
    }

    /**
     * Resets the all drive function's initialization.
     *
     * @return
     */
    public void reset()
    {
        super.reset();
        this.turnDegreesInit = true;
        this.turnDegreesGyroInit = true;
        this.driveStraightInchesInit = true;
        this.brakeInit = true;
        // this.turnGyroPID.disable();
    }

    /**
     * Sets the p, i, and d of the drive function selected for autonomous driving.
     *
     * @param driveFunction
     *            Which PID loop will be tuned
     * @param p
     *            the Proportional value
     * @param i
     *            the Integral value
     * @param d
     *            the Derivative value
     * @param tolerance
     *            How far when we are considered "on target", in default sensor
     *            units.
     *
     * @param accel
     *            the acceleration part of the motion profile. This is only useful
     *            for turns, as the driving acceleration
     */
    public void setPIDToleranceAccel(PIDDriveFunction driveFunction, double p, double i, double d, double tolerance,
            double accel)
    {
        switch (driveFunction)
            {
            case BRAKE:
                this.brakePIDTolerance[0] = p;
                this.brakePIDTolerance[1] = i;
                this.brakePIDTolerance[2] = d;
                this.brakePIDTolerance[3] = tolerance;
                break;
            case DRIVESTRAIGHT_ENC:
                this.driveStraightEncPIDTolerance[0] = p;
                this.driveStraightEncPIDTolerance[1] = i;
                this.driveStraightEncPIDTolerance[2] = d;
                this.driveStraightEncPIDTolerance[3] = tolerance;
                return;
            case DRIVESTRAIGHT_GYRO:
                this.driveStraightGyroPIDTolerance[0] = p;
                this.driveStraightGyroPIDTolerance[1] = i;
                this.driveStraightGyroPIDTolerance[2] = d;
                this.driveStraightGyroPIDTolerance[3] = tolerance;
                return;
            case TURN_ENC:
                this.turnPIDToleranceAccel[0] = p;
                this.turnPIDToleranceAccel[1] = i;
                this.turnPIDToleranceAccel[2] = d;
                this.turnPIDToleranceAccel[3] = tolerance;
                this.turnPIDToleranceAccel[4] = accel;
                return;
            case TURN_GYRO:
                this.turnGyroPIDToleranceAccel[0] = p;
                this.turnGyroPIDToleranceAccel[1] = i;
                this.turnGyroPIDToleranceAccel[2] = d;
                this.turnGyroPIDToleranceAccel[3] = tolerance;
                this.turnGyroPIDToleranceAccel[4] = accel;
                return;
            case DRIVESTRAIGHTINCHES:
                this.driveStraightInchesPIDTolerance[0] = p;
                this.driveStraightInchesPIDTolerance[1] = i;
                this.driveStraightInchesPIDTolerance[2] = d;
                this.driveStraightInchesPIDTolerance[3] = tolerance;
            default:
                return;
            }
    }

    /**
     * Brakes the robot using a PID loop, setting the setpoint to 0.
     *
     * @Deprecated This was a test function, and is BAD. do not use.
     * @return
     */
    public boolean brakePID(BrakeType brakeType)
    {
        if (brakeInit == true)
            {
            this.brakeType = brakeType;
            super.resetEncoders();
            this.brakePID.getPIDController().setPID(brakePIDTolerance[0], brakePIDTolerance[1], brakePIDTolerance[2]);
            this.brakePID.getPIDController().setAbsoluteTolerance(brakePIDTolerance[3]);
            this.brakePID.getPIDController().reset();
            this.brakePID.setSetpoint(0);
            this.brakePID.enable();
            brakeInit = false;
            }

        if (brakeType == BrakeType.AFTER_DRIVE)
            getTransmission().driveRaw(brakePIDOut, brakePIDOut);
        else if (brakeType == BrakeType.AFTER_TURN)
            getTransmission().driveRaw(brakePIDOut, -brakePIDOut);

        if (brakePID.onTarget() == true)
            {
            stop();
            brakePID.disable();
            brakeInit = true;
            return true;
            }
        return false;

    }

    /**
     * Turns the robot X degrees on the spot, using left and right encoders.
     *
     * @param degrees
     *            How far the robot should turn, in degrees. Positive for clockwise,
     *            negative for counterclockwise.
     *
     * @param speed
     *            How fast the robot should turn, in positive percentage.
     *
     * @return whether or not the robot has finished turning.
     */
    public boolean turnDegrees(int degrees, double speed)
    {
        if (turnDegreesInit == true)
            {
            super.resetEncoders();
            this.turnDegreesPID_enc.getPIDController().reset();
            this.turnDegreesPID_enc.getPIDController().setPID(turnPIDToleranceAccel[0], turnPIDToleranceAccel[1],
                    turnPIDToleranceAccel[2]);
            this.turnDegreesPID_enc.setAbsoluteTolerance(turnPIDToleranceAccel[3]);
            this.turnDegreesPID_enc.setOutputRange(-speed, speed);
            this.turnDegreesPID_enc.setSetpoint(degrees);
            this.turnDegreesPID_enc.enable();
            super.reset();
            turnDegreesInit = false;
            }
        accelerateProportionaly(turnDegreesPIDOut, -turnDegreesPIDOut, turnPIDToleranceAccel[4]);

        if (turnDegreesPID_enc.onTarget() == true)
            {
            stop();
            turnDegreesPID_enc.disable();
            turnDegreesInit = true;
            }

        return false;
    }

    /**
     * Turns the robot on the spot based on the gyro as a sensor, rather than the
     * encoders.
     *
     * @param degrees
     *            how far the robot should turn. Positive for clockwise, negative
     *            for counterclockwise.
     *
     * @param speed
     *            the maximum speed that the robot is allowed to travel at.
     *
     * @return whether or not the robot has finished turning
     */
    public boolean turnDegreesGyro(int degrees, double speed)
    {
        if (turnDegreesGyroInit == true)
            {
            turnDegreesPID_gyro.getPIDController().setPID(turnGyroPIDToleranceAccel[0], turnGyroPIDToleranceAccel[1],
                    turnGyroPIDToleranceAccel[2]);
            turnDegreesPID_gyro.getPIDController().setAbsoluteTolerance(turnGyroPIDToleranceAccel[3]);
            turnDegreesPID_gyro.setOutputRange(-speed, speed);
            turnDegreesPID_gyro.getPIDController().reset();
            turnDegreesPID_gyro.setSetpoint(degrees);
            turnDegreesPID_gyro.enable();
            super.getGyro().reset();
            turnDegreesGyroInit = false;
            }

        accelerateProportionaly(turnDegreesGyroPIDOut, -turnDegreesGyroPIDOut, turnGyroPIDToleranceAccel[4]);

        if (turnDegreesPID_gyro.onTarget() == true)
            {
            stop();
            turnDegreesPID_gyro.disable();
            turnDegreesGyroInit = true;
            return true;
            }
        return false;
    }

    /**
     * Drives in a straight line based on encoder values.
     *
     * @param speed
     *            How fast the robot will be running forwards / backwards.
     * @param acceleration
     *            whether or not to accelerate at a fixed rate
     * @param isUsingGyro
     *            If true, then the PID loop will use a gyro as the sensor for
     *            correction. If false, it will use encoders.
     */
    public void driveStraight(double speed, double acceleration, boolean isUsingGyro)
    {
        if (System.currentTimeMillis() - driveStraightLastTime > INIT_TIMEOUT)
            {
            if (isUsingGyro == true)
                {
                this.driveStraightPID_gyro.getPIDController().reset();
                this.driveStraightPID_gyro.getPIDController().setPID(driveStraightGyroPIDTolerance[0],
                        driveStraightGyroPIDTolerance[1], driveStraightGyroPIDTolerance[2]);
                this.driveStraightPID_gyro.setSetpoint(0);
                this.driveStraightPID_gyro.enable();
                super.getGyro().reset();
                }
            else
                {
                this.driveStraightPID_enc.getPIDController().reset();
                this.driveStraightPID_enc.getPIDController().setPID(driveStraightEncPIDTolerance[0],
                        driveStraightEncPIDTolerance[1], driveStraightEncPIDTolerance[2]);
                this.driveStraightPID_enc.setSetpoint(0);
                this.driveStraightPID_enc.enable();
                }
            }
        if (isUsingGyro == true)
            super.accelerateProportionaly(speed + driveStraightPIDOutput_gyro, speed - driveStraightPIDOutput_gyro,
                    acceleration);
        else
            super.accelerateProportionaly(speed + driveStraightPIDOutput_enc, speed - driveStraightPIDOutput_enc,
                    acceleration);

        driveStraightLastTime = System.currentTimeMillis();
    }

    /**
     * Drives a set number of inches forwards in a straight line, based on encoder
     * distance, correcting for misalignment based on Gyro and encoder PID tunings.
     *
     * If tuned properly, this will give the robot a nice acceleration and
     * deceleration curve, while correcting it's position.
     *
     * @param speed
     *            The robot's maximum speed after acceleration, in percentage
     * @param distance
     *            How far the robot must travel, in inches
     * @param acceleration
     *            How fast to accelerate from 0, in seconds
     * @param isUsingGyro
     *            Whether or not the driveStraight part of this is using the gyro
     *            for correction. If false, the encoders are used.
     * @return Whether or not the robot has reached it's destination.
     */
    public boolean driveStraightInches(double speed, double distance, double acceleration, boolean isUsingGyro)
    {
        // On initialization, reset encoders and the PID controller, set
        // PIDTolerance values and setpoint,
        /// set maximum speed, and begin moving.
        if (driveStraightInchesInit == true)
            {
            resetEncoders();
            driveStraightInchesPID.getPIDController().setPID(driveStraightInchesPIDTolerance[0],
                    driveStraightInchesPIDTolerance[1], driveStraightInchesPIDTolerance[2]);
            driveStraightInchesPID.setAbsoluteTolerance(driveStraightInchesPIDTolerance[3]);
            driveStraightInchesPID.getPIDController().reset();
            driveStraightInchesPID.setSetpoint(distance);
            driveStraightInchesPID.setOutputRange(-speed, speed);
            driveStraightInchesPID.enable();
            driveStraightInchesInit = false;
            }

        // If we have reached the setpoint, then disable the PID loop and stop
        // moving.
        if (driveStraightInchesPID.onTarget() == true)
            {
            this.stop();
            driveStraightInchesPID.disable();
            driveStraightInchesInit = true;
            return true;
            }

        // If we have not reached the setpoint, then keep driving.
        driveStraight(this.driveStraightInchesSpeed, acceleration, isUsingGyro);

        return false;
    }

    // ======================PID Tuning=====================

    /*
     * Tunes the PID loop for turning via encoders.
     *
     * P - Proportional function: Value adds P percent per inch the encoder is off
     * currently.
     *
     * I - Integral function: Value adds I percent per inch the encoder has been off
     * cumulatively
     *
     * D - Derivative function: Value adds D percent per inch per second the error
     * is changing in relation to the setpoint (slope of current position vs error
     * get smaller as the controller slows down to reach it's target)
     *
     * A high P value will give you a more aggressive correction, but may induce
     * oscillation: Try to avoid that... A high I value will correct any long term
     * "drifting to a side" problems, but again may induce oscillation. A high D
     * value will give you a longer drawn out deceleration curve, and may fix
     * oscillation, but will make the robot take longer reach it's setpoint.
     *
     * In order to tune, start by changing P. Think of P like this: for every 1
     * sensor unit, x percent will be added. After P, start tuning D. End with I,
     * and only use it where you see fit. I is great for longer, more precise
     * movements, and must be small.
     *
     * A low tolerance will result in more accurate turns, but may induce a little
     * "wiggle" at the end (which is not bad, but takes more time.)
     *
     * The point of using a PID loop for turning is to increase the speed of the
     * turn and reduce overshoot / undershoot.
     *
     * ...Good luck...
     *
     */

    /**
     * Tunes the PID controllers for the drive fuctions of the robot
     *
     * @param type
     *            Which driving function is being tuned
     * @return whether the PID is enabled or not.
     */
    public boolean tunePID(PIDDriveFunction type)
    {
        switch (type)
            {
            case BRAKE:
                this.brakePIDTolerance[0] = brakeTuner.p;
                this.brakePIDTolerance[1] = brakeTuner.i;
                this.brakePIDTolerance[2] = brakeTuner.d;
                this.brakePIDTolerance[3] = brakeTuner.tolerance;
                break;
            case TURN_ENC:
                this.turnPIDToleranceAccel[0] = turnDegreesTuner_enc.p;
                this.turnPIDToleranceAccel[1] = turnDegreesTuner_enc.i;
                this.turnPIDToleranceAccel[2] = turnDegreesTuner_enc.d;
                this.turnPIDToleranceAccel[3] = turnDegreesTuner_enc.tolerance;
                this.turnPIDToleranceAccel[4] = turnDegreesTuner_enc.acceleration;
                break;
            case TURN_GYRO:
                this.turnGyroPIDToleranceAccel[0] = turnDegreesTuner_gyro.p;
                this.turnGyroPIDToleranceAccel[1] = turnDegreesTuner_gyro.i;
                this.turnGyroPIDToleranceAccel[2] = turnDegreesTuner_gyro.d;
                this.turnGyroPIDToleranceAccel[3] = turnDegreesTuner_gyro.tolerance;
                this.turnGyroPIDToleranceAccel[4] = turnDegreesTuner_gyro.acceleration;
                break;
            case DRIVESTRAIGHT_ENC:
                this.driveStraightEncPIDTolerance[0] = driveStraightTuner_enc.p;
                this.driveStraightEncPIDTolerance[1] = driveStraightTuner_enc.i;
                this.driveStraightEncPIDTolerance[2] = driveStraightTuner_enc.d;
                this.driveStraightEncPIDTolerance[3] = driveStraightTuner_enc.tolerance;
                break;
            case DRIVESTRAIGHT_GYRO:
                this.driveStraightGyroPIDTolerance[0] = driveStraightTuner_gyro.p;
                this.driveStraightGyroPIDTolerance[1] = driveStraightTuner_gyro.i;
                this.driveStraightGyroPIDTolerance[2] = driveStraightTuner_gyro.d;
                this.driveStraightGyroPIDTolerance[3] = driveStraightTuner_gyro.tolerance;
                break;
            case DRIVESTRAIGHTINCHES:
                this.driveStraightInchesPIDTolerance[0] = driveInchesTuner.p;
                this.driveStraightInchesPIDTolerance[1] = driveInchesTuner.i;
                this.driveStraightInchesPIDTolerance[2] = driveInchesTuner.d;
                this.driveStraightInchesPIDTolerance[3] = driveInchesTuner.tolerance;
                break;
            default:
                return false;
            }
        return false;
    }

    /**
     * @return Whether or not the PID loops are being tuned.
     */
    public boolean isTuningPID()
    {
        return isTuningPID;
    }

    /**
     * Describes the different drive-by-pid functions
     *
     * @author Ryan McGee
     */
    public enum PIDDriveFunction
        {
        BRAKE,
        /** Turning via encoders */
        TURN_ENC,
        /** Turning via gyro sensor */
        TURN_GYRO,
        /** Driving straight via encoders */
        DRIVESTRAIGHT_ENC,
        /** Driving straight via gyro sensor */
        DRIVESTRAIGHT_GYRO,
        /** Driving straight X number of inches */
        DRIVESTRAIGHTINCHES
        }

    private final PIDSubsystem brakePID = new PIDSubsystem(0, 0, 0)
        {
        @Override
        protected double returnPIDInput()
        {
            if (brakeType == BrakeType.AFTER_DRIVE)
                return getEncoderRate(MotorPosition.ALL);
            else if (brakeType == BrakeType.AFTER_TURN)
                return getEncoderRate(MotorPosition.LEFT);
            // If all else fails, return 0
            return 0;
        }

        @Override
        protected void usePIDOutput(double output)
        {
            brakePIDOut = output;
        }

        @Override
        protected void initDefaultCommand()
        {
        }

        };

    private final PIDSubsystem turnDegreesPID_enc = new PIDSubsystem(0, 0, 0)
        {

        @Override
        protected double returnPIDInput()
        {
            return getEncoderDegreesTurned();
        }

        @Override
        protected void usePIDOutput(double output)
        {
            turnDegreesPIDOut = output;
        }

        @Override
        protected void initDefaultCommand()
        {

        }
        };

    private final PIDSubsystem turnDegreesPID_gyro = new PIDSubsystem(0, 0, 0)
        {

        @Override
        protected void initDefaultCommand()
        {

        }

        @Override
        protected void usePIDOutput(double output)
        {
            turnDegreesGyroPIDOut = output;
        }

        @Override
        protected double returnPIDInput()
        {
            return getGyro().getAngle();
        }
        };

    /**
     * The PID controller behind the driveStraight function when using encoders
     */
    private final PIDSubsystem driveStraightPID_enc = new PIDSubsystem(0, 0, 0)
        {

        @Override
        protected double returnPIDInput()
        {
            return getEncoderDistanceAverage(MotorPosition.LEFT) - getEncoderDistanceAverage(MotorPosition.RIGHT);
        }

        @Override
        protected void usePIDOutput(double output)
        {
            driveStraightPIDOutput_enc = output;
        }

        @Override
        protected void initDefaultCommand()
        {
        }

        };

    /**
     * The PID controller behind the driveStraight function when using the
     * gyroscopic sensor
     */
    private final PIDSubsystem driveStraightPID_gyro = new PIDSubsystem(0, 0, 0)
        {
        @Override
        protected double returnPIDInput()
        {
            return getGyro().getAngle();
        }

        @Override
        protected void usePIDOutput(double output)
        {
            driveStraightPIDOutput_gyro = output;
        }

        @Override
        protected void initDefaultCommand()
        {
        }
        };

    /**
     * The PID loop behind the driveStraightInches function, for distance
     */
    private final PIDSubsystem driveStraightInchesPID = new PIDSubsystem(0, 0, 0)
        {

        @Override
        protected double returnPIDInput()
        {
            return (getEncoderDistanceAverage(MotorPosition.LEFT) + getEncoderDistanceAverage(MotorPosition.RIGHT))
                    / 2.0;
        }

        @Override
        protected void usePIDOutput(double output)
        {
            driveStraightInchesSpeed = output;
        }

        @Override
        protected void initDefaultCommand()
        {
        }

        };

    private final KilroyEncoder[] encoders;

    // ======================Variables======================

    private PIDTuner brakeTuner = new PIDTuner("Braking");

    private PIDTuner driveStraightTuner_gyro = new PIDTuner("Drive Straight Gyro");

    private PIDTuner driveStraightTuner_enc = new PIDTuner("Drive Straight Encoder");

    private PIDTuner driveInchesTuner = new PIDTuner("Drive Inches PID");

    private PIDTuner turnDegreesTuner_enc = new PIDTuner("Turn Degrees Encoder");

    private PIDTuner turnDegreesTuner_gyro = new PIDTuner("Turn Degrees Gyro");

    private boolean isTuningPID = false;

    private double[] brakePIDTolerance =
            // {P, I, D, Tolerance}
            { 0, 0, 0, 0 };
    private boolean brakeInit = true;

    private double brakePIDOut = 0;

    private BrakeType brakeType = BrakeType.AFTER_DRIVE;

    private double[] turnPIDToleranceAccel =
            // {P, I, D, Tolerance, Acceleration}
            { 0, 0, 0, 0, 0 };

    private boolean turnDegreesInit = true;

    private double turnDegreesPIDOut = 0;

    private double[] turnGyroPIDToleranceAccel =
        { 0, 0, 0, 0, 0 };

    private boolean turnDegreesGyroInit = true;

    private double turnDegreesGyroPIDOut = 0;

    private double driveStraightPIDOutput_enc = 0;

    private long driveStraightLastTime = 0;

    private double[] driveStraightEncPIDTolerance =
            // {P, I, D, Tolerance}
            { 0, 0, 0, 0 };

    private double[] driveStraightGyroPIDTolerance =
        { 0, 0, 0, 0 };

    private double driveStraightPIDOutput_gyro = 0;

    private boolean driveStraightInchesInit = true;

    private double driveStraightInchesSpeed = 0;

    private double[] driveStraightInchesPIDTolerance =
        { 0, 0, 0, 0 };

    /**
     * A class designed to use the Shuffleboard built in tuner for PID loops. This
     * is useful if you have multiple PID controllers that all need the same tuning
     * at once, so you don't have to change 24 values for a 4 wheel robot. When
     * values are changed in the shuffleboard, the stored values
     * p,i,d,setpoint,tolerance,speed, and enabled will reflect it.
     *
     * You are able to get a specific widget from this by finding the "LiveWindow"
     * tab on the left, and finding the name of the PID controller (for example,
     * "EncoderPID") and dragging it onto the screen, if there's not already a saved
     * view for it
     *
     * @author Ryan McGee
     * @written 6/2018
     *
     */
    class PIDTuner
        {
        // Store the P (proportional), I (integral), D (derivative), setpoint,
        // tolerance, speed and enabled in variables inside
        // the object
        double p, i, d, setpoint, tolerance, speed, acceleration;

        boolean enabled;

        /**
         * Creates the PIDTuner class, sets the name and creates / sends the PID widget
         * to shuffleboard.
         *
         * @param name
         *            what the PID tuner will show up as in the list under "LiveWindow".
         */
        public PIDTuner(String name)
            {
                SendableRegistry.setName(this.sendable, name);
                // TODO implement new SendableBuilder to allow PID tuning, Im too lazy to do
                // this right now.
                // TODO --------- McGee fix -------
            }

        public Sendable getSendable()
        {
            return sendable;
        }

        // Creating the sendable creates the widget and sends the values to the
        // shuffleboard.
        Sendable sendable = new Sendable()
            {
            @Override
            public void initSendable(SendableBuilder builder)
            {
                // Telling shuffleboard we want this specific widget
                builder.setSmartDashboardType("PIDController");

                // the "() -> variable" is a way of sending the variable to
                // shuffleboard
                // the "(arg0) -> variable = arg0" is a way of getting the
                // variable from the shuffleboard
                // PS the p, i, and d are CASE SENSITIVE! DO NOT CHANGE!
                builder.addDoubleProperty("p", () -> p, (arg0) -> p = arg0);
                builder.addDoubleProperty("i", () -> i, (arg0) -> i = arg0);
                builder.addDoubleProperty("d", () -> d, (arg0) -> d = arg0);
                builder.addDoubleProperty("Setpoint", () -> setpoint, (arg0) -> setpoint = arg0);
                builder.addDoubleProperty("Speed", () -> speed, (arg0) -> speed = arg0);
                builder.addDoubleProperty("Tolerance", () -> tolerance, (arg0) -> tolerance = arg0);
                builder.addDoubleProperty("Acceleration", () -> acceleration, (arg0) -> acceleration = arg0);
                builder.addBooleanProperty("Enabled", () -> enabled, (arg0) -> enabled = arg0);
            }
            };

        }

    }

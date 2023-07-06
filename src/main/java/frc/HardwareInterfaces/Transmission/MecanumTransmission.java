package frc.HardwareInterfaces.Transmission;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.pidwrappers.PIDAnalogGyro;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * One of the more complex drive systems: uses 4 mecanum wheels to allow
 * for lateral movement as well as normal tank controls.
 * 
 * @author Ryan McGee
 * @written 7/21/17
 */
public class MecanumTransmission extends TransmissionBase
{
/**
 * Creates the MecanumTransmission object.
 * 
 * @param leftFrontMotor
 *            The left front motor controller
 * @param rightFrontMotor
 *            The right front motor controller
 * @param leftRearMotor
 *            The left rear motor controller
 * @param rightRearMotor
 *            The right rear motor controller
 */
public MecanumTransmission (MotorController leftRearMotor,
        MotorController rightRearMotor,
        MotorController leftFrontMotor, MotorController rightFrontMotor)
{
    super(leftRearMotor, rightRearMotor, leftFrontMotor,
            rightFrontMotor);
    super.type = TransmissionType.OMNI_DIR;
}

/**
 * Creates the Mecanum Transmission class with an added gyro sensor, for PID
 * tuning
 * functionality and field-oriented driving.
 * 
 * @param leftFrontMotor
 *            The left front motor controller
 * @param rightFrontMotor
 *            The right front motor controller
 * @param leftRearMotor
 *            The left rear motor controller
 * @param rightRearMotor
 *            The right rear motor controller
 * @param gyro
 *            A gyroscopic sensor.
 */
public MecanumTransmission (MotorController leftRearMotor,
        MotorController rightRearMotor,
        MotorController leftFrontMotor, MotorController rightFrontMotor,
        PIDAnalogGyro gyro)
{
    super(leftRearMotor, rightRearMotor, leftFrontMotor,
            rightFrontMotor);
    this.gyro = gyro;
    super.type = TransmissionType.OMNI_DIR;
}

/**
 * Drives the robot with mecanum with raw values, taking into account
 * joystick deadbands and gear ratios.
 * 
 * 
 * @param magnitude
 *            The magnitude of the joystick, (0.0 to 1.0)
 * @param direction
 *            The direction of the joystick in radians (-PI to PI)
 * @param rotation
 *            The rotation of the joystick, (-1.0 to 1.0)
 */
public void drive (double magnitude, double direction, double rotation)
{
    double altMagnitude, altDirection, altRotation;

    altMagnitude = super.scaleJoystickForDeadband(magnitude)
            * super.getCurrentGearRatio();
    altDirection = direction;
    altRotation = super.scaleJoystickForDeadband(rotation)
            * super.getCurrentGearRatio();

    // Check between the deadbands for the strafing cushion and and 90
    // degree "snap".
    if (direction > -90 - (directionalDeadband / 2.0)
            && direction < -90 + (directionalDeadband / 2.0))
        {
        altDirection = -90;
        altMagnitude += this.strafeCushion;
        }
    else if (direction > 90 - (directionalDeadband / 2.0)
            && direction < 90 + (directionalDeadband / 2.0))
        {
        altDirection = 90;
        altMagnitude += this.strafeCushion;
        }

    this.driveRaw(altMagnitude, altDirection, altRotation);

}

@Override
/**
 * Drives the robot based on raw inputs, for autonomous uses.
 * Also, can use a correction PID loop for rotation, if that is enabled.
 * 
 * @param magnitude
 *            How fast the robot will travel (0.0 to 1.0)
 * @param direction
 *            In which direction, laterally, will the robot travel (degrees,
 *            -180 to 180. 0 is forward.)
 * @param rotation
 *            How much the robot should be turning (left,(-1.0) to right,(1.0)
 */
public void driveRaw (double magnitude, double direction,
        double rotation)
{
    double altRotation = rotation;
    double altDirection = Math.toRadians(direction);

    // Only enable / disable / reset based on rotation IF we are using PID
    // loops.
    if (isUsingPID == true && altRotation != 0)
        {
        // While we are rotating, set the angle and reset/disable the PID.
        this.mecanumPID.disable();
        this.mecanumPID.getPIDController().reset();
        currentSetAngle = this.gyro.getAngle();
        }
    else if (isUsingPID == true)
        {
        // When we are NOT rotating (but still using PID loops),
        // enable PID loop and set the setpoint as the last angle since we
        // rotated.
        this.mecanumPID.enable();
        this.mecanumPID.setSetpoint(currentSetAngle);
        }
    else
        {
        // if we are not using the PID loop, then just disable everything.
        mecanumPID.disable();
        mecanumPID.getPIDController().reset();
        }

    if (isUsingPID == true)
        {
        altRotation = inRange(altRotation + adjustedAngle);
        }

    double leftFrontVal = Math.cos(altDirection - (Math.PI / 4.0));
    double rightFrontVal = Math.cos(altDirection + (Math.PI / 4.0));
    double rightRearVal = leftFrontVal;// The corner's equations are
    double leftRearVal = rightFrontVal;// equal to each other.

    super.getMotorController(MotorPosition.LEFT_FRONT)
            .set(inRange((magnitude * leftFrontVal) + altRotation));
    super.getMotorController(MotorPosition.LEFT_REAR)
            .set(inRange((magnitude * leftRearVal) + altRotation));
    super.getMotorController(MotorPosition.RIGHT_FRONT)
            .set(inRange((magnitude * rightFrontVal) - altRotation));
    super.getMotorController(MotorPosition.RIGHT_REAR)
            .set(inRange((magnitude * rightRearVal) - altRotation));
}

/**
 * Sets the angle deadband where the direction snaps to a 90 degree angle.
 * 
 * @param value
 *            deadband, in degrees
 */
public void setDirectionalDeadband (double value)
{
    this.directionalDeadband = Math.abs(value);
}

/**
 * Sets the percentage added to the magnitude while strafing.
 * This is only used inside the directional deadband, so if it is set to 0,
 * then it will not take affect.
 * 
 * @param value
 *            The value that strafeCushion will be set to.
 */
public void setStrafeCushion (double value)
{
    this.strafeCushion = value;
}

/**
 * Decides if we are using the PID loop to correct the rotation while we are
 * driving.
 * 
 * @param enabled
 *            true, if we are to enable the PID loop. False to not use it.
 */
public void setRotationCorrectionEnabled (boolean enabled)
{
    this.isUsingPID = enabled;
}

/**
 * Sets the PID values for the rotation correction loop.
 * 
 * @param p
 *            Proportional value
 * @param i
 *            Integral value
 * @param d
 *            Derivative value
 * @param tolerance
 *            voltage shutoff value (Degrees)
 */
public void setPIDValues (double p, double i, double d,
        double tolerance)
{
    this.mecanumPID.getPIDController().setPID(p, i, d);
    this.mecanumPID.getPIDController().setAbsoluteTolerance(tolerance);

    this.p = p;
    this.i = i;
    this.d = d;
    this.tolerance = tolerance;
}

/**
 * Sends all values to the SmartDashboard to be able to tune easily.
 * 
 * Setpoint - what is the target value? In this case, the setpoint is a number
 * of
 * degrees that the robot wants to stay at.
 * 
 * P - Proportional - creates the initial straight line of error to correction,
 * based on the sensor. High error means high correction, low error means low
 * correction.
 * I - Integral - Corrects based on how much time the error has had to build up.
 * The longer
 * the time, the higher the correction.
 * D - Derivative - Overall, this smoothes out the P-loop line, by adding
 * acceleration /
 * deceleration curves. Use this to get to the setpoint without overshooting it.
 * 
 * Tolerance - At how many degrees away do we want to stop adding a value to the
 * rotation?
 */
public void tunePID ()
{
    if (tunePIDInit == true)
        {
        SmartDashboard.putNumber("Mecanum P", p);
        SmartDashboard.putNumber("Mecanum I", i);
        SmartDashboard.putNumber("Mecanum D", d);
        SmartDashboard.putNumber("Mecanum Tolerance", tolerance);
        }

    p = SmartDashboard.getNumber("Mecanum P", p);
    i = SmartDashboard.getNumber("Mecanum I", i);
    d = SmartDashboard.getNumber("Mecanum D", d);
    tolerance = SmartDashboard.getNumber("Mecanum Tolerance", 0);
    this.setPIDValues(p, i, d, tolerance);
}

/**
 * Checks if the value input is in between -1 and 1 to keep it in range for
 * motor inputs.
 * 
 * @param val
 *            The input value
 * @return The correctly ranged value
 */
private double inRange (double val)
{
    if (val > 1)
        return 1;
    else if (val < -1)
        return -1;

    return val;
}

private PIDAnalogGyro gyro = null;

private boolean isUsingPID = false;

private boolean tunePIDInit = true;

private double p, i, d, tolerance;

private double currentSetAngle = 0; // For what is set as the setpoint for
                                    // PID loop.

private double adjustedAngle = 0;// For what is the output of the PID loop

private double strafeCushion = .2; // Disabled as long as directionDeadband
                                   // is 0

private double directionalDeadband = 0;// Disabled by default.

/**
 * The overall PID correction system.
 */
private PIDSubsystem mecanumPID = new PIDSubsystem(p, i, d)
{
@Override
protected double returnPIDInput ()
{
    return gyro.pidGet();
}

@Override
protected void usePIDOutput (double output)
{
    adjustedAngle = output;
}

@Override
protected void initDefaultCommand ()
{
}
};

}

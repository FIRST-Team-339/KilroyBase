package frc.HardwareInterfaces.Transmission;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * Like the traction drive system, except with four motors, usually all as
 * omni-wheels.
 * Or, tank drive with paired motors. Each joystick controls one side of the
 * robot.
 * 
 * @author Ryan McGee
 * @written 7/21/17
 */
public class TankTransmission extends TransmissionBase
{
/**
 * Creates the Transmission object.
 * 
 * @param leftSide
 *            the grouped motor controllers on the left side of the robot
 * 
 * @param rightSide
 *            the grouped motor controllers on the right side of the robot
 */
public TankTransmission (MotorControllerGroup leftSide,
        MotorControllerGroup rightSide)
{
    super(leftSide, rightSide);

    super.type = TransmissionType.TANK;
}

/**
 * Drives the transmission based on a Tank drive system,
 * where left controls the left wheels, and right controls the right wheels.
 * 
 * Uses joystick deadbands and gear ratios.
 * 
 * @param leftVal
 *            Percentage, (-1.0 to 1.0)
 * @param rightVal
 *            Percentage, (-1.0 to 1.0)
 */
public void drive (double leftVal, double rightVal)
{
    double leftOut = super.scaleJoystickForDeadband(leftVal)
            * super.getCurrentGearRatio();
    double rightOut = super.scaleJoystickForDeadband(rightVal)
            * super.getCurrentGearRatio();

    super.driveRaw(leftOut, rightOut);
}
}

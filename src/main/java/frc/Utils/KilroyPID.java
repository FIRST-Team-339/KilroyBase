package frc.Utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import frc.HardwareInterfaces.KilroyEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO -------------McGee fix--------------------------

/**
 * A unifying PID class that allows the programmer to choose whether to process
 * PID loops on-board or off-board. The pros and cons for each are listed in
 * their respective constructor.
 * 
 * @author Ryan McGee
 *
 */
public class KilroyPID
	{

	private final BaseMotorController offBoardController;

	private final PIDSubsystem onBoardController;

	private final PIDSource sensor;

	private final ControllerType type;

	/**
	 * Creates the KilroyPID object.
	 * 
	 * This constructor is for either PWM motors or CAN controllers using the
	 * "x_wpi" motor controller classes. It allows for a wider array of sensors, at
	 * the cost of PWM and sensor ports, and more strain on the RIO.
	 * 
	 * @param motorCont
	 *            The motor controller that will be manipulated by this class.
	 * @param sensor
	 *            The sensor that will pull data to compute the value sent to the
	 *            motor.
	 * @param pidType
	 *            Describes the function for the PID controller. If velocity, then
	 *            it is continuous rotation and the setpoint will be based on rate.
	 *            If position, then the setpoint will move to a certain position.
	 */
	public KilroyPID(MotorController motorCont, PIDSource sensor)
		{
			if (sensor instanceof KilroyEncoder && ((KilroyEncoder) sensor).getAttachedCANDevice() == motorCont)
				{
				initCAN();
				}
			else
				{
				initOnBoard();
				}

			this.sensor = sensor;

			if (pidType == PIDType.POSITION)
				sensor.setPIDSourceType(PIDSourceType.kDisplacement);
			else if (pidType == PIDType.VELOCITY)
				sensor.setPIDSourceType(PIDSourceType.kRate);

			onBoardController = new PIDSubsystem(0, 0, 0)
				{
				@Override
				protected double returnPIDInput()
				{
					if (isSensorReversed == true)
						return -sensor.pidGet();
					// else
					return sensor.pidGet();
				}

				@Override
				protected void usePIDOutput(double output)
				{
					motorCont.set(output);
				}

				@Override
				protected void initDefaultCommand()
				{

				}
				};
			offBoardController = null;
			type = ControllerType.ONBOARD;
		}

	private void initCAN()
	{

	}

	private void initOnBoard()
	{

	}

	/**
	 * Sets the PIDF values for the current PID controller.
	 * 
	 * Sent motor value = (p * error) + (i * integral of error) + (d * derivative of
	 * error) + (f * setpoint)
	 * 
	 * While calculating the PID values for tuning, remember that the motor values
	 * is between -1023 and +1023, NOT -1 and 1.
	 * 
	 * @param p
	 *            The Proportional value: the error multiplied by this scalar
	 *            directly. This is good for speeding up correction based on how far
	 *            away the current position is from the setpoint.
	 * @param i
	 *            The Integral value: the cumulative error multiplied by this
	 *            scalar. Over time, if the current position is consistently off,
	 *            then this is good for making sure it eventually makes it to the
	 *            setpoint.
	 * @param d
	 *            The Derivative value: how fast the current position is nearing the
	 *            setpoint, times this scalar. This is good for making sure the
	 *            motor does not overshoot the setpoint by slowing it down as it
	 *            reaches it.
	 * @param f
	 *            The Feed-Forward value: A base line for what should be sent to the
	 *            motor. This is multiplied by the setpoint, and is used mostly by
	 *            veolocity loops.
	 */
	public void setPIDF(double p, double i, double d, double f)
	{
		this.p = p;
		this.i = i;
		this.d = d;
		this.f = f;
		switch (type)
			{
			case CAN:
				this.offBoardController.config_kP(0, p, 0);
				this.offBoardController.config_kI(0, i, 0);
				this.offBoardController.config_kD(0, d, 0);
				this.offBoardController.config_kF(0, f, 0);
				break;
			case ONBOARD:
				this.onBoardController.getPIDController().setPID(p, i, d, f);
				break;
			default:
				break;
			}
	}

	/**
	 * Sets at what point the PID loop is considered "on target"
	 * 
	 * @param tolerance
	 *            "on target" range (plus or minus), in correct units.
	 */
	public void setTolerance(double tolerance)
	{
		this.tolerance = tolerance;
		switch (type)
			{
			case ONBOARD:
				this.onBoardController.getPIDController().setAbsoluteTolerance(tolerance);
				return;
			default:
				return;
			}
	}

	/**
	 * Sets the setpoint of the PID loop. Whatever value and type of value is input,
	 * the motor will use the PID loop and attempt to reach that value.
	 * 
	 * @param value
	 *            The value (in correct units based on the type) the motor will move
	 *            to.
	 * @param runSensorReversed
	 *            TODO
	 * @param pidType
	 *            What the PID loop will see the value as. If it's position, then it
	 *            is a move and stop. If it's velocity, it will try to go the speed
	 *            input in value.
	 */
	public void setSetpoint(double value, boolean runSensorReversed)
	{
		this.setpoint = value;
		this.setSensorInverted(runSensorReversed);
		switch (type)
			{
			case CAN:
				// Set the type of movement and the setpoint.
				if (pidType == PIDType.POSITION)
					offBoardController.set(ControlMode.Position, value);
				else if (pidType == PIDType.VELOCITY)
					offBoardController.set(ControlMode.Velocity, value);
				break;
			case ONBOARD:
				// Set the setpoint
				this.onBoardController.setSetpoint(value);
				break;
			default:
				break;
			}
	}

	/**
	 * Sets what kind of PID loop will be used.
	 * 
	 * @param pidType
	 *            If position, then the motor will go to the setpoint and stop. If
	 *            velocity, then the motor's speed will match the setpoint.
	 */
	public void setSensorType(PIDType pidType)
	{
		this.pidType = pidType;

		if (pidType == PIDType.POSITION)
			this.sensor.setPIDSourceType(PIDSourceType.kDisplacement);
		else if (pidType == PIDType.VELOCITY)
			this.sensor.setPIDSourceType(PIDSourceType.kRate);
	}

	/**
	 * Either enables or disables the PID loop. If disabled, the motor will stop.
	 * 
	 * @param enabled
	 *            Whether it is enabled(true) or disabled(false)
	 */
	public void setEnabled(boolean enabled)
	{
		switch (type)
			{
			case CAN:
				if (enabled == false)
					{
					this.offBoardController.set(ControlMode.PercentOutput, 0);
					}
				else
					{
					this.setSetpoint(setpoint, false);
					}
				break;
			case ONBOARD:
				this.onBoardController.getPIDController().setEnabled(enabled);
				break;
			default:
				break;
			}
	}

	/**
	 * If this PID controller is using a CAN sensor connected to the CAN motor
	 * controller, then this will invert it if it is "out of phase" (motor
	 * controller and sensor are not in the same direction)
	 * 
	 * @param inverted
	 *            Whether or not the sensor is going backwards
	 */
	public void setSensorInverted(boolean inverted)
	{
		switch (type)
			{
			case CAN:
				this.offBoardController.setSensorPhase(inverted);
				break;
			case ONBOARD:
				this.isSensorReversed = inverted;
				break;
			default:
				break;

			}
	}

	/**
	 * Sets the maximum velocity the robot is allowed to move while using the PID
	 * loop.
	 * 
	 * @param velocity
	 *            Maximum speed (forwards or backwards), in feet per second.
	 */
	public void setSpeed(double velocity)
	{
		switch (type)
			{
			case CAN:
				this.offBoardController.configPeakOutputForward(Math.abs(velocity), 0);
				this.offBoardController.configPeakOutputReverse(Math.abs(velocity), 0);
				return;
			case ONBOARD:
				this.onBoardController.getPIDController().setOutputRange(-Math.abs(velocity), Math.abs(velocity));
			default:
				break;
			}
	}

	/**
	 * @return Gets the last setpoint, or the one used in tuning.
	 */
	public double getSetpoint()
	{
		return setpoint;
	}

	/**
	 * @return The lower level PID controller for on-board processing
	 */
	public edu.wpi.first.wpilibj.PIDController getPIDController()
	{
		// TODO ----------McGee fix--------------
		return this.onBoardController.getPIDController();
	}

	/**
	 * @return The CAN motor controller that deals with the PID control
	 */
	public BaseMotorController getCANController()
	{
		return this.offBoardController;
	}

	/**
	 * If a PID is in use on a differential drive, for instance, then the front and
	 * rear motors can be tethered to one sensor / PID loop.
	 * 
	 * @param controller
	 *            Any CAN motor controller
	 */
	public void tetherCANMotorControllers(BaseMotorController... controller)
	{
		switch (type)
			{
			case CAN:
				for (BaseMotorController bmc : controller)
					bmc.follow(this.offBoardController);
				break;
			default:
				break;
			}
	}

	/**
	 * Clears the PID integral stored values, effectively "reseting" the PID loop.
	 */
	public void resetPID()
	{
		switch (type)
			{
			case CAN:
				this.offBoardController.setIntegralAccumulator(0, 0, 0);
				break;
			case ONBOARD:
				this.onBoardController.getPIDController().reset();
				break;
			default:
				break;
			}
	}

	/**
	 * @return Whether or not the PID loop says it is "on target", defined by the
	 *         tolerance set previously.
	 */
	public boolean isOnTarget()
	{
		switch (type)
			{
			case CAN:
				return (Math.abs(this.offBoardController.getClosedLoopError(0)) < this.tolerance);
			case ONBOARD:
				return this.onBoardController.onTarget();
			default:
				return true;
			}
	}

	/**
	 * Allows the easy tuning of a PID loop via shuffleboard / smartdashboard.
	 * 
	 * @param pidName
	 *            What to call the PID numbers. this will show up as [name]_p,
	 *            [name]_i, etc.
	 */
	public void tunePIDSmartDashboard(String pidName)
	{
		if (initTune == false)
			{
			SmartDashboard.putNumber(pidName + "_p", 0);
			SmartDashboard.putNumber(pidName + "_i", 0);
			SmartDashboard.putNumber(pidName + "_d", 0);
			SmartDashboard.putNumber(pidName + "_f", 0);
			SmartDashboard.putNumber(pidName + "_Tolerance", 0);
			SmartDashboard.putNumber(pidName + "_setPoint", 0);
			initTune = true;
			}
		p = SmartDashboard.getNumber(pidName + "_p", 0);
		i = SmartDashboard.getNumber(pidName + "_i", 0);
		d = SmartDashboard.getNumber(pidName + "_d", 0);
		f = SmartDashboard.getNumber(pidName + "_f", 0);
		tolerance = SmartDashboard.getNumber(pidName + "_Tolerance", 0);
		setpoint = SmartDashboard.getNumber(pidName + "_setPoint", 0);

		this.setPIDF(p, i, d, f);
		this.setTolerance(tolerance);
	}

	/**
	 * Describes the kind of PID motor controller used
	 * 
	 * @author Kilroy
	 *
	 */
	public enum ControllerType
		{
		/**
		 * We are using the PID built into the CAN motor controller
		 */
		CAN,
		/**
		 * We are calculating the PID values on the roboRIO
		 */
		ONBOARD
		}

	/**
	 * What kind of PID is being used
	 * 
	 * @author Kilroy
	 *
	 */
	public enum PIDType
		{
		/**
		 * Going to a set position / displacement
		 */
		POSITION,
		/**
		 * Going at a fixed rate, continuous
		 */
		VELOCITY
		}

	private double p, i, d, f, setpoint, tolerance;

	private boolean initTune = false;

	private boolean isSensorReversed = false;

	private PIDType pidType = PIDType.POSITION;
	}

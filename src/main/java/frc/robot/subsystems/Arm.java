// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;
import frc.robot.Constants.PneumaticsConstants;

public class Arm extends Rumbler
{
    // private final TalonSRX leftPivotMotor = new TalonSRX(PivotConstants.leftMotorChannel);
    // private final TalonSRX rightPivotMotor = new TalonSRX(PivotConstants.rightMotorChannel);
    private final CANSparkMax leftPivotMotor = new CANSparkMax(PivotConstants.leftMotorChannel, MotorType.kBrushless);
    private final CANSparkMax rightPivotMotor = new CANSparkMax(PivotConstants.rightMotorChannel, MotorType.kBrushless);
    private final CANSparkMax extenderMotor = new CANSparkMax(ExtenderConstants.motorChannel, MotorType.kBrushless);

    private final DoubleSolenoid pivotBrakeSolenoid = new DoubleSolenoid(PneumaticsConstants.moduleType, PivotConstants.brakeSolenoidForwardChannel, PivotConstants.brakeSolenoidReverseChannel);

    private final DigitalInput extensionMinPositionSwitch = new DigitalInput(ArmConstants.extensionMinPositionChannel);
    private final DigitalInput extensionMidPositionSwitch = new DigitalInput(ArmConstants.extensionMidPositionChannel);
    private final DigitalInput extensionMaxPositionSwitch = new DigitalInput(ArmConstants.extensionMaxPositionChannel);
    private final DigitalInput extensionMaxPositionInternalSwitch = new DigitalInput(ArmConstants.extensionMaxPositionInternalChannel);

    // THe following used for keeping track of extension position:
    private boolean isExtensionBeyondMidPosition;
    
    // The following used for changing extender position:
    private double extenderTargetPosition;
    private double extenderTargetTolerance;
    private double lastExtenderPosition;
    private boolean isExtenderAtTargetPosition;
    private boolean isExtenderPositioningStarted;
    
    // The following used for changing pivot position:
    private double pivotTargetPosition;
    private double pivotTargetTolerance;
    private double lastPivotPosition;
    private boolean isPivotAtTargetPosition;
    private boolean isPivotPositioningStarted;


    /** Creates a new Arm. */
    public Arm(XboxController gamepad)
    {
        super(gamepad);

        // Set configurtation for extender motor:
        extenderMotor.restoreFactoryDefaults();
        extenderMotor.setInverted(ExtenderConstants.isMotorInverted);
        extenderMotor.clearFaults();
        extenderMotor.setIdleMode(ExtenderConstants.idleMode);
        extenderMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ExtenderConstants.maxPosition);
        extenderMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ExtenderConstants.minPosition);
        extenderMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        extenderMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        extenderMotor.setSmartCurrentLimit(ExtenderConstants.maxMotorCurrent);

        // Set PID controller parameters on the extender motor:
        var pidController = extenderMotor.getPIDController();
        pidController = extenderMotor.getPIDController();
        pidController.setP(ExtenderConstants.PIDConstants.kP);
        pidController.setI(ExtenderConstants.PIDConstants.kI);
        pidController.setD(ExtenderConstants.PIDConstants.kD);
        pidController.setIZone(ExtenderConstants.PIDConstants.kIZ);
        pidController.setFF(ExtenderConstants.PIDConstants.kFF);

        resetExtenderPosition();

        // Set configuration common to both pivot motors:
		for (CANSparkMax motor : new CANSparkMax[] { leftPivotMotor, rightPivotMotor})
		{
            motor.restoreFactoryDefaults();
            motor.clearFaults();
            motor.setIdleMode(PivotConstants.idleMode);
            motor.setSoftLimit(SoftLimitDirection.kForward, (float) PivotConstants.maxPosition);
            motor.setSoftLimit(SoftLimitDirection.kReverse, (float) PivotConstants.minPosition);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
            motor.setSmartCurrentLimit(PivotConstants.maxMotorCurrent);
		}

        // Configure invertedness of the pivot motors:
        leftPivotMotor.setInverted(PivotConstants.isLeftMotorInverted);
        rightPivotMotor.follow(leftPivotMotor, PivotConstants.isLeftMotorInverted != PivotConstants.isRightMotorInverted);

        // Set PID controller parameters on the pivot leader:
        pidController = leftPivotMotor.getPIDController();
        pidController.setP(PivotConstants.PIDConstants.kP);
        pidController.setI(PivotConstants.PIDConstants.kI);
        pidController.setD(PivotConstants.PIDConstants.kD);
        pidController.setIZone(PivotConstants.PIDConstants.kIZ);
        pidController.setFF(PivotConstants.PIDConstants.kFF);

        resetPivotPosition();
    }

	/**
	 * Sets the extender to a particular position.
	 */
	public void setExtenderPosition(double position, double tolerance)
	{
        if (position > ExtenderConstants.maxPosition)
        {
            position = ExtenderConstants.maxPosition;
        }
        else if (position < ExtenderConstants.minPosition)
        {
            position = ExtenderConstants.minPosition;
        }
        // else if (position > PivotConstants.maxFrontPositionWhenBeyondMidExtension &&
        //          isExtensionAtOrBeyondMidPosition())
        // {
        //     position = PivotConstants.maxFrontPositionWhenBeyondMidExtension;
        // }

        extenderTargetPosition = position;
        extenderTargetTolerance = tolerance;
        lastExtenderPosition = 0;
        var minOutput = SmartDashboard.getNumber(ArmKeys.extenderPidMinOutput, ExtenderConstants.PIDConstants.defaultMinOutput);
        var maxOutput = SmartDashboard.getNumber(ArmKeys.extenderPidMaxOutput, ExtenderConstants.PIDConstants.defaultMaxOutput);
        extenderMotor.getPIDController().setOutputRange(minOutput, maxOutput);
        extenderMotor.getPIDController().setReference(position, ControlType.kPosition);
        isExtenderAtTargetPosition = false;
        isExtenderPositioningStarted = true;
	}

    public void setExtenderSpeed(double speed)
    {
        if ((speed < 0 && isExtensionAtHardMinPosition()) ||
            (speed > 0 && (isExtensionAtHardMaxPosition() || isPivotMaxedForExtension())))
    
        {
            speed = 0;
            rightRumbleOn();
        }
        else
        {
            rightRumbleOff();
        }

        extenderMotor.set(speed);
    }

    /**
     * Redsets the extender position counter to 0.
     */
    public void resetExtenderPosition()
    {
        extenderMotor.getEncoder().setPosition(0);
    }

    public double getExtenderPosition()
    {
        return extenderMotor.getEncoder().getPosition();
    }

    public boolean isExtenderAtTargetPosition()
    {
        return isExtenderAtTargetPosition;
    }

    public void stopExtender()
    {
        setExtenderSpeed(0);
        rightRumbleOff();
    }

	/**
	 * Sets the pivot to a particular position.
	 */
	public void setPivotPosition(double position, double tolerance)
	{
        if (position > PivotConstants.maxPosition)
        {
            position = PivotConstants.maxPosition;
        }
        else if (position < PivotConstants.minPosition)
        {
            position = PivotConstants.minPosition;
        }
        else if (position > PivotConstants.maxFrontPositionWhenBeyondMidExtension &&
                 isExtensionAtOrBeyondMidPosition())
        {
            position = PivotConstants.maxFrontPositionWhenBeyondMidExtension;
        }

        pivotTargetPosition = position;
        pivotTargetTolerance = tolerance;
        lastPivotPosition = 0;
        var minOutput = SmartDashboard.getNumber(ArmKeys.pivotPidMinOutput, PivotConstants.PIDConstants.defaultMinOutput);
        var maxOutput = SmartDashboard.getNumber(ArmKeys.pivotPidMaxOutput, PivotConstants.PIDConstants.defaultMaxOutput);
        leftPivotMotor.getPIDController().setOutputRange(minOutput, maxOutput);
        leftPivotMotor.getPIDController().setReference(position, ControlType.kPosition);
        releasePivotBrake();
        isPivotAtTargetPosition = false;
        isPivotPositioningStarted = true;
	}

    public void setPivotSpeed(double speed)
    {
        if ((speed < 0 && isPivotAtRevLimit()) ||
            (speed > 0 && (isPivotAtFwdLimit() || isPivotMaxedForExtension())))
        {
            speed = 0;
            leftRumbleOn();
        }
        else
        {
            leftRumbleOff();
        }

        if (speed == 0)
        {
            setPivotBrake();
        }

        isPivotAtTargetPosition = false;
        isPivotPositioningStarted = false;
        leftPivotMotor.set(speed);
        
        if (speed != 0)
        {
            releasePivotBrake();
        }
    }

    public void stopPivot()
    {
        setPivotBrake();
        leftPivotMotor.set(0);
        leftRumbleOff();
    }

    public void setPivotBrake()
    {
        pivotBrakeSolenoid.set(Value.kForward);
    }

    public void releasePivotBrake()
    {
        pivotBrakeSolenoid.set(Value.kReverse);
    }

    /**
     * Redsets the pivot position counter to 0.
     */
    public void resetPivotPosition()
    {
        leftPivotMotor.getEncoder().setPosition(0);
        rightPivotMotor.getEncoder().setPosition(0);
    }

    public double getPivotPosition()
    {
        return leftPivotMotor.getEncoder().getPosition();
    }

    /**
     * Returns the angle (in radians) of the arm relative to the front of the robot
     */
    public double getPivotAngle()
    {
        return Math.min(PivotConstants.frontHorizontalPosition, getPivotPosition()) * PivotConstants.radiansPerMotorShaftRotation;
    }

    public boolean isPivotAtTargetPosition()
    {
        return isPivotAtTargetPosition;
    }

    public boolean isPivotAtFwdLimit()
    {
        return leftPivotMotor.getFault(FaultID.kSoftLimitFwd) || rightPivotMotor.getFault(FaultID.kSoftLimitFwd);
    }

    public boolean isPivotAtRevLimit()
    {
        return leftPivotMotor.getFault(FaultID.kSoftLimitRev) || rightPivotMotor.getFault(FaultID.kSoftLimitRev);
    }

    public boolean isPivotAtOrBeyondMidExtensionLimit()
    {
        return leftPivotMotor.getEncoder().getPosition() >= PivotConstants.maxFrontPositionWhenBeyondMidExtension;
    }

    public boolean isPivotMaxedForExtension()
    {
        return Constants.DEBUG
            ? false
            : ExtenderConstants.maxPositionAtFrontHorizontalPivot / getExtenderPosition() <= Math.cos(getPivotAngle());
    }

    @Override
    public void periodic()
    {
        // Get the current extender position:
        var extenderPosition = extenderMotor.getEncoder().getPosition();

        // Get the current pivot position:
        var leftPivotPosition = leftPivotMotor.getEncoder().getPosition();

        // Query hardware limit switches just once:
        var isExtensionAtHardMinPosition = isExtensionAtHardMinPosition();
        var isExtensionAtHardMidPosition = isExtensionAtHardMidPosition();
        var isExtensionAtHardMaxPosition = isExtensionAtHardMaxPosition();


        SmartDashboard.putNumber(ArmKeys.extenderPosition, extenderPosition);
        SmartDashboard.putBoolean(ArmKeys.extensionMinPosition, !isExtensionAtHardMinPosition);
        SmartDashboard.putBoolean(ArmKeys.extensionMidPosition, isExtensionAtHardMidPosition);
        SmartDashboard.putBoolean(ArmKeys.extensionBeyondMidPosition, isExtensionBeyondMidPosition);
        SmartDashboard.putBoolean(ArmKeys.extensionMaxPosition, !isExtensionAtHardMaxPosition);
        SmartDashboard.putBoolean(ArmKeys.pivotMaxedForExtension, !isPivotMaxedForExtension());
        SmartDashboard.putNumber(ArmKeys.pivotCurLeftPosition, leftPivotPosition);

        // If we are currently extending to a specific position, see if we are
        // at the desired position. If so, indicate that we have reached the
        // desired position and are no longer extending to said position.
        // Otherwise, remember the current extender position to use for the next check.
        if (isExtenderPositioningStarted)
        {
            if (Math.abs(extenderPosition - extenderTargetPosition) < extenderTargetTolerance && Math.abs(extenderPosition - lastExtenderPosition) < extenderTargetTolerance)
            {
                isExtenderPositioningStarted = false;
                isExtenderAtTargetPosition = true;
            }
            else
            {
                lastExtenderPosition = extenderPosition;
            }
        }

        // If we are currently pivoting to a specific position, see if we are
        // at the desired position. If so, indicate that we have reached the
        // desired position and are no longer pivoting to said position.
        // Otherwise, remember the current pivot position to use for the next check.
        if (isPivotPositioningStarted)
        {
            if (Math.abs(leftPivotPosition - pivotTargetPosition) < pivotTargetTolerance && Math.abs(leftPivotPosition - lastPivotPosition) < pivotTargetTolerance)
            {
                isPivotPositioningStarted = false;
                isPivotAtTargetPosition = true;
            }
            else
            {
                lastPivotPosition = leftPivotPosition;
            }
        }
    }

    public boolean isExtensionAtHardMinPosition()
    {
        // The mag limit switch returns false when the switch is engaged.
        return !extensionMinPositionSwitch.get();
    }

    public boolean isExtensionAtHardMidPosition()
    {
        // The mag limit switch returns false when the switch is engaged.
        return !extensionMidPositionSwitch.get();
    }

    public boolean isExtensionBeyondMidPosition()
    {
        return getExtenderPosition() > ExtenderConstants.midPosition;
    }

    public boolean isExtensionAtOrBeyondMidPosition()
    {
        return getExtenderPosition() >= ExtenderConstants.midPosition;
    }

    public boolean isExtensionAtHardMaxPosition()
    {
        return !extensionMaxPositionSwitch.get() || extensionMaxPositionInternalSwitch.get();
    }
}

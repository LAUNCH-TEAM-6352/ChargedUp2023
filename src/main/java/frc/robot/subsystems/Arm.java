// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.ArmConstants.PivotConstants.PIDConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;
import frc.robot.Constants.PneumaticsConstants;

public class Arm extends Rumbler
{
    // private final TalonSRX leftPivotMotor = new TalonSRX(PivotConstants.leftMotorChannel);
    // private final TalonSRX rightPivotMotor = new TalonSRX(PivotConstants.rightMotorChannel);
    private final CANSparkMax leftPivotMotor = new CANSparkMax(PivotConstants.leftMotorChannel, MotorType.kBrushless);
    private final CANSparkMax rightPivotMotor = new CANSparkMax(PivotConstants.rightMotorChannel, MotorType.kBrushless);
    private final VictorSPX extenderMotor = new VictorSPX(ExtenderConstants.motorChannel);

    private final DoubleSolenoid pivotBrakeSolenoid = new DoubleSolenoid(PneumaticsConstants.moduleType, PivotConstants.brakeSolenoidForwardChannel, PivotConstants.brakeSolenoidReverseChannel);

    private final DigitalInput extensionMinPositionSwitch = new DigitalInput(ArmConstants.extensionMinPositionChannel);
    private final DigitalInput extensionMidPositionSwitch = new DigitalInput(ArmConstants.extensionMidPositionChannel);
    private final DigitalInput extensionMaxPositionSwitch = new DigitalInput(ArmConstants.extensionMaxPositionChannel);
    private final DigitalInput extensionMaxPositionInternalSwitch = new DigitalInput(ArmConstants.extensionMaxPositionInternalChannel);

    // THe following used for keeping track of extension position:
    private boolean wasExtensionAtMidPosition;
    private boolean isExtensionBeyondMidPosition;
    private double lastExtenderRunningSpeed = 0;
    
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

        extenderMotor.setInverted(ExtenderConstants.isMotorInverted);
        
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

        // COnfigure invertedness of the pivoit motors:
        leftPivotMotor.setInverted(PivotConstants.isLeftMotorInverted);
        rightPivotMotor.follow(leftPivotMotor, PivotConstants.isLeftMotorInverted != PivotConstants.isRightMotorInverted);

        // Set PID controller parameters on the pivot leader:
        var pidController = leftPivotMotor.getPIDController();
        pidController.setP(PIDConstants.kP);
        pidController.setI(PIDConstants.kI);
        pidController.setD(PIDConstants.kD);
        pidController.setIZone(PIDConstants.kIZ);
        pidController.setFF(PIDConstants.kFF);

        resetPivotPosition();
    }

	/**
	 * Sets the shooter motor speeds in velocity (RPM).
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
        pivotTargetPosition = position;
        pivotTargetTolerance = tolerance;
        lastPivotPosition = 0;
        var minOutput = SmartDashboard.getNumber(ArmKeys.pivotPidMinOutput, PIDConstants.defaultMinOutput);
        var maxOutput = SmartDashboard.getNumber(ArmKeys.pivotPidMaxOutput, PIDConstants.defaultMaxOutput);
        leftPivotMotor.getPIDController().setOutputRange(minOutput, maxOutput);
        leftPivotMotor.getPIDController().setReference(position, ControlType.kPosition);
        releasePivotBrake();
        isPivotAtTargetPosition = false;
        isPivotPositioningStarted = true;
	}

    public void setPivotSpeed(double speed)
    {
        if ((speed < 0 && isPivotAtRevLimit()) || (speed > 0 && isPivotAtFwdLimit()))
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

    public void setExtenderSpeed(double speed)
    {
        if ((isExtensionAtMaxPosition() && speed > 0) || (isExtensionAtMinPosition() && speed < 0))
        {
            speed = 0;
            rightRumbleOn();
        }
        else
        {
            rightRumbleOff();
        }

        // Remember last non-zero speed for determining when go beyond mid position:
        if (speed !=0 )
        {
            lastExtenderRunningSpeed = speed;
        }

        extenderMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopExtender()
    {
        setExtenderSpeed(0);
        rightRumbleOff();
    }

    @Override
    public void periodic()
    {
        var leftPivotPosition = leftPivotMotor.getEncoder().getPosition();
        var rightPivotPosition = rightPivotMotor.getEncoder().getPosition();

        // Determine if the arm is extended beyond the mid position.
        // If we were at the mid position but are not anymore use the
        // most recent non-zero extender speed to determine in what
        // direction the extender has moved.
        if (wasExtensionAtMidPosition && !isExtensionAtMidPosition())
        {
            isExtensionBeyondMidPosition = lastExtenderRunningSpeed > 0.0;
        }

        // Remember if we were at the mid position:
        wasExtensionAtMidPosition = isExtensionAtMidPosition();

        SmartDashboard.putBoolean(ArmKeys.extensionMinPosition, !isExtensionAtMinPosition());
        SmartDashboard.putBoolean(ArmKeys.extensionMidPosition, isExtensionAtMidPosition());
        SmartDashboard.putBoolean(ArmKeys.extensionBeyondMidPosition, isExtensionBeyondMidPosition);
        SmartDashboard.putBoolean(ArmKeys.extensionMaxPosition, !isExtensionAtMaxPosition());
        SmartDashboard.putNumber(ArmKeys.extenderLastSpeed, lastExtenderRunningSpeed);
        SmartDashboard.putNumber(ArmKeys.pivotCurLeftPosition, leftPivotPosition);
        SmartDashboard.putNumber(ArmKeys.pivotCurRightPosition, rightPivotPosition);

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

    public boolean isExtensionAtMinPosition()
    {
        // The mag limit switch returns false when the switch is engaged.
        return !extensionMinPositionSwitch.get();
    }

    public boolean isExtensionAtMidPosition()
    {
        // The mag limit switch returns false when the switch is engaged.
        return !extensionMidPositionSwitch.get();
    }

    public boolean isExtensionBeyondMidPosition()
    {
        return isExtensionBeyondMidPosition;
    }

    public boolean isExtensionAtMaxPosition()
    {
        return !extensionMaxPositionSwitch.get() || extensionMaxPositionInternalSwitch.get();
    }
}

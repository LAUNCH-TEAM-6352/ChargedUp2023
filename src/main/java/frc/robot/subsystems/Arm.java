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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ExtenderConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.ArmConstants.PivotConstants.PIDConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;
import frc.robot.Constants.PneumaticsConstants;

public class Arm extends SubsystemBase
{
    // private final TalonSRX leftPivotMotor = new TalonSRX(PivotConstants.leftMotorChannel);
    // private final TalonSRX rightPivotMotor = new TalonSRX(PivotConstants.rightMotorChannel);
    private final CANSparkMax leftPivotMotor = new CANSparkMax(PivotConstants.leftMotorChannel, MotorType.kBrushless);
    private final CANSparkMax rightPivotMotor = new CANSparkMax(PivotConstants.rightMotorChannel, MotorType.kBrushless);
    private final VictorSPX extenderMotor = new VictorSPX(ExtenderConstants.motorChannel);

    private final DoubleSolenoid pivotBrakeSolenoid = new DoubleSolenoid(PneumaticsConstants.moduleType, PivotConstants.brakeSolenoidForwardChannel, PivotConstants.brakeSolenoidReverseChannel);

    private final DigitalInput maxPivotFrontPosition = new DigitalInput(ArmConstants.maxPivotFrontPositionChannel);
    private final DigitalInput maxPivotBackPosition = new DigitalInput(ArmConstants.maxPivotBackPositionChannel);
    private final DigitalInput homePivotPosition = new DigitalInput(ArmConstants.homePivotPositionChannel);
    private final DigitalInput deliveryPivotPosition = new DigitalInput(ArmConstants.deliveryPivotPositionChannel);

    private final DigitalInput minExtensionPosition = new DigitalInput(ArmConstants.minExtensionPositionChannel);
    private final DigitalInput maxExtensionPosition = new DigitalInput(ArmConstants.maxExtensionPositionChannel);
    private final DigitalInput deliveryExtensionPosition = new DigitalInput(ArmConstants.deliveryExtensionPositionChannel);

    private double targetPivotPosition;
    private double targetPivotTolerance;
    private double lastPivotPosition;
    private boolean isAtTargetPivotPosition;
    private boolean isPivotPositioningStarted;


    /** Creates a new Arm. */
    public Arm()
    {
        extenderMotor.setInverted(ExtenderConstants.isMotorInverted);
        
        // Set configuration common to both pivot motors:
		for (CANSparkMax motor : new CANSparkMax[] { leftPivotMotor, rightPivotMotor})
		{
            motor.restoreFactoryDefaults();
            motor.clearFaults();
            motor.setIdleMode(PivotConstants.idleMode);
            motor.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.maxPosition);
            motor.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.minPosition);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
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
        targetPivotPosition = position;
        targetPivotTolerance = tolerance;
        lastPivotPosition = 0;
        var minOutput = SmartDashboard.getNumber(ArmKeys.pivotPidMinOutput, PIDConstants.defaultMinOutput);
        var maxOutput = SmartDashboard.getNumber(ArmKeys.pivotPidMaxOutput, PIDConstants.defaultMaxOutput);
        leftPivotMotor.getPIDController().setOutputRange(minOutput, maxOutput);
        leftPivotMotor.getPIDController().setReference(position, ControlType.kPosition);
        releasePivotBrake();
        isAtTargetPivotPosition = false;
        isPivotPositioningStarted = true;
	}

    public void setPivotSpeed(double speed)
    {
        if ((speed < 0 && isPivotAtRevLimit()) || (speed > 0 && isPivotAtFwdLimit()))
        {
            speed = 0;
            setPivotBrake();
        }
        isAtTargetPivotPosition = false;
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
    }

    public void setPivotBrake()
    {
        pivotBrakeSolenoid.set(Value.kReverse);
    }

    public void releasePivotBrake()
    {
        pivotBrakeSolenoid.set(Value.kForward);
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

    public boolean isAtTargetPivotPosition()
    {
        return isAtTargetPivotPosition;
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
        if ((isAtMaxExtensionLimit() && speed > 0) || (isAtMinExtensionLimit() && speed < 0))
        {
            speed = 0;
        }
        extenderMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopExtender()
    {
        setExtenderSpeed(0);
    }

    @Override
    public void periodic()
    {
        var leftPivotPosition = leftPivotMotor.getEncoder().getPosition();
        var rightPivotPosition = rightPivotMotor.getEncoder().getPosition();

        // This method will be called once per scheduler run
        SmartDashboard.putBoolean(ArmKeys.extensionLimit, !(isAtMaxExtensionLimit() || isAtMinExtensionLimit()));
        SmartDashboard.putBoolean(ArmKeys.extensionDeliveryPosition, isAtDeliveryExtensionPosition());
        SmartDashboard.putNumber(ArmKeys.pivotCurLeftPosition, leftPivotPosition);
        SmartDashboard.putNumber(ArmKeys.pivotCurRightPosition, rightPivotPosition);

        if (isPivotPositioningStarted)
        {
            if (Math.abs(leftPivotPosition - targetPivotPosition) < targetPivotTolerance && Math.abs(leftPivotPosition - lastPivotPosition) < targetPivotTolerance)
            {
                isPivotPositioningStarted = false;
                isAtTargetPivotPosition = true;
            }
            else
            {
                lastPivotPosition = leftPivotPosition;
            }
        }
    }


    public boolean isAtMaxPivotFrontPosition()
    {
        return maxPivotFrontPosition.get();
    }

    public boolean isAtMaxPivotBackPosition()
    {
        return maxPivotBackPosition.get();
    }

    public boolean isAtHomePivotPosition()
    {
        return homePivotPosition.get();
    }

    public boolean isAtDeliveryPivotPosition()
    {
        return deliveryPivotPosition.get();
    }

    public boolean isAtMinExtensionLimit()
    {
        return minExtensionPosition.get();
    }

    public boolean isAtMaxExtensionLimit()
    {
        return maxExtensionPosition.get();
    }

    public boolean isAtDeliveryExtensionPosition()
    {
        return deliveryExtensionPosition.get();
    }

}

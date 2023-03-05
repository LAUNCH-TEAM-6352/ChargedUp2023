// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
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
        leftPivotMotor.setInverted(PivotConstants.isLeftMotorInverted);
        rightPivotMotor.follow(leftPivotMotor, PivotConstants.isLeftMotorInverted != PivotConstants.isRightMotorInverted);

        extenderMotor.setInverted(ExtenderConstants.isMotorInverted);

		// for (TalonSRX motor : new TalonSRX[] { leftPivotMotor, rightPivotMotor})
		// {
		// 	motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
		// 	motor.configAllowableClosedloopError(PivotConstants.pidProfileSlot, PivotConstants.pidAllowableError,
        //     PivotConstants.pidTimeoutMs);
		// 	motor.configClosedLoopPeakOutput(PivotConstants.pidProfileSlot, PivotConstants.pidPeakOutput,
        //     PivotConstants.pidTimeoutMs);
		// 	motor.configClosedLoopPeriod(PivotConstants.pidProfileSlot, PivotConstants.pidLoopPeriodMs,
        //         PivotConstants.pidTimeoutMs);
		// 	motor.config_kP(PivotConstants.pidProfileSlot, PivotConstants.pidP,
        //         PivotConstants.pidTimeoutMs);
		// 	motor.config_kI(PivotConstants.pidProfileSlot, PivotConstants.pidI,
        //         PivotConstants.pidTimeoutMs);
		// 	motor.config_kD(PivotConstants.pidProfileSlot, PivotConstants.pidD,
        //         PivotConstants.pidTimeoutMs);
		// 	motor.config_kF(PivotConstants.pidProfileSlot, PivotConstants.pidFF,
        //         PivotConstants.pidTimeoutMs);
		// 	motor.config_IntegralZone(PivotConstants.pidProfileSlot, PivotConstants.pidIZ,
        //         PivotConstants.pidTimeoutMs);
		// 	motor.selectProfileSlot(PivotConstants.pidProfileSlot, PivotConstants.primaryClosedLoop);
		// 	motor.setSensorPhase(PivotConstants.isSensorPhaseInverted);
        //     motor.setNeutralMode(PivotConstants.neutralMode);
		// }

        // Set PID controller parameters on the pivot leader:
        var pidController = leftPivotMotor.getPIDController();
        pidController.setP(PivotConstants.pidP);
        pidController.setI(PivotConstants.pidI);
        pidController.setD(PivotConstants.pidD);
        pidController.setIZone(PivotConstants.pidIZ);
        pidController.setFF(PivotConstants.pidFF);
        pidController.setOutputRange(PivotConstants.pidMinOutput, PivotConstants.pidMaxOutput);
        
        // Set configuration common to both pivot motors:
		for (CANSparkMax motor : new CANSparkMax[] { leftPivotMotor, rightPivotMotor})
		{
            motor.setIdleMode(PivotConstants.idleMode);
            motor.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.forwardPositionLimit);
            motor.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.reversePositionLimit);
            motor.enableSoftLimit(SoftLimitDirection.kForward, true);
            motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
		}

        resetPivotPosition();
    }

	/**
	 * Sets the shooter motor speeds in velocity (RPM).
	 */
	public void setPivotPosition(double position, double tolerance)
	{        
        targetPivotPosition = position;
        targetPivotTolerance = tolerance;
        lastPivotPosition = 0;
        leftPivotMotor.getPIDController().setReference(position, ControlType.kPosition);
        releasePivotBrake();
        isAtTargetPivotPosition = false;
        isPivotPositioningStarted = true;
	}

    public void setPivotSpeed(double speed)
    {
        isAtTargetPivotPosition = false;
        isPivotPositioningStarted = false;
        leftPivotMotor.set(speed);
        releasePivotBrake();
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
        SmartDashboard.putBoolean(ArmKeys.deliveryExtensionPosition, isAtDeliveryExtensionPosition());
        SmartDashboard.putBoolean(ArmKeys.maxPivotPosition, !(isAtMaxPivotBackPosition() || isAtMaxPivotFrontPosition()));
        SmartDashboard.putBoolean(ArmKeys.homePivotPosition, isAtHomePivotPosition());
        SmartDashboard.putBoolean(ArmKeys.deliveryPivotPosition, isAtDeliveryPivotPosition());
        SmartDashboard.putNumber(ArmKeys.currentPivotLeftPosition, leftPivotPosition);
        SmartDashboard.putNumber(ArmKeys.currentPivotRightPosition, rightPivotPosition);

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;

public class Arm extends SubsystemBase
{
    private final TalonSRX leftPivotMotor = new TalonSRX(PivotConstants.leftMotorChannel);
    private final TalonSRX rightPivotMotor = new TalonSRX(PivotConstants.rightMotorChannel);
    private final VictorSPX extenderMotor = new VictorSPX(ArmConstants.extenderMotorChannel);

    private final TalonSRX encodedPivotMotor = leftPivotMotor;

    private final DigitalInput maxPivotFrontPosition = new DigitalInput(ArmConstants.maxPivotFrontPositionChannel);
    private final DigitalInput maxPivotBackPosition = new DigitalInput(ArmConstants.maxPivotBackPositionChannel);
    private final DigitalInput homePivotPosition = new DigitalInput(ArmConstants.homePivotPositionChannel);
    private final DigitalInput deliveryPivotPosition = new DigitalInput(ArmConstants.deliveryPivotPositionChannel);

    private final DigitalInput maxExtensionPosition = new DigitalInput(ArmConstants.maxExtensionPositionChannel);
    private final DigitalInput minExtensionPosition = new DigitalInput(ArmConstants.minExtensionPositionChannel);
    private final DigitalInput deliveryExtensionPosition = new DigitalInput(ArmConstants.deliveryExtensionPositionChannel);

    /** Creates a new Arm. */
    public Arm()
    {
        leftPivotMotor.setInverted(PivotConstants.isLeftMotorInverted);
        rightPivotMotor.setInverted(PivotConstants.isRightMotorInverted);
		rightPivotMotor.set(ControlMode.Follower, leftPivotMotor.getDeviceID());

		for (TalonSRX motor : new TalonSRX[] { leftPivotMotor, rightPivotMotor})
		{
			motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
			motor.configAllowableClosedloopError(PivotConstants.pidProfileSlot, PivotConstants.pidAllowableError,
            PivotConstants.pidTimeoutMs);
			motor.configClosedLoopPeakOutput(PivotConstants.pidProfileSlot, PivotConstants.pidPeakOutput,
            PivotConstants.pidTimeoutMs);
			motor.configClosedLoopPeriod(PivotConstants.pidProfileSlot, PivotConstants.pidLoopPeriodMs,
                PivotConstants.pidTimeoutMs);
			motor.config_kP(PivotConstants.pidProfileSlot, PivotConstants.pidP,
                PivotConstants.pidTimeoutMs);
			motor.config_kI(PivotConstants.pidProfileSlot, PivotConstants.pidI,
                PivotConstants.pidTimeoutMs);
			motor.config_kD(PivotConstants.pidProfileSlot, PivotConstants.pidD,
                PivotConstants.pidTimeoutMs);
			motor.config_kF(PivotConstants.pidProfileSlot, PivotConstants.pidFF,
                PivotConstants.pidTimeoutMs);
			motor.config_IntegralZone(PivotConstants.pidProfileSlot, PivotConstants.pidIZ,
                PivotConstants.pidTimeoutMs);
			motor.selectProfileSlot(PivotConstants.pidProfileSlot, PivotConstants.primaryClosedLoop);
			motor.setSensorPhase(PivotConstants.isSensorPhaseInverted);
            motor.setNeutralMode(PivotConstants.neutralMode);
		}


        // Tell the motor controller with the attached encoder how to access the encoder:
        //encodedPivotMotor.sensor
    }

    /**
     * Redsets the pivot position counter to 0.
     */
    public void resetPivotPosition()
    {
        leftPivotMotor.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean(ArmKeys.extensionLimit, !(isAtMaxExtensionLimit() || isAtMinExtensionLimit()));
        SmartDashboard.putBoolean(ArmKeys.deliveryExtensionPosition, isAtDeliveryExtensionPosition());
        SmartDashboard.putBoolean(ArmKeys.maxPivotPosition, !(isAtMaxPivotBackPosition() || isAtMaxPivotFrontPosition()));
        SmartDashboard.putBoolean(ArmKeys.homePivotPosition, isAtHomePivotPosition());
        SmartDashboard.putBoolean(ArmKeys.deliveryPivotPosition, isAtDeliveryPivotPosition());
    }

    private boolean isAtMaxPivotFrontPosition()
    {
        return maxPivotFrontPosition.get();
    }

    private boolean isAtMaxPivotBackPosition()
    {
        return maxPivotBackPosition.get();
    }

    private boolean isAtHomePivotPosition()
    {
        return homePivotPosition.get();
    }

    private boolean isAtDeliveryPivotPosition()
    {
        return deliveryPivotPosition.get();
    }

    private boolean isAtMinExtensionLimit()
    {
        return minExtensionPosition.get();
    }

    private boolean isAtMaxExtensionLimit()
    {
        return maxExtensionPosition.get();
    }

    private boolean isAtDeliveryExtensionPosition()
    {
        return deliveryExtensionPosition.get();
    }

}

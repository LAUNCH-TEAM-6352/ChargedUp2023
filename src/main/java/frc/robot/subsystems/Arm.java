// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DashboardConstants.ArmKeys;

public class Arm extends SubsystemBase
{
    private final VictorSPX leftPivotMotor = new VictorSPX(ArmConstants.leftPivotMotorChannel);
    private final VictorSPX rightPivotMotor = new VictorSPX(ArmConstants.rightPivotMotorChannel);
    private final VictorSPX extenderMotor = new VictorSPX(ArmConstants.extenderMotorChannel);

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

    private boolean isAtMaxPivotFrontPosition()
    {
        return maxPivotFrontPosition.get();
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
}

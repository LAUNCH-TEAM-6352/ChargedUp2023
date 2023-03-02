// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.DashboardConstants.DriveToPositionPidKeys;
import frc.robot.Constants.DashboardConstants.DriveTrainKeys;
import frc.robot.Constants.DriveTrainConstants.DriveToPositionPidDefaultValues;
import frc.robot.Constants.DashboardConstants;

/***
 * Models the robot's drive train.
 * 
 * TODO: Add IMU sensor and methods for getting and resetting robot's current angle.
 */
public class DriveTrain extends SubsystemBase
{
    // Lists of motors on left and right side of drive train:
    private final List<CANSparkMax> leftMotors = new ArrayList<CANSparkMax>();
    private final List<CANSparkMax> rightMotors = new ArrayList<CANSparkMax>();

    // A map of motors:
    private final Map<Integer, CANSparkMax> motors = new HashMap<Integer, CANSparkMax>();

    // The following used during PID control:
    private CANSparkMax pidLeader = null;
    private double targetPosition;

    // The current open loop ramp rate:
    private double openLoopRampRate = DriveTrainConstants.defaultOpenLoopRampRate;

    // The gyro:
    private ADIS16470_IMU adis16470Imu;

    /**
     * Creates a new DriveTrain.
     */
    public DriveTrain()
    {
        // Construct the motors on the left side of the drive train:
        for (int channel : DriveTrainConstants.leftMotorChannels)
        {
            var motor = new CANSparkMax(channel, MotorType.kBrushless);
            setCommonConfig(motor);
            motor.setInverted(DriveTrainConstants.areLeftMotorsInverted);
            leftMotors.add(motor);
            motors.put(Integer.valueOf(channel), motor);
        }

        // Construct the motors on the right side of the drive train:
        for (int channel : DriveTrainConstants.rightMotorChannels)
        {
            var motor = new CANSparkMax(channel, MotorType.kBrushless);
            setCommonConfig(motor);
            motor.setInverted(DriveTrainConstants.areRightMotorsInverted);
            rightMotors.add(motor);
            motors.put(Integer.valueOf(channel), motor);
        }

        // Construct the IMU:
        adis16470Imu = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kX, SPI.Port.kOnboardCS0, ADIS16470_IMU.CalibrationTime._4s);
    }

    /**
     * Make sure motor controllers are configured for driver control:
     */
    public void configureForDriverControl()
    {
        leftMotors.forEach((motor) -> motor.follow(ExternalFollower.kFollowerDisabled, 0));
        rightMotors.forEach((motor) -> motor.follow(ExternalFollower.kFollowerDisabled, 0));
        openLoopRampRate = SmartDashboard.getNumber(DriveTrainKeys.openLoopRampRate, DriveTrainConstants.defaultOpenLoopRampRate);
        setOpenLoopRampRate(openLoopRampRate);
        setIdleMode(DriveTrainConstants.defaultIdleMode);
    }

    /**
     * Sets up PID controller and leader and followers for PID control.
     */
    public void configureForPositionPidControl()
    {
        // set up leader and followers:
        pidLeader = null;
        leftMotors.forEach((motor) ->
        {
            // Make the first left motor the leader for PID:
            if (pidLeader == null)
            {
                pidLeader = motor;
            }
            else
            {
                motor.follow(pidLeader);
            }
        });

        rightMotors.forEach((motor) ->
        {
            motor.follow(pidLeader, DriveTrainConstants.areLeftMotorsInverted != DriveTrainConstants.areRightMotorsInverted);
        });

        // Set closed loop ramp rate on the leader:
        pidLeader.setClosedLoopRampRate(
            SmartDashboard.getNumber(DriveTrainKeys.closedLoopRampRate, DriveTrainConstants.defaultClosedLoopRampRate));
        
        // Set PID controller parameters on the leader:
        var pidController = pidLeader.getPIDController();
        pidController.setP(SmartDashboard.getNumber(DriveToPositionPidKeys.kP, DriveToPositionPidDefaultValues.kP));
        pidController.setI(SmartDashboard.getNumber(DriveToPositionPidKeys.kI, DriveToPositionPidDefaultValues.kI));
        pidController.setD(SmartDashboard.getNumber(DriveToPositionPidKeys.kD, DriveToPositionPidDefaultValues.kD));
        pidController.setIZone(SmartDashboard.getNumber(DriveToPositionPidKeys.kIZ, DriveToPositionPidDefaultValues.iZone));
        pidController.setFF(SmartDashboard.getNumber(DriveToPositionPidKeys.kFF, DriveToPositionPidDefaultValues.ff));
        pidController.setOutputRange(
            SmartDashboard.getNumber(DriveToPositionPidKeys.minOutput, DriveToPositionPidDefaultValues.minOutput),
            SmartDashboard.getNumber(DriveToPositionPidKeys.maxOutput, DriveToPositionPidDefaultValues.maxOutput));
    }

    /**
     * Using a PID controller, drives the robot to a specific position,
     * where position is expresed in encoder rotations.
     * 
     * @param position encoder counts
     */
    public void driveToPosition(double position)
    {
        targetPosition = position;
        pidLeader.getPIDController().setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber(DriveToPositionPidKeys.target, position);
    }

    /**
     * Determines if PID controller is at target position.
     * 
     * @return true if at the target position
     */
    public boolean isAtTargetPosition()
    {
        return Math.abs(pidLeader.getEncoder().getPosition() - targetPosition) < DriveToPositionPidDefaultValues.tolerance;
    }
        
    /**
     * Team Caution style drive using input from two joysticks, left and right.
     * 
     * @param leftStick
     * @param rightStick
     */
    public void driveCaution(Joystick leftStick, Joystick rightStick)
    {
        setTunedOutputs(leftStick.getY() - rightStick.getX(), leftStick.getY() + rightStick.getX());
    }

    /**
     * Team Caution style drive using input from the joysticks on one Xbox controller.
     * 
     * @param controller
     */
    public void driveCaution(XboxController controller)
    {
        setTunedOutputs(controller.getLeftY() - controller.getRightX(), controller.getLeftY() + controller.getRightX());
    }

    /**
     * Squares inputs for more sensitivity.
     * 
     * @param left
     * @param right
     */
    public void setTunedOutputs(double left, double right)
    {
        left = MathUtil.clamp(left, -1.0, +1.0);
        right = MathUtil.clamp(right, -1.0, +1.0);

        double leftOut = Math.copySign(left * left * left, left);
        double rightOut = Math.copySign(right * right * right, right);
        
        setPercentage(leftMotors, leftOut);
        setPercentage(rightMotors, rightOut);

        SmartDashboard.putNumber(DriveTrainKeys.leftPercentOutput, leftOut);
        SmartDashboard.putNumber(DriveTrainKeys.rightPercentOutput, rightOut);
    }

    /**
     * Sets un-tuned motor percentages.
     * 
     * @param percentage
     */
    public void setRawMotorOutputs(double percentage)
    {
        percentage = MathUtil.clamp(percentage, -1.0, +1.0);

        setPercentage(leftMotors, percentage);
        setPercentage(rightMotors, percentage);

        SmartDashboard.putNumber(DriveTrainKeys.leftPercentOutput, percentage);
        SmartDashboard.putNumber(DriveTrainKeys.rightPercentOutput, percentage);
    }

	/**
	 * Stop the drive.
	 */
	public void stop()
	{
        leftMotors.forEach((motor) -> motor.stopMotor());
        rightMotors.forEach((motor) -> motor.stopMotor());

        SmartDashboard.putNumber(DriveTrainKeys.leftPercentOutput, 0);
        SmartDashboard.putNumber(DriveTrainKeys.rightPercentOutput, 0);
	}

    /**
     * Runs the motor at the specified channel at the specified percentage.
     * 
     * @param channel
     * @param percentage
     */
    public void set(int channel, double percentage)
    {
        var motor = motors.get(Integer.valueOf(channel));
        if (motor != null)
        {
            motor.set(percentage);
        }
    }

    /**
     * Stops the motor at the speficied channel.
     * 
     * @param channel
     */
    public void stop(int channel)
    {
        var motor = motors.get(Integer.valueOf(channel));
        if (motor != null)
        {
            motor.stopMotor();
        }
    }

    /**
     * Resets encoder position on all motors.
     */
    public void resetPosition()
    {
        leftMotors.forEach((motor) -> motor.getEncoder().setPosition(0));
        rightMotors.forEach((motor) -> motor.getEncoder().setPosition(0));
    }

    @Override
    public void periodic()
    {
        if (!leftMotors.isEmpty())
        {
            SmartDashboard.putNumber(DriveTrainKeys.leftPosition, leftMotors.get(0).getEncoder().getPosition());
            // SmartDashboard.putNumber("DT Left Applied", leftMotors.get(0).getAppliedOutput());
        }

        if (!rightMotors.isEmpty())
        {
            SmartDashboard.putNumber(DriveTrainKeys.rightPosition, rightMotors.get(0).getEncoder().getPosition());
            // SmartDashboard.putNumber("DT Right Applied", rightMotors.get(0).getAppliedOutput());
        }

        SmartDashboard.putNumber("DT Angle", getAngle());
    }

    /**
     * Sets configuration values common to all motors.
     * 
     * @param motor
     */
    private void setCommonConfig(CANSparkMax motor)
    {
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setIdleMode(DriveTrainConstants.defaultIdleMode);
    }

    /**
     * Sets the open loop ramp rate on all motors.
     */
    private void setOpenLoopRampRate(double rampRate)
    {
        leftMotors.forEach((motor) -> motor.setOpenLoopRampRate(rampRate));
        rightMotors.forEach((motor) -> motor.setOpenLoopRampRate(rampRate));
    }

    /**
     * Set motors to brake when idle.
     */
    public void setIdleBrake()
    {
        setIdleMode(IdleMode.kBrake);
    }

    /**
     * Set motors to coast when idle.
     */
    public void setIdleCoast()
    {
        setIdleMode(IdleMode.kCoast);
    }

    /**
     * Sets the idle mode on all motors.
     * 
     * @param idleMode
     */
    private void setIdleMode(IdleMode idleMode)
    {
        leftMotors.forEach((motor) -> motor.setIdleMode(idleMode));
        rightMotors.forEach((motor) -> motor.setIdleMode(idleMode));
    }

    /**
     * Sets the speeds of the specified motors to the specified percentage.
     * 
     * @param motors The motors to adjust.
     * @param percentage The new percentage. 
     */
    private void setPercentage(List<CANSparkMax> motors, double percentage)
    {
        motors.forEach((motor) -> motor.set(percentage));
    }

    /**
     * Returns the angle of the robot in degrees betwween -180 and +180.
     * 
     * @return the robot angle between -180 and +180 degrees,
     */
    public double getAngle()
    {
        var angle = adis16470Imu.getAngle();
        return angle > 180 ? 360.0 - angle : angle;
    }

    public void resetAngle()
    {
        adis16470Imu.reset();
    }
}

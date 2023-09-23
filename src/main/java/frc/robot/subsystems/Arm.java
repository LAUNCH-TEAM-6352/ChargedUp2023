// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
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

    // The following keeps track of if the brake has been set when driving manually:
    private boolean isPivotBrakeSet;


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
        else if (!areExtenderAndPivotPositionsLegal(position, getPivotPosition()))
        {
            position = getLargestLegalExtenderPosition();
        }

        extenderTargetPosition = position;
        extenderTargetTolerance = tolerance;
        lastExtenderPosition = getExtenderPosition();
        var minOutput = SmartDashboard.getNumber(ArmKeys.extenderPidMinOutput, ExtenderConstants.PIDConstants.defaultMinOutput);
        var maxOutput = SmartDashboard.getNumber(ArmKeys.extenderPidMaxOutput, ExtenderConstants.PIDConstants.defaultMaxOutput);
        extenderMotor.getPIDController().setOutputRange(minOutput, maxOutput);
        extenderMotor.getPIDController().setReference(position, ControlType.kPosition);
        isExtenderAtTargetPosition = false;
        isExtenderPositioningStarted = true;
	}

    public void setExtenderSpeed(double speed)
    {
        if (!areExtenderAndPivotPositionsLegal())
        {
            speed = 0;
            rightRumbleOn();
        }
        else
        {
            rightRumbleOff();
        }

        // When explicitly setting speed, indicate we are not positioning via PID:
        isExtenderAtTargetPosition = false;
        isExtenderPositioningStarted = false;

        extenderMotor.set(speed);
    }

    /**
     * Redsets the extender encoder position to the min (starting) position.
     */
    public void resetExtenderPosition()
    {
        extenderMotor.getEncoder().setPosition(ExtenderConstants.minPosition);
    }

    public double getExtenderPosition()
    {
        return Math.max(extenderMotor.getEncoder().getPosition(), ExtenderConstants.minPosition);
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
        else if (!areExtenderAndPivotPositionsLegal(getExtenderPosition(), position))
        {
            position = getLargestLegalPivotPosition();
        }

        pivotTargetPosition = position;
        pivotTargetTolerance = tolerance;
        lastPivotPosition = getPivotPosition();
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
        if ((speed > 0 && isPivotAtFwdLimit()) ||
            (speed < 0 && (isPivotAtRevLimit() || 
             !areExtenderAndPivotPositionsLegal())))
        {
            speed = 0;
            leftRumbleOn();
        }
        else
        {
            leftRumbleOff();
        }

        if (speed == 0 && !isPivotBrakeSet)
        {
            setPivotBrake();
            isPivotBrakeSet = true;
        }

        // When explicitly setting speed, indicate we are not positioning via PID:
        isPivotAtTargetPosition = false;
        isPivotPositioningStarted = false;

        leftPivotMotor.set(speed);
        
        if (speed != 0)
        {
            releasePivotBrake();
            isPivotBrakeSet = false;
        }
    }

    public void stopPivot()
    {
        setPivotBrake();
        isPivotBrakeSet = true;
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
     * Redsets the pivot position the home (start) position.
     */
    public void resetPivotPosition()
    {
        leftPivotMotor.getEncoder().setPosition(PivotConstants.startPosition);
        rightPivotMotor.getEncoder().setPosition(PivotConstants.startPosition);
    }

    public double getPivotPosition()
    {
        return leftPivotMotor.getEncoder().getPosition();
    }

    /**
     * Returns the angle (in radians) of the arm relative to
     * the front of the robot for the current pivot position.
     */
    public double getPivotAngleInRadians()
    {
        return getPivotAngleInRadians(getPivotPosition());
    }

    /**
     * Returns the angle (in radians) of the arm relative to
     * the front of the robot for the specified pivot position.
     */
    public double getPivotAngleInRadians(double pivotPosition)
    {
        return pivotPosition * PivotConstants.radiansPerMotorShaftRotation;
    }

    /**
     * Returns the angle (in degreres) of the arm relative to
     * the front of the robot for the current pivot position.
     */
    public double getPivotAngleInDegrees()
    {
        return Math.toDegrees(getPivotAngleInRadians());
    }

    /**
     * Returns the angle (in degreres) of the arm relative to
     * the front of the robot for the specified pivot position.
     */
    public double getPivotAngleInDegrees(double pivotPosition)
    {
        return Math.toDegrees(getPivotAngleInRadians(pivotPosition));
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

    /**
     * Determines if current arm extender and pivot positions are legal.
     * This default version of this method applies a fudge factor to the
     * pivot position (angle) to account for hysterisis in the measurement
     * of the arm angle.
     */
    public boolean areExtenderAndPivotPositionsLegal()
    {
        // If the arm is angled towards the front of the robot, apply
        // the fudge factor to hopefully get more accurate results:
        var pivotPosition = getPivotPosition();
        if (pivotPosition < PivotConstants.verticalPosition)
        {
            // Treat below horizontal the same as above horizontal.
            // Afterall, cos(a) = cos(-a)
            pivotPosition = Math.abs(pivotPosition);

            // Apply fudge factor in a limited number of stages
            // as we seem to get more error for larger angles:
            int i = PivotConstants.fudgeFactorForCosineLimitMaxApplications;
            while (pivotPosition >= PivotConstants.fudgeFactorForCosineLimit && i > 0)
            {
                pivotPosition -= PivotConstants.fudgeFactorForCosineLimit;
                i--;
            }

            if (Constants.DEBUG)
            {
                SmartDashboard.putNumber("Fudged Pivot Pos", pivotPosition);
            }
        }

        return areExtenderAndPivotPositionsLegal(getExtenderPosition(), pivotPosition);
    }

    public boolean areExtenderAndPivotPositionsLegal(double extenderPosition, double pivotPosition)
    {
        return ExtenderConstants.maxFrontHorizontalPosition / extenderPosition > Math.cos(getPivotAngleInRadians(pivotPosition));
    }

    /**
     * Computes and returns the largest legal extender
     * position for the current pivot position.
     */
    public double getLargestLegalExtenderPosition()
    {
        return ExtenderConstants.maxFrontHorizontalPosition / Math.cos(getPivotAngleInRadians());
    }

    /**
     * Computes and returns the largest legal pivot
     * position for the current extender position.
     */
    public double getLargestLegalPivotPosition()
    {
        var radians = Math.acos(ExtenderConstants.maxFrontHorizontalPosition / getExtenderPosition());
        return radians / PivotConstants.radiansPerMotorShaftRotation;
    }

    @Override
    public void periodic()
    {
        // Get the current extender position:
        var extenderPosition = getExtenderPosition();

        // Get the current pivot position:
        var pivotPosition = getPivotPosition();

        SmartDashboard.putNumber(ArmKeys.extenderPosition, extenderPosition);
        SmartDashboard.putBoolean(ArmKeys.legalExtenderAndPivotPositions, areExtenderAndPivotPositionsLegal());
        SmartDashboard.putNumber(ArmKeys.pivotCurLeftPosition, pivotPosition);
        if (Constants.DEBUG)
        {
            SmartDashboard.putNumber(ArmKeys.pivotCurAngleDegrees, getPivotAngleInDegrees(pivotPosition));
            SmartDashboard.putNumber("Arm cos(pivotAngle)", Math.cos(getPivotAngleInRadians(pivotPosition)));
            SmartDashboard.putNumber("Arm adj|hyp", ExtenderConstants.maxFrontHorizontalPosition / getExtenderPosition());
            SmartDashboard.putNumber("Arm max legal ext", getLargestLegalExtenderPosition());
            SmartDashboard.putNumber("Arm max legal pivot", getLargestLegalPivotPosition());
        }

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
            if (Math.abs(pivotPosition - pivotTargetPosition) < pivotTargetTolerance && Math.abs(pivotPosition - lastPivotPosition) < pivotTargetTolerance)
            {
                isPivotPositioningStarted = false;
                isPivotAtTargetPosition = true;
            }
            else
            {
                lastPivotPosition = pivotPosition;
            }
        }
    }

}
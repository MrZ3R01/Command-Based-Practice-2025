package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{
    private SparkMax armM = new SparkMax(Constants.Arm.CANIDs.ARM, MotorType.kBrushless);
    private RelativeEncoder armEncoder = armM.getEncoder();;
    private SparkMaxConfig configA = new SparkMaxConfig();
    private ArmFeedforward armFeedforward = new ArmFeedforward(Constants.Arm.Feedforward.kS,
    Constants.Arm.Feedforward.kG, Constants.Arm.Feedforward.kV);
    private ProfiledPIDController armPID = new ProfiledPIDController(
        Constants.Arm.PID.kP, Constants.Arm.PID.kI, Constants.Arm.PID.kD, new TrapezoidProfile.Constraints(
        Constants.Arm.PID.constraints.kMAXV, Constants.Arm.PID.constraints.kMAXACC));
    public Arm(){
        configA.smartCurrentLimit(Constants.Arm.ARM_CURRENT_LIMIT);        
        configA.encoder.positionConversionFactor(Constants.Arm.ENCODER_TO_RADIANS);
        armM.configure(configA,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        armPID.setTolerance(0.03);
        armEncoder.setPosition(0);
        armPID.setGoal(Constants.Arm.PID.setpoints.GROUND);

    }
    
    public void setArmVoltage(double voltage){
        armM.setVoltage(MathUtil.clamp(voltage,-12,12));
    }
    
    public void changeArmGoal(double newGoal){
        armPID.reset(armEncoder.getPosition());
        armPID.setGoal(newGoal);
    }
    
    public Command changeArmGoalCommand(double newGoal){
        return runOnce(()->changeArmGoal(newGoal));
    }
    

    public void  setArmMovement(){
        double feedforward = armFeedforward.calculate(armPID.getSetpoint().position,
            armPID.getSetpoint().velocity);
        double PID = armPID.calculate(armEncoder.getPosition());
        setArmVoltage(feedforward + PID);
    }

    public Command setArmMovementCommand(){
        return Commands.sequence(
            resetPID(),
            run(()->setArmMovement()));
    }

    public Command resetPID(){
        return runOnce(()->armPID.reset(armEncoder.getPosition()));
    }

    public boolean armAtGoal(){
        return armPID.atGoal();
    }
    
    public Command moveArmToNewGoal(double newGoal){
        return Commands.sequence(
            resetPID(),
            changeArmGoalCommand(newGoal),
            setArmMovementCommand().until(this::armAtGoal));
    }

    public Command resetEncoderCommand(){
        return runOnce(()->armEncoder.setPosition(0));
    }

    @Override
    public void periodic(){
        System.out.println(armEncoder.getPosition());
    }
        };
package frc.robot.Subsystems;


import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    private SparkMax leftM = new SparkMax(Constants.Drivetrain.CANIDs.LEFT, MotorType.kBrushless);
    private SparkMax rightM = new SparkMax(Constants.Drivetrain.CANIDs.RIGHT, MotorType.kBrushless);
    private SparkMaxConfig configD = new SparkMaxConfig();
    private SlewRateLimiter drivetrainSlew = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT);
    private DifferentialDrive drivetrain = new DifferentialDrive(leftM,rightM);

    public Drivetrain(){
        configD.smartCurrentLimit(Constants.Drivetrain.CURRENT_LIMIT);
        rightM.configure(configD,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        configD.inverted(true);
        leftM.configure(configD, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    public Command arcadeDrive(DoubleSupplier speed, DoubleSupplier rotation){
        return run(()->drivetrain.arcadeDrive(drivetrainSlew.calculate(speed.getAsDouble()), rotation.getAsDouble()));
    }
}


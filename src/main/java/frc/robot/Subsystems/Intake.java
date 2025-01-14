package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private SparkMax intakeM = new SparkMax(Constants.Arm.CANIDs.INTAKE, MotorType.kBrushless);
    private SparkMaxConfig configI = new SparkMaxConfig();
    public Intake(){
        configI.smartCurrentLimit(Constants.Arm.INTAKE_CURRENT_LIMIT);
        intakeM.configure(configI,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }
    //Command for intaking bag
    public Command intakeBag(){
        return runOnce(()->intakeM.setVoltage(5));
    }
    //Command for outtaking bag
    public Command outtakeBag(){
        return runOnce(()->intakeM.setVoltage(-6));
    }
    //Command for stopping the intake
    public Command stopIntakeM(){
        return runOnce(()->intakeM.stopMotor());
    }
}

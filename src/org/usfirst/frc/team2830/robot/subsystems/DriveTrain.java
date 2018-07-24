package org.usfirst.frc.team2830.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public static WPI_VictorSPX victorLeft;
	public static WPI_TalonSRX talonLeft;
	public static WPI_VictorSPX victorRight;
	public static WPI_TalonSRX talonRight;
	public static DifferentialDrive robotDrive;
	public static final int LEFT_ENCODER = 0;
	public static final int RIGHT_ENCODER = 1;
	public static AHRS gyro;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}

	public DriveTrain(){
		victorLeft = new WPI_VictorSPX(14);
		talonLeft = new WPI_TalonSRX(15);
		victorRight = new WPI_VictorSPX(21);
		talonRight = new WPI_TalonSRX(20);

		talonRight.setInverted(true);
		victorRight.setInverted(true);
		talonLeft.setInverted(false);
		victorLeft.setInverted(false);

		talonLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		talonRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

		victorLeft.follow(talonLeft);
		victorRight.follow(talonRight);


		talonLeft.configOpenloopRamp(.01, 10);
		talonRight.configOpenloopRamp(.01, 10);

		talonLeft.setSensorPhase(true);

		talonLeft.configPeakOutputForward(1, 10);
		talonLeft.configPeakOutputReverse(-1, 10);

		talonRight.setSensorPhase(true);

		talonRight.configPeakOutputForward(1, 10);
		talonRight.configPeakOutputReverse(-1, 0);
		
		gyro = new AHRS(SerialPort.Port.kUSB1);
	}

	public int getEncoderValue(int encoder){
		if (encoder == LEFT_ENCODER){
			return talonLeft.getSelectedSensorPosition(0);
		}
		else if(encoder == RIGHT_ENCODER){
			return talonRight.getSelectedSensorPosition(0);
		}
		return 0;
	}

	public double getAngle(){
		return gyro.getYaw();
	}
	
	public void setLeft(double output){
		talonLeft.set(output);
	}
	
	public void setRight(double output){
		talonRight.set(output);
	}
	
	public void writeToSmartDashboard(){
		SmartDashboard.putNumber("Left Encoder Distance", getEncoderValue(LEFT_ENCODER));
		SmartDashboard.putNumber("Right Encoder Distance", getEncoderValue(RIGHT_ENCODER));
		SmartDashboard.putNumber("Gyro Angle", getAngle());
	}
	
	public void resetCounters(){
		gyro.zeroYaw();
		talonLeft.setSelectedSensorPosition(0,0,10);
		talonRight.setSelectedSensorPosition(0,0,10);
	}
	
	
}



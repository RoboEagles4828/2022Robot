package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Intake {
    private WPI_TalonFX intake;

    public Intake(){
        intake = new WPI_TalonFX(Constants.intake_port);
    }

    public void set_speed(double speed){
        intake.set(speed);
    }
}

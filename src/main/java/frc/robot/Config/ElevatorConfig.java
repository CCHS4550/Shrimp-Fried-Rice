package frc.robot.Config;

public class ElevatorConfig {
    public double elevatorKp;
    public double elevatorKi;
    public double elevatorKd;
    public double elevatorKv;
    public double elevatorKs;
    public double elevatorKg;

    public ElevatorConfig withElevatorKp(double elevatorKp){
        this.elevatorKp = elevatorKp;
        return this;
    }
    public ElevatorConfig withElevatorKi(double elevatorKi){
        this.elevatorKi = elevatorKi;
        return this;
    }
    public ElevatorConfig withElevatorKd(double elevatorKd){
        this.elevatorKd = elevatorKd;
        return this;
    }
    public ElevatorConfig withElevatorKv(double elevatorKv){
        this.elevatorKv = elevatorKv;
        return this;
    }
    public ElevatorConfig withElevatorKs(double elevatorKs){
        this.elevatorKs = elevatorKs;
        return this;
    }

    public ElevatorConfig withElevatorKg (double elevatorKg){
        this.elevatorKg = elevatorKg;
        return this;
    }
}

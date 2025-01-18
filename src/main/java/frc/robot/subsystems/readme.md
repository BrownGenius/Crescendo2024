Subsystem and IO interfaces and implementations

Subsystemts utilize the [AdvantageKit IO Layer](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/RECORDING-INPUTS.md) paradigm.


Each subsystem is implemented using the following breakdown.  Here, assume we have a mechanism called a "Gizmo":
* Hardware IO
   * _Simple_ Interface (defines the desired _hardware_ functionality)
      * Gizmo*IO*.java
        ```
        public static class GizmoIOInputs {
            public double velocityRadPerSec = 0.0;
            public double appliedVolts = 0.0;
            public double current;
        }

        void updateInputs(GizmoIOInputs inputs); ← gets current sensor readings

        void setVoltage(double volts); ← sets desired voltage
        ```
   * One _or more_ Implementation(s)
      * GizmoIO*Stub*.java ← Simulated implementation
        ```
        public class GizmoIOStub implements GizmoIO {
        ```
      * GizmoIO*SparkMax*.java ← SparkMax motor based implementation
      * GizmoIO*Etc*.java
* Subsystem (utilizes one or more Hardware IO implementation)
   * _Single_ Interface (defines the desired _subsystem_ functionality)
      * Gizmo*Subsystem*.java
        * Low Level/Debug Controls
            ```
            void runVoltage(double volts);
            double getCurrentVoltage();
            void add2dSim(Mechanism2d mech2d);
            ```
        * High Level Controls
            ```
            Command getTurnOffCommand();
            Command getTurnOnCommand();
            ```
   * One _or more_ Implementation(s)
      * GizmoSubsystem*Simple*.java
        ```
        public class GizmoSubsystemSimple implements Gizmo {
            GizmoSubsystemSimple(GizmoIO io); ← an IO instance is passed into the GizmoSubsystemSimple

            void periodic() {
                IO.updateInputs(inputs);
                Logger.processInputs("Intake", inputs)
            }
            ...
        }
        ```
      * GizmoSubsystem*Advanced*.java
        ```
        public class GizmoSubsystemAdvanced implements Gizmo {
            ...
        }
        ```
      * GizmoSubsystem*Etc*.java

It is important to note that the *same* Hardware IO can be shared by completely different subsystems.  E.g. we may just want a Hardware IO layer abstraction for basic functionality  E.g.
* MotorIO
   * MotorIOSparkMax
   * MotorIOTalonSRX
* LimitSwitchIO

And then a subsystem can consist of 1 or more MotorIO instances and 1 or more LimitSwitchIO instances.

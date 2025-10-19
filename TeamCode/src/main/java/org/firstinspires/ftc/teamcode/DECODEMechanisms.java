package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DECODEMechanisms {
    // === Hardware Components ===
    private final HardwareMap hardwareMap;

    // === Drive System ===
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private BNO055IMU imu;
    private double headingOffset = 0;
    private boolean isFieldCentric = true;

    // === Intake System ===
    private DcMotor intake;

    // === Spindexer System with Axon Encoder and Limit Switch ===
    private DcMotor spindexer;
    private DcMotorEx spindexerEnc; // Axon encoder for spindexer position
    private DigitalChannel spindexerLimitSwitch; // REV limit switch
    private int spindexerStep = 0;
    private static final double SPINDEXER_TICKS_PER_STEP = 1322;
    private boolean spindexerMoving = false;
    private double spindexerTargetPosition = 0;
    private boolean useLimitSwitch = true; // Enable/disable limit switch functionality

    // === Launcher System ===
    private DcMotorEx launcher;
    private Servo kicker;
    private double launcherRPM = 0;
    private static final double LAUNCHER_MAX_RPM = 5000;
    private static final double LAUNCHER_STEP = 250;
    private static final double TICKS_PER_REV = 28;

    // === Hood System with Axon Encoder ===
    private Servo hood;
    private DcMotorEx hoodEnc; // Axon encoder for hood position
    private double hoodPos = 0.5;
    private double hoodTargetPosition = 0.5;
    private static final double HOOD_MIN_POS = 0.0;
    private static final double HOOD_MAX_POS = 1.0;
    private static final double HOOD_ENCODER_TICKS_PER_REV = 1000;
    private static final int HOOD_ENC_MIN = 0; // Minimum encoder ticks for hood
    private static final int HOOD_ENC_MAX = 1000; // Maximum encoder ticks for hood

    // === Turret System ===
    private Servo turretSer; // Turret servo

    // === Continuous Servo Encoders ===
    private CRServo continuousServo1, continuousServo2;
    private DcMotor servoEncoder1, servoEncoder2;
    private double servo1TargetPosition = 0;
    private double servo2TargetPosition = 0;
    private static final double SERVO_ENCODER_TICKS_PER_REV = 1120;
    private static final double SERVO_POWER = 0.8;

    // === PID Controller ===
    private PIDController launcherPID;
    private PIDController servo1PID, servo2PID;
    private PIDController hoodPID;
    private PIDController spindexerPID;

    private static final double LAUNCHER_KP = 0.02;
    private static final double LAUNCHER_KI = 0.001;
    private static final double LAUNCHER_KD = 0.005;
    private static final double SERVO_KP = 0.01;
    private static final double SERVO_KI = 0.0005;
    private static final double SERVO_KD = 0.002;
    private static final double HOOD_KP = 0.015;
    private static final double HOOD_KI = 0.0002;
    private static final double HOOD_KD = 0.002;
    private static final double SPINDEXER_KP = 0.01;
    private static final double SPINDEXER_KI = 0.0001;
    private static final double SPINDEXER_KD = 0.001;

    // === Timing ===
    private final ElapsedTime runtime = new ElapsedTime();

    public DECODEMechanisms(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initializeAllSystems();
    }

    // ================= INITIALIZATION METHODS =================

    private void initializeAllSystems() {
        initializeDriveSystem();
        initializeIMU();
        initializeIntakeSystem();
        initializeSpindexerSystem();
        initializeLauncherSystem();
        initializeHoodSystem();
        initializeTurretSystem();
        initializeContinuousServos();
        initializePIDControllers();
    }

    private void initializeDriveSystem() {
        try {
            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            // Drive motors not configured
        }
    }

    private void initializeIMU() {
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters params = new BNO055IMU.Parameters();
            params.mode = BNO055IMU.SensorMode.IMU;
            params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(params);
        } catch (Exception e) {
            // IMU not configured
        }
    }

    private void initializeIntakeSystem() {
        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            // Intake not configured
        }
    }

    private void initializeSpindexerSystem() {
        try {
            spindexer = hardwareMap.get(DcMotor.class, "spindexer");
            spindexerEnc = hardwareMap.get(DcMotorEx.class, "spindexerEnc"); // Axon encoder
            spindexerLimitSwitch = hardwareMap.get(DigitalChannel.class, "spindexerLimitSwitch");

            // Configure limit switch
            spindexerLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Initialize Axon encoder
            spindexerEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            // Spindexer system not configured
        }
    }

    private void initializeLauncherSystem() {
        try {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            kicker = hardwareMap.get(Servo.class, "kicker");

            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kicker.setPosition(0.0);
        } catch (Exception e) {
            // Launcher system not configured
        }
    }

    private void initializeHoodSystem() {
        try {
            hood = hardwareMap.get(Servo.class, "hood");
            hoodEnc = hardwareMap.get(DcMotorEx.class, "hoodEnc"); // Axon encoder

            hood.setPosition(hoodPos);

            // Initialize Axon encoder
            hoodEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hoodEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            // Hood system not configured
        }
    }

    private void initializeTurretSystem() {
        try {
            turretSer = hardwareMap.get(Servo.class, "turretSer");
            // Set turret to initial position
            turretSer.setPosition(0.5); // Center position
        } catch (Exception e) {
            // Turret system not configured
        }
    }

    private void initializeContinuousServos() {
        try {
            continuousServo1 = hardwareMap.get(CRServo.class, "continuousServo1");
            continuousServo2 = hardwareMap.get(CRServo.class, "continuousServo2");

            servoEncoder1 = hardwareMap.get(DcMotor.class, "servoEncoder1");
            servoEncoder2 = hardwareMap.get(DcMotor.class, "servoEncoder2");

            servoEncoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            servoEncoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            servoEncoder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            servoEncoder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            servoEncoder1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            servoEncoder2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            // Continuous servos not configured
        }
    }

    private void initializePIDControllers() {
        launcherPID = new PIDController(LAUNCHER_KP, LAUNCHER_KI, LAUNCHER_KD);
        servo1PID = new PIDController(SERVO_KP, SERVO_KI, SERVO_KD);
        servo2PID = new PIDController(SERVO_KP, SERVO_KI, SERVO_KD);
        hoodPID = new PIDController(HOOD_KP, HOOD_KI, HOOD_KD);
        spindexerPID = new PIDController(SPINDEXER_KP, SPINDEXER_KI, SPINDEXER_KD);
    }

    // ================= SPINDEXER LIMIT SWITCH METHODS =================

    /**
     * Check if the spindexer limit switch is pressed
     * @return true if limit switch is pressed (usually means at home position)
     */
    public boolean isSpindexerLimitPressed() {
        if (spindexerLimitSwitch != null) {
            // REV limit switches are normally HIGH (true) when not pressed
            // and LOW (false) when pressed. This may vary based on your wiring.
            return !spindexerLimitSwitch.getState();
        }
        return false;
    }

    /**
     * Home the spindexer using the limit switch
     * This will rotate the spindexer until the limit switch is pressed
     */
    public void homeSpindexer() {
        if (spindexer == null || spindexerLimitSwitch == null) return;

        spindexerMoving = true;
        spindexer.setPower(0.3); // Slow speed for homing

        // Home the spindexer by rotating until limit switch is pressed
        while (!isSpindexerLimitPressed() && spindexerMoving) {
            // This loop will be broken by the limit switch or external stop
            // In a real implementation, you might want to add a timeout
            try {
                Thread.sleep(10); // Small delay to prevent busyloop
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        spindexer.setPower(0);
        spindexerMoving = false;

        // Reset encoder position to zero at home position
        if (spindexerEnc != null) {
            spindexerEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        spindexerStep = 0; // Reset to step 0 at home position
    }

    /**
     * Move spindexer to a specific step with limit switch safety
     */
    private void moveSpindexerToStep(int step) {
        if (spindexer == null) return;

        // If using limit switch and we're at step 0, ensure we're homed
        if (useLimitSwitch && step == 0) {
            homeSpindexer();
            return;
        }

        spindexerMoving = true;
        int targetPos = (int)(step * SPINDEXER_TICKS_PER_STEP);
        spindexer.setTargetPosition(targetPos);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(0.6);
    }

    /**
     * Advance to next step with limit switch awareness
     */
    public void advanceSpindexer() {
        if (!spindexerMoving && spindexer != null) {
            int nextStep = (spindexerStep + 1) % 3;

            // If using limit switch, ensure we don't advance past limits
            if (useLimitSwitch) {
                // Add any additional safety checks here
            }

            spindexerStep = nextStep;
            moveSpindexerToStep(spindexerStep);
        }
    }

    /**
     * Set spindexer step with limit switch safety
     */
    public void setSpindexerStep(int step) {
        if (step >= 0 && step < 3 && !spindexerMoving && spindexer != null) {
            // If using limit switch, ensure we don't set invalid positions
            if (useLimitSwitch) {
                // Add any additional safety checks here
            }

            spindexerStep = step;
            moveSpindexerToStep(step);
        }
    }

    /**
     * Enable or disable limit switch functionality
     */
    public void setUseLimitSwitch(boolean useLimitSwitch) {
        this.useLimitSwitch = useLimitSwitch;
    }

    /**
     * Check if limit switch functionality is enabled
     */
    public boolean isUseLimitSwitch() {
        return useLimitSwitch;
    }

    /**
     * Stop spindexer movement immediately
     */
    public void stopSpindexer() {
        spindexerMoving = false;
        if (spindexer != null) {
            spindexer.setPower(0);
        }
    }

    public void updateSpindexer() {
        if (spindexerMoving && spindexer != null) {
            // If using limit switch and it gets pressed during movement, stop immediately
            if (useLimitSwitch && isSpindexerLimitPressed() && spindexerStep != 0) {
                stopSpindexer();
                // Optionally reset to home position
                spindexerStep = 0;
            }

            // Check if movement is complete
            if (!spindexer.isBusy()) {
                spindexer.setPower(0);
                spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spindexerMoving = false;
            }
        }
    }

    public int getSpindexerStep() {
        return spindexerStep;
    }

    public int getSpindexerPosition() {
        return spindexer != null ? spindexer.getCurrentPosition() : 0;
    }

    public boolean isSpindexerMoving() {
        return spindexerMoving;
    }

    public int getSpindexerEncoderTicks() {
        return spindexerEnc != null ? spindexerEnc.getCurrentPosition() : 0;
    }

    public double getSpindexerEncoderRevolutions() {
        return spindexerEnc != null ? spindexerEnc.getCurrentPosition() / SPINDEXER_TICKS_PER_STEP : 0;
    }

    // ================= HOOD SYSTEM METHODS WITH AXON ENCODER =================

    public void setHoodPosition(double position) {
        hoodTargetPosition = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        hoodPos = hoodTargetPosition;
        hood.setPosition(hoodPos);
    }

    public void setHoodPositionWithEncoder(double targetTicks) {
        if (hoodEnc != null) {
            // Convert encoder ticks to servo position (0-1 range)
            double normalizedPosition = (targetTicks - HOOD_ENC_MIN) / (HOOD_ENC_MAX - HOOD_ENC_MIN);
            setHoodPosition(normalizedPosition);
        }
    }

    public void adjustHoodPosition(double delta) {
        setHoodPosition(hoodPos + delta);
    }

    public double getHoodPosition() {
        return hoodPos;
    }

    public int getHoodEncoderTicks() {
        return hoodEnc != null ? hoodEnc.getCurrentPosition() : 0;
    }

    public double getHoodEncoderRevolutions() {
        return hoodEnc != null ? hoodEnc.getCurrentPosition() / HOOD_ENCODER_TICKS_PER_REV : 0;
    }

    public boolean isHoodAtPosition(double tolerance) {
        if (hoodEnc == null) return true;
        double currentPos = (hoodEnc.getCurrentPosition() - HOOD_ENC_MIN) / (HOOD_ENC_MAX - HOOD_ENC_MIN);
        return Math.abs(currentPos - hoodTargetPosition) <= tolerance;
    }

    // ================= TURRET SYSTEM METHODS =================

    public void setTurretPosition(double position) {
        if (turretSer != null) {
            turretSer.setPosition(Math.max(0.0, Math.min(1.0, position)));
        }
    }

    public void adjustTurretPosition(double delta) {
        if (turretSer != null) {
            double currentPos = turretSer.getPosition();
            setTurretPosition(currentPos + delta);
        }
    }

    public double getTurretPosition() {
        return turretSer != null ? turretSer.getPosition() : 0.5;
    }

    // ================= CONTINUOUS SERVO ENCODER METHODS =================

    public void setContinuousServo1Position(double revolutions) {
        servo1TargetPosition = revolutions * SERVO_ENCODER_TICKS_PER_REV;
    }

    public void setContinuousServo2Position(double revolutions) {
        servo2TargetPosition = revolutions * SERVO_ENCODER_TICKS_PER_REV;
    }

    public void moveContinuousServo1Revolutions(double revolutions) {
        if (continuousServo1 != null) {
            setContinuousServo1Position(getServo1PositionRevolutions() + revolutions);
        }
    }

    public void moveContinuousServo2Revolutions(double revolutions) {
        if (continuousServo2 != null) {
            setContinuousServo2Position(getServo2PositionRevolutions() + revolutions);
        }
    }

    public double getServo1PositionRevolutions() {
        return servoEncoder1 != null ? servoEncoder1.getCurrentPosition() / SERVO_ENCODER_TICKS_PER_REV : 0;
    }

    public double getServo2PositionRevolutions() {
        return servoEncoder2 != null ? servoEncoder2.getCurrentPosition() / SERVO_ENCODER_TICKS_PER_REV : 0;
    }

    public int getServo1EncoderTicks() {
        return servoEncoder1 != null ? servoEncoder1.getCurrentPosition() : 0;
    }

    public int getServo2EncoderTicks() {
        return servoEncoder2 != null ? servoEncoder2.getCurrentPosition() : 0;
    }

    public void setContinuousServo1Power(double power) {
        if (continuousServo1 != null) {
            continuousServo1.setPower(power);
        }
    }

    public void setContinuousServo2Power(double power) {
        if (continuousServo2 != null) {
            continuousServo2.setPower(power);
        }
    }

    public void stopContinuousServos() {
        setContinuousServo1Power(0);
        setContinuousServo2Power(0);
    }

    public void resetServoEncoders() {
        if (servoEncoder1 != null && servoEncoder2 != null) {
            servoEncoder1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            servoEncoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            servoEncoder1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            servoEncoder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            servo1TargetPosition = 0;
            servo2TargetPosition = 0;
        }
    }

    private void updateContinuousServos() {
        if (continuousServo1 != null && servoEncoder1 != null) {
            double servo1Power = servo1PID.calculate(servoEncoder1.getCurrentPosition(), servo1TargetPosition);
            servo1Power = Math.max(-SERVO_POWER, Math.min(SERVO_POWER, servo1Power));
            continuousServo1.setPower(servo1Power);
        }

        if (continuousServo2 != null && servoEncoder2 != null) {
            double servo2Power = servo2PID.calculate(servoEncoder2.getCurrentPosition(), servo2TargetPosition);
            servo2Power = Math.max(-SERVO_POWER, Math.min(SERVO_POWER, servo2Power));
            continuousServo2.setPower(servo2Power);
        }
    }

    public boolean isServo1AtTarget(double toleranceTicks) {
        return servoEncoder1 != null &&
                Math.abs(servoEncoder1.getCurrentPosition() - servo1TargetPosition) <= toleranceTicks;
    }

    public boolean isServo2AtTarget(double toleranceTicks) {
        return servoEncoder2 != null &&
                Math.abs(servoEncoder2.getCurrentPosition() - servo2TargetPosition) <= toleranceTicks;
    }

    // ================= DRIVE SYSTEM METHODS =================

    public void driveMecanum(double y, double x, double rx) {
        if (leftFrontDrive == null) return;

        double heading = getHeadingRadians();

        if (isFieldCentric) {
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            x = rotX;
            y = rotY;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lf = (y + x + rx) / denominator;
        double lb = (y - x + rx) / denominator;
        double rf = (y - x - rx) / denominator;
        double rb = (y + x - rx) / denominator;

        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }

    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }

    public void resetHeadingOffset() {
        headingOffset = getHeadingRadians();
    }

    public boolean isFieldCentricActive() {
        return isFieldCentric;
    }

    private double getHeadingRadians() {
        if (imu == null) return 0;
        try {
            Orientation o = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            return -o.firstAngle + headingOffset;
        } catch (Exception e) {
            return 0;
        }
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeadingRadians());
    }

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        if (leftFrontDrive != null) leftFrontDrive.setZeroPowerBehavior(behavior);
        if (leftBackDrive != null) leftBackDrive.setZeroPowerBehavior(behavior);
        if (rightFrontDrive != null) rightFrontDrive.setZeroPowerBehavior(behavior);
        if (rightBackDrive != null) rightBackDrive.setZeroPowerBehavior(behavior);
    }

    // ================= INTAKE SYSTEM METHODS =================

    public void setIntakePower(double power) {
        if (intake != null) {
            intake.setPower(power);
        }
    }

    public void startIntake() {
        setIntakePower(1.0);
    }

    public void stopIntake() {
        setIntakePower(0.0);
    }

    public void reverseIntake() {
        setIntakePower(-1.0);
    }

    // ================= LAUNCHER SYSTEM METHODS =================

    public void setLauncherRPM(double rpm) {
        launcherRPM = Math.max(0, Math.min(rpm, LAUNCHER_MAX_RPM));
        updateLauncherVelocity();
    }

    public void increaseLauncherRPM() {
        setLauncherRPM(launcherRPM + LAUNCHER_STEP);
    }

    public void decreaseLauncherRPM() {
        setLauncherRPM(launcherRPM - LAUNCHER_STEP);
    }

    public void stopLauncher() {
        setLauncherRPM(0);
    }

    private void updateLauncherVelocity() {
        if (launcherRPM > 0 && launcher != null) {
            double targetTicksPerSec = (launcherRPM / 60.0) * TICKS_PER_REV;
            double currentTicksPerSec = launcher.getVelocity();
            double correction = launcherPID.calculate(currentTicksPerSec, targetTicksPerSec);
            launcher.setVelocity(targetTicksPerSec + correction);
        } else if (launcher != null) {
            launcher.setPower(0);
        }
    }

    public void fireKicker() {
        if (kicker != null) {
            kicker.setPosition(1.0);
        }
    }

    public void retractKicker() {
        if (kicker != null) {
            kicker.setPosition(0.0);
        }
    }

    public double getLauncherRPM() {
        return launcherRPM;
    }

    public double getLauncherActualRPM() {
        if (launcher == null) return 0;
        double ticksPerSec = launcher.getVelocity();
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    // ================= SYSTEM MANAGEMENT =================

    public void stopAllMotors() {
        if (leftFrontDrive != null) leftFrontDrive.setPower(0);
        if (leftBackDrive != null) leftBackDrive.setPower(0);
        if (rightFrontDrive != null) rightFrontDrive.setPower(0);
        if (rightBackDrive != null) rightBackDrive.setPower(0);
        if (intake != null) intake.setPower(0);
        if (spindexer != null) spindexer.setPower(0);
        if (launcher != null) launcher.setPower(0);
        stopContinuousServos();
    }

    public void updateAllSystems() {
        updateSpindexer();
        updateLauncherVelocity();
        updateContinuousServos();
    }

    public void resetAllEncoders() {
        if (spindexer != null) {
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        resetServoEncoders();
        if (hoodEnc != null) {
            hoodEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hoodEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (spindexerEnc != null) {
            spindexerEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexerEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // ================= TELEMETRY METHODS =================

    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading (deg)", "%.1f", getHeadingDegrees());
        telemetry.addData("Spindexer", "Step %d, Pos %d, Moving: %b",
                spindexerStep, getSpindexerPosition(), spindexerMoving);
        telemetry.addData("Spindexer Encoder", "%d ticks", getSpindexerEncoderTicks());
        telemetry.addData("Spindexer Limit", "%s", isSpindexerLimitPressed() ? "PRESSED" : "Released");
        telemetry.addData("Spindexer Use Limit", "%s", useLimitSwitch ? "Enabled" : "Disabled");
        telemetry.addData("Launcher", "Target: %.0f RPM, Actual: %.0f RPM",
                launcherRPM, getLauncherActualRPM());
        telemetry.addData("Hood Position", "%.2f", hoodPos);
        telemetry.addData("Hood Encoder", "%d ticks", getHoodEncoderTicks());
        telemetry.addData("Turret Position", "%.2f", getTurretPosition());
        telemetry.addData("Servo1 Encoder", "%d ticks (%.2f rev)",
                getServo1EncoderTicks(), getServo1PositionRevolutions());
        telemetry.addData("Servo2 Encoder", "%d ticks (%.2f rev)",
                getServo2EncoderTicks(), getServo2PositionRevolutions());
    }
}

// ================= PID CONTROLLER CLASS =================

class PIDController {
    private double kP, kI, kD;
    private double integral = 0;
    private double previousError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer.reset();
    }

    public double calculate(double current, double target) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        integral += error * dt;
        double derivative = (error - previousError) / dt;
        previousError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        timer.reset();
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
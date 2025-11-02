package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Add these imports for color/distance sensor
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DECODEMechanisms {
    // === Hardware Components ===
    private final HardwareMap hardwareMap;

    // === Drive System ===
    private DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private BNO055IMU imu;
    private double headingOffset = 0;
    private boolean isFieldCentric = false;

    // === Intake System ===
    private DcMotor intake;

    // === Spindexer System with Color/Distance Sensor ===
    private DcMotor spindexer;
    private ColorSensor spindexerColorSensor;
    private DistanceSensor spindexerDistanceSensor;
    private DigitalChannel spindexerLimitSwitch = null;
    private int spindexerStep = 0;
    private int ballsLoaded = 0;
    private boolean spindexerMoving = true;
    private boolean homing = false;
    private boolean useLimitSwitch = true;
    private boolean useColorSensor = true;
    private boolean autoIntakeEnabled = true; // Auto-intake enabled by default

    // Spindexer constants
    private static final double SPINDEXER_TICKS_PER_STEP = 377.67;

    // Color sensor thresholds
    private static final double BALL_DETECTION_DISTANCE_MM = 50;
    private static final int COLOR_THRESHOLD = 100;
    private static final int DEBOUNCE_COUNT = 3;
    private int ballDetectionCount = 0;
    private boolean lastBallDetected = false;

    // === Launcher System ===
    private DcMotorEx launcher;
    public Servo kicker;
    private double launcherRPM = 0;
    private boolean isKicking = false;
    private boolean shootSequenceActive = false;
    private static final double LAUNCHER_MAX_RPM = 5000;
    private static final double LAUNCHER_STEP = 250;
    private static final double TICKS_PER_REV = 28;

    // === Hood System with Analog Encoder and PID ===
    private CRServo hood;
    private AnalogInput hoodEnc;
    private double hoodPos = 0.5;
    private double hoodTargetPosition = 0.5;
    private boolean hoodHoldingPosition = false;
    private double hoodManualPower = 0;
    private static final double HOOD_MIN_POS = 0.1;
    private static final double HOOD_MAX_POS = 1.0;
    private static final double HOOD_VOLTS_PER_DEGREE = 3.3 / 360.0;
    private static final double HOOD_ZERO_VOLTAGE = 0.0;
    private static final double HOOD_POSITION_TOLERANCE = 0.02; // Position tolerance for PID
    private static final double HOOD_HOLD_POWER_LIMIT = 0.3; // Max power for holding position

    // === Turret System with Analog Encoder ===
    private CRServo turretSer;
    private AnalogInput turretEnc;

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
    private PIDController hoodPID; // PID for hood position holding
    private PIDController spindexerPID;

    private static final double LAUNCHER_KP = 0.02;
    private static final double LAUNCHER_KI = 0.001;
    private static final double LAUNCHER_KD = 0.005;
    private static final double SERVO_KP = 0.01;
    private static final double SERVO_KI = 0.0005;
    private static final double SERVO_KD = 0.002;
    private static final double HOOD_KP = 0.8;  // Increased for better position holding
    private static final double HOOD_KI = 0.05; // Increased for better position holding
    private static final double HOOD_KD = 0.1;  // Increased for better position holding
    private static final double SPINDEXER_KP = 0.01;
    private static final double SPINDEXER_KI = 0.0001;
    private static final double SPINDEXER_KD = 0.001;

    // === Timing ===
    private final ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime ballCheckTimer = new ElapsedTime();
    private ElapsedTime hoodHoldTimer = new ElapsedTime();

    // Debug flags
    private boolean hoodServoConfigured = false;
    private boolean hoodEncoderConfigured = false;

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
        ballCheckTimer.reset();
        hoodHoldTimer.reset();
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
            spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
            spindexerLimitSwitch = hardwareMap.get(DigitalChannel.class, "limit");

            // Initialize color/distance sensor
            try {
                spindexerColorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
                spindexerDistanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
            } catch (Exception e) {
                // Color sensor not configured
                useColorSensor = false;
            }

            spindexerLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            spindexer.setDirection(DcMotorSimple.Direction.REVERSE);
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            kicker.setPosition(1.0);
        } catch (Exception e) {
            // Launcher system not configured
        }
    }

    private void initializeHoodSystem() {
        try {
            hood = hardwareMap.get(CRServo.class, "hood");
            hoodServoConfigured = true;
        } catch (Exception e) {
            hoodServoConfigured = false;
        }

        try {
            hoodEnc = hardwareMap.get(AnalogInput.class, "hoodEnc");
            hoodEncoderConfigured = true;
        } catch (Exception e) {
            hoodEncoderConfigured = false;
        }

        if (hood != null) {
            hood.setPower(0);
        }

        // Initialize hood position from encoder if available
        if (hoodEncoderConfigured && hoodServoConfigured) {
            hoodPos = getHoodEncoderPosition();
            hoodTargetPosition = hoodPos;
        }
    }

    private void initializeTurretSystem() {
        try {
            turretSer = hardwareMap.get(CRServo.class, "turretSer");
            if (turretSer != null) {
                turretSer.setPower(0);
            }
        } catch (Exception e) {
            // Turret system not configured
        }

        try {
            turretEnc = hardwareMap.get(AnalogInput.class, "turretEnc");
        } catch (Exception e) {
            // Turret encoder not configured
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
        hoodPID = new PIDController(HOOD_KP, HOOD_KI, HOOD_KD); // Initialize hood PID
        spindexerPID = new PIDController(SPINDEXER_KP, SPINDEXER_KI, SPINDEXER_KD);
    }

    // ================= SERVO TEST METHODS =================

    public void testAllServos() {
        // Test hood servo
        if (hood != null) {
            moveHoodManual(0.7); // Test with more power
            // Stop after a short delay
            new Thread(() -> {
                try {
                    Thread.sleep(1000);
                    stopHood();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }

        // Test turret servo
        if (turretSer != null) {
            moveTurretManual(0.7); // Test with more power
            // Stop after a short delay
            new Thread(() -> {
                try {
                    Thread.sleep(1000);
                    stopTurret();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }

        // Test kicker servo
        if (kicker != null) {
            double currentKickerPos = kicker.getPosition();
            double testKickerPos = (currentKickerPos > 0.5) ? 0.3 : 0.7;
            kicker.setPosition(testKickerPos);
        }

        // Test continuous servos
        if (continuousServo1 != null) {
            setContinuousServo1Power(0.5);
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    setContinuousServo1Power(0);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }

        if (continuousServo2 != null) {
            setContinuousServo2Power(0.5);
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    setContinuousServo2Power(0);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
    }

    public void setHoodTestPosition(double position) {
        // For testing, use direct power based on position
        // Convert position (0.0-1.0) to power (-1.0 to 1.0)
        double power = (position - 0.5) * 2.0; // More responsive
        moveHoodManual(power);

        // Set target position for PID holding
        hoodTargetPosition = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
        hoodHoldingPosition = true;
    }

    public void setTurretTestPosition(double position) {
        // For testing, use direct power based on position
        double power = (position - 0.5) * 2.0; // More responsive
        moveTurretManual(power);
    }

    // ================= COLOR SENSOR METHODS =================

    public boolean isBallDetected() {
        if (!useColorSensor || spindexerDistanceSensor == null) {
            return false;
        }

        double distance = spindexerDistanceSensor.getDistance(DistanceUnit.MM);
        int red = spindexerColorSensor.red();
        int blue = spindexerColorSensor.blue();
        int green = spindexerColorSensor.green();

        // Check if something is close enough and has significant color
        boolean ballDetected = distance < BALL_DETECTION_DISTANCE_MM &&
                (red > COLOR_THRESHOLD || blue > COLOR_THRESHOLD || green > COLOR_THRESHOLD);

        // Debouncing
        if (ballDetected != lastBallDetected) {
            ballDetectionCount++;
            if (ballDetectionCount >= DEBOUNCE_COUNT) {
                lastBallDetected = ballDetected;
                ballDetectionCount = 0;
            }
        } else {
            ballDetectionCount = 0;
        }

        return lastBallDetected;
    }

    public double getBallDistance() {
        if (spindexerDistanceSensor != null) {
            return spindexerDistanceSensor.getDistance(DistanceUnit.MM);
        }
        return 999;
    }

    public int getColorRed() {
        return spindexerColorSensor != null ? spindexerColorSensor.red() : 0;
    }

    public int getColorBlue() {
        return spindexerColorSensor != null ? spindexerColorSensor.blue() : 0;
    }

    public int getColorGreen() {
        return spindexerColorSensor != null ? spindexerColorSensor.green() : 0;
    }

    public void setUseColorSensor(boolean useColorSensor) {
        this.useColorSensor = useColorSensor;
    }

    public boolean isUseColorSensor() {
        return useColorSensor;
    }

    public void setAutoIntakeEnabled(boolean enabled) {
        this.autoIntakeEnabled = enabled;
    }

    public boolean isAutoIntakeEnabled() {
        return autoIntakeEnabled;
    }

    // ================= IMPROVED SPINDEXER METHODS =================

    public boolean isSpindexerLimitPressed() {
        if (spindexerLimitSwitch != null) {
            return !spindexerLimitSwitch.getState();
        }
        return false;
    }

    public void homeSpindexer() {
        if (spindexer == null) return;

//        spindexerMoving = true;
        homing = true;

        ElapsedTime timeout = new ElapsedTime();

        while (!isSpindexerLimitPressed() && timeout.seconds() < 5.0 && !Thread.currentThread().isInterrupted()) {
            spindexer.setPower(0.4);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        spindexer.setPower(0);
//        spindexerMoving = false;
        homing = false;

        if (spindexer != null) {
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        spindexerStep = 0;
//        ballsLoaded = 0;
    }

    public void autoRotateToNextSlot() {
        if (spindexerMoving || spindexer == null) return;

//        spindexerMoving = true;
        int targetStep = (spindexerStep + 1) % 3;

        if (targetStep == 0) {
            homeSpindexer();
        } else {
            moveSpindexerToStep(targetStep);
        }
    }

    private void moveSpindexerToStep(int step) {
        if (spindexer == null) return;

//        spindexerMoving = true;

        if (step == 0) {
            homeSpindexer();
        } else {
            int targetPos = (int)(step * SPINDEXER_TICKS_PER_STEP);

            spindexer.setTargetPosition(targetPos);
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(0.5);

            ElapsedTime movementTimer = new ElapsedTime();
            while (spindexer.isBusy() && movementTimer.seconds() < 3.0 && !Thread.currentThread().isInterrupted()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            spindexer.setPower(0);
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            spindexerStep = step;
//            spindexerMoving = false;
        }
    }

    public void intakeBallSequence() {
        if (!autoIntakeEnabled || spindexerMoving) return;

        if (ballsLoaded >= 3) {
            if (!spindexerMoving) {
                homeSpindexer();
            }
            return;
        }

        // Check for ball detection every 200ms to avoid excessive checking
        if (ballCheckTimer.seconds() > .2 && !spindexerMoving && ballsLoaded < 3) {
            if (isBallDetected() && !spindexerMoving && !isKicking && !shootSequenceActive) {
                ballsLoaded++;
                ballCheckTimer.reset();
                autoRotateToNextSlot();

                // Move to next slot after brief delay
//                new Thread(() -> {
//                    try {
//                        Thread.sleep(100);
//                        if (!Thread.currentThread().isInterrupted()) {
//                            autoRotateToNextSlot();
//                        }
//                    } catch (InterruptedException e) {
//                        Thread.currentThread().interrupt();
//                    }
//                }).start();
            }
            ballCheckTimer.reset();
        }
    }

    public void advanceSpindexer() {
        if (!spindexerMoving && spindexer != null) {
            if (ballsLoaded >= 3) {
                homeSpindexer();
                ballsLoaded = 0;
            } else {
                autoRotateToNextSlot();
                ballsLoaded = Math.min(3, ballsLoaded + 1);
            }
        }
    }

    public void setSpindexerStep(int step) {
        if (step >= 0 && step < 3 && !spindexerMoving && spindexer != null) {
            spindexerStep = step;
            moveSpindexerToStep(step);

            if (step == 0) {
                ballsLoaded = 0;
            }
        }
    }

    public void manualAdvanceSpindexer() {
        if (!spindexerMoving && spindexer != null) {
            int nextStep = (spindexerStep + 1) % 3;
            setSpindexerStep(nextStep);
        }
    }

    public int getBallsLoaded() {
        return ballsLoaded;
    }

    public void setBallsLoaded(int count) {
        ballsLoaded = Math.max(0, Math.min(3, count));
    }

    public void setUseLimitSwitch(boolean useLimitSwitch) {
        this.useLimitSwitch = useLimitSwitch;
    }

    public boolean isUseLimitSwitch() {
        return useLimitSwitch;
    }

    public void stopSpindexer() {
//        spindexerMoving = false;
        if (spindexer != null) {
            spindexer.setPower(0);
        }
    }

    public void updateSpindexer() {
        spindexerMoving = spindexer.getPower() > .1 || spindexer.getPower() < -.1;

        if (spindexerMoving && spindexer != null) {
            if (useLimitSwitch && isSpindexerLimitPressed() && !homing) {
                spindexer.setPower(0);
//                spindexerMoving = false;
            }

            if (!spindexer.isBusy() && !homing) {
                spindexer.setPower(0);
                spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                spindexerMoving = false;
            }
        }

        // Auto ball detection
        if (autoIntakeEnabled && useColorSensor && !spindexerMoving) {
            intakeBallSequence();
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
        return spindexer != null ? spindexer.getCurrentPosition() : 0;
    }

    public double getSpindexerEncoderRevolutions() {
        return spindexer != null ? spindexer.getCurrentPosition() / SPINDEXER_TICKS_PER_STEP : 0;
    }

    // ================= HOOD SYSTEM METHODS WITH PID HOLDING =================

    /**
     * Converts encoder voltage to position (0.0 to 1.0)
     */
    private double getHoodEncoderPosition() {
        if (hoodEnc == null) return hoodPos;
        double voltage = hoodEnc.getVoltage();
        // Convert voltage to position (assuming 0-3.3V range)
        return Math.max(0.0, Math.min(1.0, voltage / 3.3));
    }

    /**
     * Sets hood power with PID holding when manual control stops
     */
    public void setHoodPower(double power) {
        if (hood != null) {
            double deadzone = 0.1;

            if (Math.abs(power) < deadzone) {
                // No manual input - enable position holding
                if (!hoodHoldingPosition) {
                    // Start holding current position
                    hoodTargetPosition = getHoodEncoderPosition();
                    hoodHoldingPosition = true;
                    hoodPID.reset(); // Reset PID for fresh start
                }

                // Use PID to maintain position
                double currentPos = getHoodEncoderPosition();
                double pidPower = hoodPID.calculate(currentPos, hoodTargetPosition);

                // Limit holding power to prevent excessive force
                pidPower = Math.max(-HOOD_HOLD_POWER_LIMIT, Math.min(HOOD_HOLD_POWER_LIMIT, pidPower));

                // Only apply power if we're not at target position (avoid jitter)
                if (Math.abs(currentPos - hoodTargetPosition) > HOOD_POSITION_TOLERANCE) {
                    hood.setPower(pidPower);
                } else {
                    hood.setPower(0);
                }

                hoodManualPower = 0;
            } else {
                // Manual input - disable position holding and use direct power
                hoodHoldingPosition = false;
                hoodManualPower = power;

                // Apply power with scaling for better control
                double scaledPower = Math.signum(power) * (0.3 + 0.7 * (Math.abs(power) - deadzone) / (1 - deadzone));
                double limitedPower = Math.max(-1.0, Math.min(1.0, scaledPower));
                hood.setPower(limitedPower);

                // Update target position to current position for when we release
                hoodTargetPosition = getHoodEncoderPosition();
            }
        }
    }

    /**
     * Sets hood to a specific position using PID control
     */
    public void setHoodPosition(double position) {
        if (hood != null && hoodEnc != null) {
            hoodTargetPosition = Math.max(HOOD_MIN_POS, Math.min(HOOD_MAX_POS, position));
            hoodPos = hoodTargetPosition;
            hoodHoldingPosition = true;
            hoodPID.reset(); // Reset PID for fresh position target
        }
    }

    /**
     * Manual hood control - sets power directly and updates target position
     */
    public void moveHoodManual(double power) {
        setHoodPower(power);
    }

    public void adjustHoodPosition(double delta) {
        setHoodPosition(hoodPos + delta);
    }

    public void stopHood() {
        setHoodPower(0);
    }

    public double getHoodPosition() {
        return getHoodEncoderPosition();
    }

    public double getHoodEncoderVoltage() {
        return hoodEnc != null ? hoodEnc.getVoltage() : 0;
    }

    public double getHoodEncoderAngle() {
        if (hoodEnc == null) return 0;
        return (hoodEnc.getVoltage() - HOOD_ZERO_VOLTAGE) / HOOD_VOLTS_PER_DEGREE;
    }

    public boolean isHoodAtPosition(double tolerance) {
        return Math.abs(getHoodEncoderPosition() - hoodTargetPosition) <= tolerance;
    }

    public boolean isHoodHoldingPosition() {
        return hoodHoldingPosition;
    }

    public double getHoodTargetPosition() {
        return hoodTargetPosition;
    }

    // ================= TURRET SYSTEM METHODS FOR CRServo =================

    public void setTurretPower(double power) {
        if (turretSer != null) {
            double deadzone = 0.1;
            if (Math.abs(power) < deadzone) {
                power = 0;
            } else {
                power = Math.signum(power) * (0.4 + 0.6 * (Math.abs(power) - deadzone) / (1 - deadzone));
            }

            double limitedPower = Math.max(-1.0, Math.min(1.0, power));
            turretSer.setPower(limitedPower);
        }
    }

    public void moveTurretManual(double power) {
        setTurretPower(power);
    }

    public void adjustTurretPosition(double delta) {
        if (turretSer != null) {
            double currentPower = 0;
            setTurretPower(currentPower + (delta * 2.0));
        }
    }

    public void stopTurret() {
        setTurretPower(0);
    }

    public double getTurretPosition() {
        return 0.5;
    }

    public double getTurretEncoderVoltage() {
        return turretEnc != null ? turretEnc.getVoltage() : 0;
    }

    public double getTurretEncoderAngle() {
        if (turretEnc == null) return 0;
        return (turretEnc.getVoltage() - HOOD_ZERO_VOLTAGE) / HOOD_VOLTS_PER_DEGREE;
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
            return o.firstAngle - headingOffset;
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
        setIntakePower(-1.0);
    }

    public void stopIntake() {
        setIntakePower(0.4);
    }

    public void reverseIntake() {
        setIntakePower(1.0);
        shootSequenceActive = false;
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
            isKicking = true;
            shootSequenceActive = true;
            ballsLoaded--;
            kicker.setPosition(0.67);
            if (ballsLoaded < 0) ballsLoaded = 0;
        }
    }

    public void retractKicker() {
        if (kicker != null) {
            kicker.setPosition(1.0);
            isKicking = false;
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
        stopHood();
        stopTurret();
        stopContinuousServos();
    }

    public void updateAllSystems() {
        updateSpindexer();
        updateLauncherVelocity();
        updateContinuousServos();
        // Hood PID is handled in setHoodPower automatically
    }

    public void resetAllEncoders() {
        if (spindexer != null) {
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        resetServoEncoders();
        if (spindexer != null) {
            spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // ================= TELEMETRY METHODS =================

    public void addTelemetryData(Telemetry telemetry) {
        telemetry.addData("iskicking", isKicking);
        telemetry.addData("shootingSequence", shootSequenceActive);
        telemetry.addData("Drive Mode", isFieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading (deg)", "%.1f", getHeadingDegrees());
        telemetry.addData("Spindexer", "Step %d, Balls: %d/3, Moving: %b",
                spindexerStep, ballsLoaded, spindexerMoving);
        telemetry.addData("Spindexer Encoder", "%d ticks", getSpindexerEncoderTicks());
        telemetry.addData("Spindexer Limit", "%s", isSpindexerLimitPressed() ? "PRESSED" : "Released");
        telemetry.addData("Spindexer Use Limit", "%s", useLimitSwitch ? "Enabled" : "Disabled");
        telemetry.addData("Auto-Intake", "%s", autoIntakeEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Color Sensor", useColorSensor ? "ENABLED" : "DISABLED");
        if (useColorSensor) {
            telemetry.addData("Ball Detected", isBallDetected() ? "YES" : "NO");
            telemetry.addData("Distance", "%.1f mm", getBallDistance());
            telemetry.addData("Color", "R:%d G:%d B:%d", getColorRed(), getColorGreen(), getColorBlue());
        }
        telemetry.addData("Launcher", "Target: %.0f RPM, Actual: %.0f RPM",
                launcherRPM, getLauncherActualRPM());
        telemetry.addData("Hood Position", "%.3f", getHoodEncoderPosition());
        telemetry.addData("Hood Target", "%.3f", hoodTargetPosition);
        telemetry.addData("Hood Holding", hoodHoldingPosition ? "ACTIVE" : "INACTIVE");
        telemetry.addData("Hood Manual Power", "%.2f", hoodManualPower);
        telemetry.addData("Hood Encoder", "%.3fV (%.1f°)", getHoodEncoderVoltage(), getHoodEncoderAngle());
        telemetry.addData("Turret Position", "%.2f", getTurretPosition());
        telemetry.addData("Turret Encoder", "%.3fV (%.1f°)", getTurretEncoderVoltage(), getTurretEncoderAngle());
        telemetry.addData("Servo1 Encoder", "%d ticks (%.2f rev)",
                getServo1EncoderTicks(), getServo1PositionRevolutions());
        telemetry.addData("Servo2 Encoder", "%d ticks (%.2f rev)",
                getServo2EncoderTicks(), getServo2PositionRevolutions());
    }

    // ================= DEBUG METHODS =================

    public void debugHoodSystem(Telemetry telemetry) {
        telemetry.addData("Hood Servo", hood != null ? "CONFIGURED (CRServo)" : "NULL");
        telemetry.addData("Hood Encoder", hoodEnc != null ? "CONFIGURED" : "NULL");
        if (hood != null) {
            telemetry.addData("Hood Target Pos", "%.3f", hoodTargetPosition);
            telemetry.addData("Hood Actual Pos", "%.3f", getHoodEncoderPosition());
            telemetry.addData("Hood Error", "%.3f", Math.abs(getHoodEncoderPosition() - hoodTargetPosition));
        }
        if (hoodEnc != null) {
            telemetry.addData("Hood Encoder Voltage", "%.3fV", hoodEnc.getVoltage());
            telemetry.addData("Hood Encoder Angle", "%.1f°", getHoodEncoderAngle());
        }
        telemetry.addData("Hood Configured", hoodServoConfigured ? "YES" : "NO");
        telemetry.addData("Hood Encoder Configured", hoodEncoderConfigured ? "YES" : "NO");
        telemetry.addData("Hood PID Holding", hoodHoldingPosition ? "ACTIVE" : "INACTIVE");

        telemetry.addData("Turret Servo", turretSer != null ? "CONFIGURED (CRServo)" : "NULL");
        telemetry.addData("Turret Encoder", turretEnc != null ? "CONFIGURED" : "NULL");
        if (turretEnc != null) {
            telemetry.addData("Turret Encoder Voltage", "%.3fV", turretEnc.getVoltage());
            telemetry.addData("Turret Encoder Angle", "%.1f°", getTurretEncoderAngle());
        }
    }

    public void debugServoSystem(Telemetry telemetry) {
        telemetry.addData("Hood Servo", hood != null ? "CONFIGURED (CRServo)" : "NULL");
        telemetry.addData("Turret Servo", turretSer != null ? "CONFIGURED (CRServo)" : "NULL");
        telemetry.addData("Kicker Servo", kicker != null ? "CONFIGURED (Servo)" : "NULL");

        if (hood != null) {
            telemetry.addData("Hood Target Pos", "%.3f", hoodTargetPosition);
            telemetry.addData("Hood Actual Pos", "%.3f", getHoodEncoderPosition());
        }

        if (turretSer != null) {
            telemetry.addData("Turret Status", "CRServo - use power control");
        }
    }
}

class PIDController {
    private double kP, kI, kD;
    private double integral = 0;
    private double previousError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double maxIntegral = 1000;

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

        if (dt <= 0) {
            dt = 0.01;
        }

        integral += error * dt;
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

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
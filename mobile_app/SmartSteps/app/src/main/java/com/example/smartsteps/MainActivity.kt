package com.example.smartsteps

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.os.Build
import android.os.Bundle
import android.os.VibrationEffect
import android.os.Vibrator
import android.widget.*
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.isVisible
import kotlin.math.roundToInt

class MainActivity : AppCompatActivity(), BleManager.Listener {

    private lateinit var bleManager: BleManager

    // ===== UI ELEMENTS =====
    private lateinit var connectButton: Button
    private lateinit var stepsText: TextView
    private lateinit var distanceText: TextView
    private lateinit var caloriesText: TextView
    private lateinit var activeTimeText: TextView
    private lateinit var batteryText: TextView
    private lateinit var batteryIcon: ImageView
    private lateinit var goalInput: EditText
    private lateinit var sensitivitySeek: SeekBar
    private lateinit var achievementBadge: ImageView
    private lateinit var goalRingView: GoalRingView
    private lateinit var applyBtn: Button
    private lateinit var resetBtn: Button

    // ===== Pace calculation state =====
    private var lastSteps = 0
    private var lastTime = 0L

    companion object {
        private const val REQ_BLE_PERMISSIONS = 1001
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // ===== Bind UI =====
        connectButton = findViewById(R.id.connectButton)
        stepsText = findViewById(R.id.stepsText)
        distanceText = findViewById(R.id.distanceText)
        caloriesText = findViewById(R.id.caloriesText)
        activeTimeText = findViewById(R.id.activeTimeText)
        batteryText = findViewById(R.id.batteryText)
        batteryIcon = findViewById(R.id.batteryIcon)
        goalInput = findViewById(R.id.goalInput)
        sensitivitySeek = findViewById(R.id.sensitivitySeek)
        achievementBadge = findViewById(R.id.achievementBadge)
        goalRingView = findViewById(R.id.goalRingView)
        applyBtn = findViewById(R.id.applyBtn)
        resetBtn = findViewById(R.id.resetBtn)

        achievementBadge.isVisible = false

        // ===== BLE Manager =====
        bleManager = BleManager(this, this)

        // ===== Connect Button =====
        connectButton.setOnClickListener {
            startScanAndConnect()
        }

        // ===== APPLY SETTINGS BUTTON =====
        applyBtn.setOnClickListener {
            val goal = goalInput.text.toString().toIntOrNull()
            if (goal != null && goal > 0) {
                bleManager.sendCommand("goal:$goal")
                Toast.makeText(this, "Goal updated", Toast.LENGTH_SHORT).show()
            }

            val sens = valueToSensitivity(sensitivitySeek.progress)
            bleManager.sendCommand("sens:$sens")
            Toast.makeText(this, "Sensitivity updated", Toast.LENGTH_SHORT).show()
        }

        // ===== RESET BUTTON =====
        resetBtn.setOnClickListener {
            bleManager.sendCommand("reset")

            // Reset UI immediately
            stepsText.text = "0 steps"
            caloriesText.text = "Calories: 0.0"
            distanceText.text = "Distance: 0.00 km"
            activeTimeText.text = "Pace: 0 spm"
            achievementBadge.isVisible = false
            goalRingView.setProgress(0f)

            Toast.makeText(this, "Reset done", Toast.LENGTH_SHORT).show()
        }


        // ===== Sensitivity slider =====
        sensitivitySeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar?, value: Int, fromUser: Boolean) {}
            override fun onStartTrackingTouch(sb: SeekBar?) {}
            override fun onStopTrackingTouch(sb: SeekBar?) {
                val sens = valueToSensitivity(sensitivitySeek.progress)
                bleManager.sendCommand("sens:$sens")
            }
        })

        // Optional: send goal when user presses Done
        goalInput.setOnEditorActionListener { v, _, _ ->
            val text = v.text.toString()
            val goal = text.toIntOrNull()
            if (goal != null && goal > 0) {
                bleManager.sendCommand("goal:$goal")
                Toast.makeText(this, "Goal sent: $goal", Toast.LENGTH_SHORT).show()
            }
            false
        }
    }

    // Convert SeekBar 0–10 → sensitivity 1.0–2.0
    private fun valueToSensitivity(v: Int): Float {
        return 0.5f + (v / 40f)   // 0 → 0.5, 100 → 3.0
    }


    // ===== BLE PERMISSIONS =====
    private fun hasBlePermissions(): Boolean {
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.S) return true

        val scan = checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN)
        val connect = checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT)
        return scan == android.content.pm.PackageManager.PERMISSION_GRANTED &&
                connect == android.content.pm.PackageManager.PERMISSION_GRANTED
    }

    private fun requestBlePermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            requestPermissions(
                arrayOf(
                    Manifest.permission.BLUETOOTH_SCAN,
                    Manifest.permission.BLUETOOTH_CONNECT
                ),
                REQ_BLE_PERMISSIONS
            )
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == REQ_BLE_PERMISSIONS) {
            startScanAndConnect()
        }
    }

    // ===== BLE Scan =====
    private fun startScanAndConnect() {
        if (!hasBlePermissions()) {
            requestBlePermissions()
            return
        }

        val adapter = BluetoothAdapter.getDefaultAdapter()
        if (adapter == null) {
            Toast.makeText(this, "Bluetooth not available", Toast.LENGTH_LONG).show()
            return
        }

        if (!adapter.isEnabled) {
            Toast.makeText(this, "Turn on Bluetooth", Toast.LENGTH_LONG).show()
            return
        }

        val device: BluetoothDevice? = adapter.bondedDevices.find {
            it.name == "SmartSteps BLE"
        }

        if (device != null) {
            connectButton.text = "Connecting..."
            bleManager.connect(device)
        } else {
            Toast.makeText(this, "SmartSteps BLE not paired", Toast.LENGTH_LONG).show()
        }
    }

    // ===== BLE CALLBACKS =====
    override fun onConnected() {
        runOnUiThread {
            connectButton.text = "Connected ✓"
        }
    }

    override fun onDisconnected() {
        runOnUiThread {
            connectButton.text = "Connect to SmartSteps"
            achievementBadge.isVisible = false
            batteryText.text = "Battery: --%"
        }
    }

    override fun onDataReceived(
        steps: Int,
        goal: Int,
        battery: Int,
        reached: Boolean
    ) {
        runOnUiThread {
            stepsText.text = "$steps steps"

            val calories = steps * 0.04f
            caloriesText.text = "Calories: ${"%.1f".format(calories)}"

            val distance = steps * 0.00078f
            distanceText.text = "Distance: ${"%.2f".format(distance)} km"

            val pace = calculatePace(steps)
            activeTimeText.text = "Pace: $pace spm"

            batteryText.text = "Battery: $battery%"
            updateBatteryColor(battery)

            achievementBadge.isVisible = reached

            if (reached) {
                celebrateGoal()
            }

            val percent = if (goal > 0) {
                (steps.toFloat() / goal.toFloat()).coerceIn(0f, 1f)
            } else 0f

            goalRingView.setProgress(percent)
        }
    }

    private fun celebrateGoal() {
        Toast.makeText(this, "🎉 Goal Achieved!", Toast.LENGTH_SHORT).show()

        val vibrator = getSystemService(VIBRATOR_SERVICE) as Vibrator

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            // API 26+
            vibrator.vibrate(
                VibrationEffect.createOneShot(
                    200,
                    VibrationEffect.DEFAULT_AMPLITUDE
                )
            )
        } else {
            // API 24–25 fallback
            vibrator.vibrate(200)
        }
    }


    private fun calculatePace(steps: Int): Int {
        val now = System.currentTimeMillis()

        if (lastTime == 0L) {
            lastTime = now
            lastSteps = steps
            return 0
        }

        val dt = (now - lastTime) / 1000f
        val ds = steps - lastSteps

        lastSteps = steps
        lastTime = now

        return if (dt > 0) ((ds / dt) * 60).roundToInt() else 0
    }

    private fun updateBatteryColor(percent: Int) {
        when {
            percent <= 20 -> batteryText.setTextColor(0xFFFF4444.toInt())
            percent <= 50 -> batteryText.setTextColor(0xFFFFBB33.toInt())
            else -> batteryText.setTextColor(0xFF33CC33.toInt())
        }
    }
}

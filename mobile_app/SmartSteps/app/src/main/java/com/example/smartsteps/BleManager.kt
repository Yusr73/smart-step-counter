package com.example.smartsteps

import android.annotation.SuppressLint
import android.bluetooth.*
import android.content.Context
import java.util.UUID

class BleManager(
    private val context: Context,
    private val listener: Listener
) {

    interface Listener {
        fun onConnected()
        fun onDisconnected()
        fun onDataReceived(
            steps: Int,
            goal: Int,
            battery: Int,
            reached: Boolean
        )
    }

    private var bluetoothGatt: BluetoothGatt? = null

    private val SERVICE_UUID = UUID.fromString("0000abcd-0000-1000-8000-00805f9b34fb")
    private val STEPS_UUID   = UUID.fromString("0001abcd-0000-1000-8000-00805f9b34fb")
    private val GOAL_UUID    = UUID.fromString("0002abcd-0000-1000-8000-00805f9b34fb")
    private val SENS_UUID    = UUID.fromString("0003abcd-0000-1000-8000-00805f9b34fb")
    private val CMD_UUID     = UUID.fromString("0004abcd-0000-1000-8000-00805f9b34fb")

    private var stepsChar: BluetoothGattCharacteristic? = null
    private var goalChar:  BluetoothGattCharacteristic? = null
    private var sensChar:  BluetoothGattCharacteristic? = null
    private var cmdChar:   BluetoothGattCharacteristic? = null

    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        bluetoothGatt = device.connectGatt(context, false, gattCallback)
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        bluetoothGatt?.close()
        bluetoothGatt = null
        listener.onDisconnected()
    }

    @SuppressLint("MissingPermission")
    fun sendCommand(cmd: String) {
        val char = cmdChar ?: return
        val gatt = bluetoothGatt ?: return

        char.value = cmd.toByteArray()

        val supportsWriteNoResp =
            char.properties and BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE != 0

        char.writeType =
            if (supportsWriteNoResp)
                BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
            else
                BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT

        gatt.writeCharacteristic(char)
    }

    @SuppressLint("MissingPermission")
    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                listener.onConnected()
                gatt.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                listener.onDisconnected()
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) return

            println("DEBUG: onServicesDiscovered")

            val service = gatt.getService(SERVICE_UUID)
            println("DEBUG: service = $service")

            if (service == null) {
                println("DEBUG: Service NOT found")
                return
            }

            stepsChar = service.getCharacteristic(STEPS_UUID)
            goalChar  = service.getCharacteristic(GOAL_UUID)
            sensChar  = service.getCharacteristic(SENS_UUID)
            cmdChar   = service.getCharacteristic(CMD_UUID)

            val steps = stepsChar ?: return

            val ok = gatt.setCharacteristicNotification(steps, true)
            println("DEBUG: setCharacteristicNotification = $ok")

            val cccd = steps.getDescriptor(
                UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
            )
            println("DEBUG: descriptor = $cccd")

            if (cccd == null) {
                println("DEBUG: CCCD descriptor is NULL")
                return
            }

            cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            val writeOk = gatt.writeDescriptor(cccd)
            println("DEBUG: writeDescriptor = $writeOk")

            gatt.requestMtu(128)
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            println("DEBUG: MTU changed to $mtu")
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            if (characteristic.uuid != STEPS_UUID) return

            val raw = characteristic.value ?: return
            if (raw.isEmpty()) return

            val text = raw.toString(Charsets.UTF_8)
            println("DEBUG: onCharacteristicChanged → $text")

            try {
                // Incoming format: {s:7,g:5,b:0,r:1}
                val clean = text.removePrefix("{").removeSuffix("}")
                val parts = clean.split(",")

                var steps = 0
                var goal = 0
                var battery = 0
                var reached = false

                for (p in parts) {
                    val kv = p.split(":")
                    if (kv.size == 2) {
                        val key = kv[0]
                        val value = kv[1]

                        when (key) {
                            "s" -> steps = value.toIntOrNull() ?: 0
                            "g" -> goal = value.toIntOrNull() ?: 0
                            "b" -> battery = (value.toIntOrNull() ?: 0) * 10
                            "r" -> reached = (value == "1")
                        }
                    }
                }

                listener.onDataReceived(steps, goal, battery, reached)

            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
    }
}

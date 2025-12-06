package com.example.embedded_term_application

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.Message
import android.util.Log
import android.view.View
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.ScrollView
import android.widget.TextView
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import java.io.IOException
import java.io.InputStream
import java.text.SimpleDateFormat
import java.util.*
import kotlin.collections.ArrayList

class MainActivity : AppCompatActivity() {

    companion object {
        val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        const val MESSAGE_READ = 0
    }

    private lateinit var btnBluetoothConnect: Button
    private lateinit var tvStatusText: TextView

    // 센서 UI
    private lateinit var tvValTemp: TextView
    private lateinit var tvValPressure: TextView
    private lateinit var tvValFlame: TextView

    // 로그 UI
    private lateinit var tvLogResult: TextView
    private lateinit var svLogContainer: ScrollView

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var connectThread: ConnectThread? = null
    private var connectedThread: ConnectedThread? = null

    private val scanResultList = ArrayList<BluetoothDevice>()
    private val scanResultStrings = ArrayList<String>()
    private lateinit var listAdapter: ArrayAdapter<String>
    private var scanDialog: AlertDialog? = null

    // 로그 및 상태 감지용 변수
    private var prevPressure = -1
    private var prevFlame = -1

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.values.all { it }) {
            startDiscovery()
        } else {
            addLog("⚠권한이 거부되어 장치를 찾을 수 없습니다.")
        }
    }

    private val receiver = object : BroadcastReceiver() {
        @SuppressLint("MissingPermission")
        override fun onReceive(context: Context, intent: Intent) {
            val action: String? = intent.action
            if (BluetoothDevice.ACTION_FOUND == action) {
                val device: BluetoothDevice? = if (Build.VERSION.SDK_INT >= 33) {
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
                } else {
                    @Suppress("DEPRECATION")
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                }

                device?.let {
                    val address = it.address
                    if (scanResultList.none { d -> d.address == address }) {
                        scanResultList.add(it)
                        val name = it.name ?: "Unknown"
                        scanResultStrings.add("$name\n$address")
                        listAdapter.notifyDataSetChanged()
                    }
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter

        initViews()
        setUiDisconnectedState()

        btnBluetoothConnect.setOnClickListener {
            if (connectedThread != null) {
                stopConnection()
            } else {
                checkPermissionsAndScan()
            }
        }

        val filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
        registerReceiver(receiver, filter)

        addLog("앱이 시작되었습니다. (대기 중)")
    }

    override fun onDestroy() {
        super.onDestroy()
        stopConnection()
        try { unregisterReceiver(receiver) } catch (e: IllegalArgumentException) {}
    }

    private fun initViews() {
        btnBluetoothConnect = findViewById(R.id.btnBluetoothConnect)
        tvStatusText = findViewById(R.id.tvStatusText)
        listAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, scanResultStrings)

        tvLogResult = findViewById(R.id.tvLogResult)
        svLogContainer = findViewById(R.id.svLogContainer)

        val cardTemp = findViewById<View>(R.id.cardTemp)
        cardTemp.findViewById<TextView>(R.id.tvSensorLabel).text = "Temperature"
        tvValTemp = cardTemp.findViewById(R.id.tvSensorValue)

        val cardPressure = findViewById<View>(R.id.cardPressure)
        cardPressure.findViewById<TextView>(R.id.tvSensorLabel).text = "Pressure"
        tvValPressure = cardPressure.findViewById(R.id.tvSensorValue)

        val cardFlame = findViewById<View>(R.id.cardFlame)
        cardFlame.findViewById<TextView>(R.id.tvSensorLabel).text = "Flame"
        tvValFlame = cardFlame.findViewById(R.id.tvSensorValue)
    }

    private fun setUiDisconnectedState() {
        runOnUiThread {
            val grayColor = Color.GRAY
            tvValTemp.text = "-"
            tvValTemp.setTextColor(grayColor)
            tvValPressure.text = "-"
            tvValPressure.setTextColor(grayColor)
            tvValFlame.text = "-"
            tvValFlame.setTextColor(grayColor)

            tvStatusText.text = "DISCONNECTED"
            tvStatusText.setTextColor(Color.GRAY)
            btnBluetoothConnect.text = "장치 찾기"
        }
    }

    private fun setUiConnectedState() {
        runOnUiThread {
            val noneColor = Color.RED
            val noneText = "NONE"

            tvValTemp.text = noneText
            tvValTemp.setTextColor(noneColor)

            tvValPressure.text = noneText
            tvValPressure.setTextColor(noneColor)

            tvValFlame.text = noneText
            tvValFlame.setTextColor(noneColor)

            tvStatusText.text = "CONNECTED"
            tvStatusText.setTextColor(Color.GREEN)
            btnBluetoothConnect.text = "연결 해제"
        }
    }

    // 로그를 UI에 출력하는 함수 (Toast 대체)
    private fun addLog(message: String) {
        runOnUiThread {
            val currentTime = SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(Date())
            val logMessage = "[$currentTime] $message\n"
            tvLogResult.append(logMessage)
            // 자동 스크롤
            svLogContainer.post {
                svLogContainer.fullScroll(View.FOCUS_DOWN)
            }
        }
    }

    private fun checkPermissionsAndScan() {
        val permissions = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            permissions.add(Manifest.permission.BLUETOOTH)
            permissions.add(Manifest.permission.BLUETOOTH_ADMIN)
            permissions.add(Manifest.permission.ACCESS_FINE_LOCATION)
        }

        val notGranted = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (notGranted.isNotEmpty()) {
            requestPermissionLauncher.launch(notGranted.toTypedArray())
        } else {
            startDiscovery()
        }
    }

    @SuppressLint("MissingPermission")
    private fun startDiscovery() {
        scanResultList.clear()
        scanResultStrings.clear()

        val pairedDevices: Set<BluetoothDevice>? = bluetoothAdapter?.bondedDevices
        if (!pairedDevices.isNullOrEmpty()) {
            for (device in pairedDevices) {
                scanResultList.add(device)
                val name = device.name ?: "Unknown"
                scanResultStrings.add("[저장됨] $name\n${device.address}")
            }
        }
        listAdapter.notifyDataSetChanged()

        if (bluetoothAdapter?.isDiscovering == true) {
            bluetoothAdapter?.cancelDiscovery()
        }
        bluetoothAdapter?.startDiscovery()

        if (scanDialog == null || !scanDialog!!.isShowing) {
            val builder = AlertDialog.Builder(this)
            builder.setTitle("장치 선택")
            builder.setAdapter(listAdapter) { _, which ->
                bluetoothAdapter?.cancelDiscovery()
                connectToDevice(scanResultList[which])
            }
            builder.setNegativeButton("취소") { dialog, _ ->
                bluetoothAdapter?.cancelDiscovery()
                dialog.dismiss()
            }
            scanDialog = builder.create()
            scanDialog?.show()
        }
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BluetoothDevice) {
        // Toast 대신 로그 사용
        addLog("연결 시도 중... (${device.name})")

        connectThread?.cancel()
        connectedThread?.cancel()

        connectThread = ConnectThread(device)
        connectThread?.start()
    }

    @SuppressLint("MissingPermission")
    private inner class ConnectThread(private val device: BluetoothDevice) : Thread() {
        private val mmSocket: BluetoothSocket? by lazy(LazyThreadSafetyMode.NONE) {
            device.createRfcommSocketToServiceRecord(SPP_UUID)
        }

        override fun run() {
            bluetoothAdapter?.cancelDiscovery()

            try {
                mmSocket?.connect()
            } catch (e: IOException) {
                Log.e("BT_TAG", "Socket connect failed", e)
                // Toast 삭제 -> 로그로 대체
                addLog("연결 실패: ${e.message}")
                cancel()
                return
            }

            mmSocket?.let { manageMyConnectedSocket(it) }
        }

        fun cancel() {
            try { mmSocket?.close() } catch (e: IOException) { }
        }
    }

    private fun manageMyConnectedSocket(socket: BluetoothSocket) {
        connectedThread = ConnectedThread(socket)
        connectedThread?.start()

        runOnUiThread {
            setUiConnectedState()
            // Toast 삭제
        }
        // 로그로 연결 성공 알림
        addLog("연결 성공! 데이터 수신 대기 중...")

        prevPressure = -1
        prevFlame = -1
    }

    private inner class ConnectedThread(private val socket: BluetoothSocket) : Thread() {
        private val mmInStream: InputStream = socket.inputStream
        private val mmBuffer: ByteArray = ByteArray(1024)

        override fun run() {
            var numBytes: Int
            val stringBuilder = StringBuilder()

            while (true) {
                try {
                    numBytes = mmInStream.read(mmBuffer)
                    val readMessage = String(mmBuffer, 0, numBytes)
                    stringBuilder.append(readMessage)

                    val endOfLineIndex = stringBuilder.indexOf("\n")
                    if (endOfLineIndex > 0) {
                        val completeData = stringBuilder.substring(0, endOfLineIndex).trim()
                        stringBuilder.delete(0, endOfLineIndex + 1)
                        mHandler.obtainMessage(MESSAGE_READ, completeData).sendToTarget()
                    }
                } catch (e: IOException) {
                    // 연결 끊김 발생 시
                    addLog("연결이 종료되었습니다.")
                    runOnUiThread {
                        stopConnection()
                    }
                    break
                }
            }
        }

        fun cancel() {
            try { socket.close() } catch (e: IOException) { }
        }
    }

    private fun stopConnection() {
        connectThread?.cancel()
        connectedThread?.cancel()
        connectThread = null
        connectedThread = null

        setUiDisconnectedState()
        // 여기서도 로그 남기기 (중복 호출 방지를 위해 필요시 조정 가능)
        // addLog("연결 종료 처리됨")
    }

    private val mHandler = object : Handler(Looper.getMainLooper()) {
        override fun handleMessage(msg: Message) {
            if (msg.what == MESSAGE_READ) {
                val readMessage = msg.obj as String
                parseAndDisplay(readMessage)
            }
        }
    }

    // =========================================================================
    // 데이터 파싱 및 로그 로직
    // =========================================================================
    private fun parseAndDisplay(data: String) {
        if (data.isEmpty()) return

        try {
            val parts = data.split("/")

            val validColor = Color.BLACK
            val alertColor = Color.RED

            for (part in parts) {
                val keyValue = part.split(":")
                if (keyValue.size == 2) {
                    val key = keyValue[0].trim().uppercase()
                    val valueStr = keyValue[1].trim()

                    if (valueStr.isEmpty()) continue

                    when (key) {
                        "TEMP" -> {
                            tvValTemp.text = "$valueStr °C"
                            tvValTemp.setTextColor(validColor)
                        }
                        "PRES" -> {
                            tvValPressure.text = valueStr
                            tvValPressure.setTextColor(validColor)

                            val currentPres = valueStr.toIntOrNull() ?: 0
                            if (prevPressure != -1) {
                                if (prevPressure > 50 && currentPres < 10) {
                                    addLog("⚠[도난 경보] 물건 제거 감지됨!")
                                }
                                if (prevPressure < 10 && currentPres > 50) {
                                    addLog("[보관 감지] 물건 보관됨.")
                                }
                            }
                            prevPressure = currentPres
                        }
                        "FLAME" -> {
                            val currentFlame = valueStr.toIntOrNull() ?: 0

                            if (currentFlame == 1) {
                                tvValFlame.text = "DETECTED"
                                tvValFlame.setTextColor(alertColor)

                                if (prevFlame != 1) {
                                    addLog("[화재 경보] 불꽃 감지됨!")
                                }
                            } else {
                                tvValFlame.text = "SAFE"
                                tvValFlame.setTextColor(Color.GREEN)

                                if (prevFlame == 1) {
                                    addLog("[화재 해제] 상태 안전.")
                                }
                            }
                            prevFlame = currentFlame
                        }
                    }
                }
            }
        } catch (e: Exception) {
            // 파싱 에러도 로그에 남기기 (디버깅용)
            // addLog("데이터 형식 오류: $data")
        }
    }
}
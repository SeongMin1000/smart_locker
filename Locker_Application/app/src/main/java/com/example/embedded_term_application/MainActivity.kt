package com.example.embedded_term_application

import android.Manifest
import android.annotation.SuppressLint
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
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
import android.widget.ImageView
import android.widget.ScrollView
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.app.NotificationCompat
import androidx.core.app.NotificationManagerCompat
import androidx.core.content.ContextCompat
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.text.SimpleDateFormat
import java.util.*
import kotlin.collections.ArrayList

class MainActivity : AppCompatActivity() {

    companion object {
        // SPP UUID (FB755AAC 등 시리얼 통신용 표준 UUID)
        val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        const val MESSAGE_READ = 0
        const val CHANNEL_ID = "SAFE_GUARD_ALERT_CHANNEL"
    }

    private lateinit var btnBluetoothConnect: Button
    private lateinit var tvStatusText: TextView
    private lateinit var btnLock: Button
    private lateinit var ivStatusIcon: ImageView
    private lateinit var btnRefreshLog: Button

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

    // 상태 변수
    private var prevPressure = -1
    private var prevFlame = -1
    private var isLocked = true // 초기 상태: 잠금

    // 권한 요청 런처
    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        // 블루투스 권한 확인
        val btConnect = permissions[Manifest.permission.BLUETOOTH_CONNECT] ?: false
        val btScan = permissions[Manifest.permission.BLUETOOTH_SCAN] ?: false

        if (btConnect && btScan) {
            startDiscovery()
        } else {
            addLog("[경고] 권한이 거부되어 장치를 찾을 수 없습니다.")
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
                        scanResultStrings.add("[저장됨] $name\n$address")
                        listAdapter.notifyDataSetChanged()
                    }
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        // 알림 채널 생성 (안드로이드 8.0 이상)
        createNotificationChannel()

        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter

        initViews()
        setUiDisconnectedState() // 초기 UI 상태 설정

        // 블루투스 연결/해제 버튼
        btnBluetoothConnect.setOnClickListener {
            if (connectedThread != null) {
                stopConnection()
            } else {
                checkPermissionsAndScan()
            }
        }

        // 로그 새로고침(초기화) 버튼
        btnRefreshLog.setOnClickListener {
            tvLogResult.text = ""
            addLog("로그가 초기화되었습니다.")
        }

        // [핵심] 잠금/해제 버튼 리스너
        btnLock.setOnClickListener {
            if (connectedThread == null) {
                addLog("[오류] 장치가 연결되지 않았습니다.")
                Toast.makeText(this, "먼저 블루투스를 연결해주세요.", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }

            if (isLocked) {
                // 현재 잠김 상태 -> 해제 명령 전송
                sendData("CMD:UNLOCK\n")
                isLocked = false
                updateLockUI()
                addLog("[명령] 잠금 해제 요청 보냄")
            } else {
                // 현재 해제 상태 -> 잠금 명령 전송
                sendData("CMD:LOCK\n")
                isLocked = true
                updateLockUI()
                addLog("[명령] 잠금 요청 보냄")
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
        btnRefreshLog = findViewById(R.id.btnRefreshLog)

        // 잠금 관련 뷰
        btnLock = findViewById(R.id.btnLock)
        ivStatusIcon = findViewById(R.id.ivStatusIcon)

        // 센서 카드 뷰
        val cardTemp = findViewById<View>(R.id.cardTemp)
        cardTemp.findViewById<TextView>(R.id.tvSensorLabel).text = "Temperature"
        tvValTemp = cardTemp.findViewById(R.id.tvSensorValue)

        val cardPressure = findViewById<View>(R.id.cardPressure)
        cardPressure.findViewById<TextView>(R.id.tvSensorLabel).text = "Pressure"
        tvValPressure = cardPressure.findViewById(R.id.tvSensorValue)

        val cardFlame = findViewById<View>(R.id.cardFlame)
        cardFlame.findViewById<TextView>(R.id.tvSensorLabel).text = "Flame"
        tvValFlame = cardFlame.findViewById(R.id.tvSensorValue)

        // 초기 잠금 상태 UI 반영
        updateLockUI()
    }

    // 잠금 상태에 따라 UI(버튼, 텍스트, 아이콘 색상) 업데이트
    private fun updateLockUI() {
        if (isLocked) {
            tvStatusText.text = "SAFE CLOSED"
            tvStatusText.setTextColor(Color.parseColor("#4CAF50")) // 녹색
            ivStatusIcon.setColorFilter(Color.parseColor("#4CAF50"))
            btnLock.text = "UNLOCK SAFE" // 누르면 열리도록 텍스트 변경
            btnLock.background.setTint(Color.parseColor("#F44336")) // 빨간색 버튼
        } else {
            tvStatusText.text = "SAFE OPENED"
            tvStatusText.setTextColor(Color.parseColor("#F44336")) // 빨간색 (열림 주의)
            ivStatusIcon.setColorFilter(Color.parseColor("#F44336"))
            btnLock.text = "LOCK SAFE" // 누르면 잠기도록 텍스트 변경
            btnLock.background.setTint(Color.parseColor("#2196F3")) // 파란색 버튼
        }
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
            ivStatusIcon.setColorFilter(Color.GRAY)
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

            // 연결되면 현재 잠금 상태 UI 반영
            updateLockUI()

            btnBluetoothConnect.text = "연결 해제"
        }
    }

    // 로그 출력 함수
    private fun addLog(message: String) {
        runOnUiThread {
            val currentTime = SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(Date())
            val logMessage = "[$currentTime] $message\n"
            tvLogResult.append(logMessage)
            svLogContainer.post {
                svLogContainer.fullScroll(View.FOCUS_DOWN)
            }
        }
    }

    // 아두이노로 데이터 전송
    private fun sendData(message: String) {
        if (connectedThread != null) {
            connectedThread?.write(message.toByteArray())
        }
    }

    private fun checkPermissionsAndScan() {
        val permissions = mutableListOf<String>()
        // 안드로이드 13(Tiramisu) 이상은 알림 권한도 요청
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            permissions.add(Manifest.permission.POST_NOTIFICATIONS)
        }

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
        }
        addLog("연결 성공! 데이터 수신 대기 중...")

        prevPressure = -1
        prevFlame = -1
    }

    private inner class ConnectedThread(private val socket: BluetoothSocket) : Thread() {
        private val mmInStream: InputStream = socket.inputStream
        private val mmOutStream: OutputStream = socket.outputStream // 데이터 전송용
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
                    addLog("연결이 종료되었습니다.")
                    runOnUiThread {
                        stopConnection()
                    }
                    break
                }
            }
        }

        // [추가] 아두이노로 데이터 쓰기
        fun write(bytes: ByteArray) {
            try {
                mmOutStream.write(bytes)
            } catch (e: IOException) {
                addLog("[오류] 데이터 전송 실패")
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
        addLog("연결 종료됨")
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
    // 데이터 파싱 및 알림 로직
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
                            tvValTemp.text = "$valueStr C" // 특수문자 제거
                            tvValTemp.setTextColor(validColor)
                        }
                        "PRES" -> {
                            tvValPressure.text = valueStr
                            tvValPressure.setTextColor(validColor)

                            val currentPres = valueStr.toIntOrNull() ?: 0
                            if (prevPressure != -1) {
                                if (prevPressure > 50 && currentPres < 10) {
                                    addLog("[도난 경보] 물건 제거 감지됨!")
                                    sendNotification("도난 경보", "보관함에서 물건이 제거되었습니다!")
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
                                    sendNotification("화재 경보", "불꽃이 감지되었습니다! 확인이 필요합니다.")
                                }
                            } else {
                                tvValFlame.text = "SAFE"
                                tvValFlame.setTextColor(Color.parseColor("#4CAF50"))

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
            // Log.e("Parsing", "Data Error")
        }
    }

    // =========================================================================
    // 알림(Notification) 관련 함수
    // =========================================================================
    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val name = "SmartSafeGuard Alerts"
            val descriptionText = "화재 및 도난 경보 알림"
            val importance = NotificationManager.IMPORTANCE_HIGH
            val channel = NotificationChannel(CHANNEL_ID, name, importance).apply {
                description = descriptionText
            }
            val notificationManager: NotificationManager =
                getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
            notificationManager.createNotificationChannel(channel)
        }
    }

    @SuppressLint("MissingPermission")
    private fun sendNotification(title: String, message: String) {
        // 안드로이드 13 이상에서 알림 권한 체크
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED) {
                addLog("[오류] 알림 권한이 없어 푸시 알림 실패")
                return
            }
        }

        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(android.R.drawable.ic_dialog_alert) // 기본 아이콘 사용
            .setContentTitle(title)
            .setContentText(message)
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setAutoCancel(true)

        with(NotificationManagerCompat.from(this)) {
            // Notification ID를 매번 다르게 주어 알림이 쌓이도록 함 (현재시간 활용)
            notify(System.currentTimeMillis().toInt(), builder.build())
        }
    }
}
package com.example.embedded_term_application

import android.Manifest
import android.annotation.SuppressLint
import android.app.NotificationChannel
import android.app.NotificationManager
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
import android.view.WindowManager
import android.widget.*
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
        val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
        const val MESSAGE_READ = 0
        const val CHANNEL_ID = "SAFE_GUARD_URGENT_ALARM"
    }

    private lateinit var btnBluetoothConnect: Button
    private lateinit var tvStatusText: TextView
    private lateinit var btnLock: Button
    private lateinit var ivStatusIcon: ImageView
    private lateinit var btnRefreshLog: Button
    private lateinit var btnShareLog: Button
    private lateinit var cbShowRx: CheckBox

    private lateinit var tvValTemp: TextView
    private lateinit var tvValPressure: TextView
    private lateinit var tvValFlame: TextView

    private lateinit var tvLogResult: TextView
    private lateinit var svLogContainer: ScrollView

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var connectThread: ConnectThread? = null
    private var connectedThread: ConnectedThread? = null

    private val scanResultList = ArrayList<BluetoothDevice>()
    private val scanResultStrings = ArrayList<String>()
    private lateinit var listAdapter: ArrayAdapter<String>
    private var scanDialog: AlertDialog? = null

    private var isLocked = false
    private var isEmergencyDialogShowing = false

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
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

        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        createNotificationChannel()

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

        btnRefreshLog.setOnClickListener {
            tvLogResult.text = ""
            addLog("로그가 초기화되었습니다.")
        }

        btnShareLog.setOnClickListener {
            val logText = tvLogResult.text.toString()
            if (logText.isEmpty()) {
                Toast.makeText(this, "공유할 로그가 없습니다.", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            val shareIntent = Intent().apply {
                action = Intent.ACTION_SEND
                putExtra(Intent.EXTRA_TEXT, logText)
                type = "text/plain"
            }
            startActivity(Intent.createChooser(shareIntent, "로그 공유하기"))
        }

        btnLock.setOnClickListener {
            if (connectedThread == null) {
                addLog("[오류] 장치가 연결되지 않았습니다.")
                Toast.makeText(this, "먼저 블루투스를 연결해주세요.", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            if (isLocked) {
                sendData("CMD:UNLOCK\n")
                isLocked = false
                updateLockUI()
                addLog("[명령] 잠금 해제 요청")
            } else {
                sendData("CMD:LOCK\n")
                isLocked = true
                updateLockUI()
                addLog("[명령] 잠금 요청")
            }
        }

        val filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
        registerReceiver(receiver, filter)

        addLog("앱이 준비되었습니다.")
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
        btnShareLog = findViewById(R.id.btnShareLog)

        btnLock = findViewById(R.id.btnLock)
        ivStatusIcon = findViewById(R.id.ivStatusIcon)

        cbShowRx = findViewById(R.id.cbShowRx)

        val cardTemp = findViewById<View>(R.id.cardTemp)
        cardTemp.findViewById<TextView>(R.id.tvSensorLabel).text = "Temperature"
        tvValTemp = cardTemp.findViewById(R.id.tvSensorValue)

        val cardPressure = findViewById<View>(R.id.cardPressure)
        tvValPressure = findViewById(R.id.tvValPressure)

        val cardFlame = findViewById<View>(R.id.cardFlame)
        cardFlame.findViewById<TextView>(R.id.tvSensorLabel).text = "Flame"
        tvValFlame = cardFlame.findViewById(R.id.tvSensorValue)

        updateLockUI()
    }

    private fun showEmergencyPopup(title: String, message: String) {
        if (isEmergencyDialogShowing) return

        runOnUiThread {
            isEmergencyDialogShowing = true

            val builder = AlertDialog.Builder(this)
            builder.setTitle("🚨 $title 🚨")
            builder.setMessage(message)
            builder.setIcon(android.R.drawable.ic_dialog_alert)
            builder.setCancelable(false)

            builder.setPositiveButton("확인") { dialog, _ ->
                isEmergencyDialogShowing = false
                dialog.dismiss()
            }

            val dialog = builder.create()
            dialog.show()
            dialog.getButton(AlertDialog.BUTTON_POSITIVE).setTextColor(Color.RED)
        }
    }

    private fun updateLockUI() {
        if (isLocked) {
            tvStatusText.text = "SAFE CLOSED"
            tvStatusText.setTextColor(Color.parseColor("#4CAF50"))
            ivStatusIcon.setColorFilter(Color.parseColor("#4CAF50"))
            btnLock.text = "UNLOCK SAFE"
            btnLock.background.setTint(Color.parseColor("#F44336"))
        } else {
            tvStatusText.text = "SAFE OPENED"
            tvStatusText.setTextColor(Color.parseColor("#F44336"))
            ivStatusIcon.setColorFilter(Color.parseColor("#F44336"))
            btnLock.text = "LOCK SAFE"
            btnLock.background.setTint(Color.parseColor("#2196F3"))
        }
    }

    private fun setUiDisconnectedState() {
        runOnUiThread {
            tvStatusText.text = "DISCONNECTED"
            tvStatusText.setTextColor(Color.GRAY)
            ivStatusIcon.setColorFilter(Color.GRAY)
            btnBluetoothConnect.text = "장치 찾기"

            tvValTemp.text = "NONE"
            tvValTemp.setTextColor(Color.RED)

            tvValPressure.text = "NONE"
            tvValPressure.setTextColor(Color.RED)

            tvValFlame.text = "NONE"
            tvValFlame.setTextColor(Color.RED)
        }
    }

    private fun setUiConnectedState() {
        runOnUiThread {
            updateLockUI()
            btnBluetoothConnect.text = "연결 해제"

            tvValPressure.text = "정상"
            tvValPressure.setTextColor(Color.parseColor("#4CAF50"))

            tvValFlame.text = "안전"
            tvValFlame.setTextColor(Color.BLUE)
        }
    }

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

    private fun sendData(message: String) {
        if (connectedThread != null) {
            connectedThread?.write(message.toByteArray())
        }
    }

    private fun checkPermissionsAndScan() {
        val permissions = mutableListOf<String>()
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
    }

    private inner class ConnectedThread(private val socket: BluetoothSocket) : Thread() {
        private val mmInStream: InputStream = socket.inputStream
        private val mmOutStream: OutputStream = socket.outputStream
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
                    runOnUiThread { stopConnection() }
                    break
                }
            }
        }
        fun write(bytes: ByteArray) {
            try { mmOutStream.write(bytes) } catch (e: IOException) { addLog("[오류] 데이터 전송 실패") }
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

    private fun parseAndDisplay(data: String) {
        if (cbShowRx.isChecked) {
            addLog("[RX] $data")
        }
        val cleanData = data.trim()
        if (cleanData.isEmpty()) return

        if (cleanData.contains("THEFT DETECTED")) {
            addLog("🚨 [도난 발생] 물건 도난 감지됨!")
            sendNotification("도난 경보", "보관함에서 물건이 제거되었습니다!")

            tvValPressure.text = "비정상"
            tvValPressure.setTextColor(Color.RED)

            showEmergencyPopup("도난 경보", "보관함에서 물건 도난이 감지되었습니다!\n즉시 확인하십시오.")
            return
        }

        if (cleanData.contains("MOVING DETECTED")) {
            addLog("🏃 [이동 감지] 보관함이 이동 중입니다!")
            sendNotification("이동 경보", "보관함의 위치 이동이 감지되었습니다!")
            showEmergencyPopup("이동 경보", "누군가 보관함을 들고 이동 중입니다!\n(Moving Detected)")
            return
        }

        if (cleanData.contains("FLAME DETECTED")) {
            addLog("🔥 [화재 발생] 불꽃 감지됨!")
            sendNotification("화재 경보", "보관함 주변에 불꽃이 감지되었습니다!")

            tvValFlame.text = "화재 경고"
            tvValFlame.setTextColor(Color.RED)

            showEmergencyPopup("화재 경보", "불꽃이 감지되었습니다!\n즉시 조치하십시오.")
            return
        }

        if (cleanData.startsWith("T=")) {
            try {
                val numberOnly = cleanData.substring(2).filter { it.isDigit() || it == '-' }
                if (numberOnly.isNotEmpty()) {
                    tvValTemp.text = "$numberOnly °C"
                    tvValTemp.setTextColor(Color.BLACK)
                }
            } catch (e: Exception) {
            }
        }
    }

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
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            if (ActivityCompat.checkSelfPermission(this, Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED) {
                addLog("[오류] 알림 권한이 없어 푸시 알림 실패")
                return
            }
        }
        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(android.R.drawable.ic_dialog_alert)
            .setContentTitle(title)
            .setContentText(message)
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setAutoCancel(true)
        with(NotificationManagerCompat.from(this)) {
            notify(System.currentTimeMillis().toInt(), builder.build())
        }
    }
}
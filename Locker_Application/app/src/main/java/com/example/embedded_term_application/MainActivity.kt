package com.example.embedded_term_application

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.SharedPreferences
import android.content.pm.PackageManager
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.example.embedded_term_application.R

class MainActivity : AppCompatActivity() {

    private lateinit var btnBluetoothConnect: Button
    private lateinit var tvStatusText: TextView
    private lateinit var btnLock: Button

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bluetoothGatt: BluetoothGatt? = null
    private var isScanning = false
    private val handler = Handler(Looper.getMainLooper())

    private var targetDeviceName: String? = null
    private lateinit var prefs: SharedPreferences

    // 스캔 결과 리스트
    private val scanResultList = ArrayList<BluetoothDevice>()
    private val scanResultStrings = ArrayList<String>()
    private lateinit var listAdapter: ArrayAdapter<String>
    private var scanDialog: AlertDialog? = null

    // Classic 스캔 리시버 등록 여부
    private var isReceiverRegistered = false

    // 권한 요청
    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.values.all { it }) {
            processScanAction()
        } else {
            Toast.makeText(this, "권한이 거부되었습니다. 설정에서 권한을 허용해주세요.", Toast.LENGTH_LONG).show()
        }
    }

    // Classic 블루투스 검색 결과 수신 리시버
    private val classicScanReceiver = object : BroadcastReceiver() {
        @SuppressLint("MissingPermission")
        override fun onReceive(context: Context, intent: Intent) {
            val action = intent.action
            if (BluetoothDevice.ACTION_FOUND == action) {
                // 장치 발견!
                val device: BluetoothDevice? = if (Build.VERSION.SDK_INT >= 33) {
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
                } else {
                    @Suppress("DEPRECATION")
                    intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                }

                device?.let {
                    val name = it.name
                    val addr = it.address
                    // 이름이 없거나 주소만 있는 경우도 일단 다 추가 (디버깅 위해)
                    addDeviceToList(it, 0)
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        prefs = getSharedPreferences("BleAppPrefs", Context.MODE_PRIVATE)
        targetDeviceName = prefs.getString("TARGET_NAME", null)

        initViews()

        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter

        btnBluetoothConnect.setOnClickListener {
            if (bluetoothGatt != null) {
                disconnectGatt()
            } else {
                checkPermissionsAndScan()
            }
        }

        // 롱클릭 시 초기화
        btnBluetoothConnect.setOnLongClickListener {
            targetDeviceName = null
            prefs.edit().remove("TARGET_NAME").apply()
            Toast.makeText(this, "초기화됨", Toast.LENGTH_SHORT).show()
            true
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        stopAllScan()
    }

    private fun initViews() {
        btnBluetoothConnect = findViewById(R.id.btnBluetoothConnect)
        tvStatusText = findViewById(R.id.tvStatusText)
        btnLock = findViewById(R.id.btnLock)
        listAdapter = ArrayAdapter(this, android.R.layout.simple_list_item_1, scanResultStrings)
    }

    private fun checkPermissionsAndScan() {
        // 1. GPS 켜짐 확인 (가장 중요)
        val locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        if (!locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
            Toast.makeText(this, "⚠️ 중요: 스캔을 위해 GPS를 켜주세요!", Toast.LENGTH_LONG).show()
            // 위치 설정 화면으로 보낼 수도 있음 (선택사항)
            return
        }

        // 2. 권한 목록 구성
        val permissions = mutableListOf<String>()

        // 위치 권한은 버전에 상관없이 필수로 넣습니다 (Classic 스캔 때문)
        permissions.add(Manifest.permission.ACCESS_FINE_LOCATION)
        permissions.add(Manifest.permission.ACCESS_COARSE_LOCATION)

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions.add(Manifest.permission.BLUETOOTH_SCAN)
            permissions.add(Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            permissions.add(Manifest.permission.BLUETOOTH)
            permissions.add(Manifest.permission.BLUETOOTH_ADMIN)
        }

        // 3. 권한 체크
        val notGranted = permissions.filter {
            ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }

        if (notGranted.isNotEmpty()) {
            requestPermissionLauncher.launch(notGranted.toTypedArray())
        } else {
            processScanAction()
        }
    }

    private fun processScanAction() {
        if (targetDeviceName != null) {
            Toast.makeText(this, "등록된 장치($targetDeviceName) 연결 시도", Toast.LENGTH_SHORT).show()
            startAllScan(true) // 자동 연결 모드
        } else {
            showDeviceScanDialog()
            startAllScan(false) // 목록 모드
        }
    }

    private fun showDeviceScanDialog() {
        scanResultList.clear()
        scanResultStrings.clear()
        listAdapter.notifyDataSetChanged()

        val builder = AlertDialog.Builder(this)
        builder.setTitle("장치 선택 (검색중...)")
        builder.setAdapter(listAdapter) { _, which ->
            stopAllScan()
            connectToDevice(scanResultList[which])
        }
        builder.setNegativeButton("취소") { dialog, _ ->
            stopAllScan()
            dialog.dismiss()
        }
        builder.setOnCancelListener { stopAllScan() }

        scanDialog = builder.create()
        scanDialog?.show()
    }

    @SuppressLint("MissingPermission")
    private fun startAllScan(isAuto: Boolean) {
        if (isScanning) return
        isScanning = true
        btnBluetoothConnect.text = "검색 중..."

        // 1. Classic 블루투스 스캔 시작 (BroadcastReceiver)
        val filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
        registerReceiver(classicScanReceiver, filter)
        isReceiverRegistered = true

        if (bluetoothAdapter?.isDiscovering == true) {
            bluetoothAdapter?.cancelDiscovery()
        }
        bluetoothAdapter?.startDiscovery()
        Log.d("BT_TAG", "Classic Discovery Started")

        // 2. BLE 스캔 시작
        val scanSettings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()
        bluetoothAdapter?.bluetoothLeScanner?.startScan(null, scanSettings, bleScanCallback)
        Log.d("BT_TAG", "BLE Scan Started")

        // 15초 후 중지
        handler.postDelayed({
            if (isScanning) {
                stopAllScan()
                if (!isAuto) {
                    Toast.makeText(this, "검색 종료. (${scanResultList.size}개 발견)", Toast.LENGTH_SHORT).show()
                    scanDialog?.setTitle("장치 선택 (완료)")
                }
            }
        }, 15000)
    }

    @SuppressLint("MissingPermission")
    private fun stopAllScan() {
        if (!isScanning) return
        isScanning = false
        btnBluetoothConnect.text = "BT 연결"

        // BLE 중지
        bluetoothAdapter?.bluetoothLeScanner?.stopScan(bleScanCallback)

        // Classic 중지
        bluetoothAdapter?.cancelDiscovery()

        // 리시버 해제
        if (isReceiverRegistered) {
            try {
                unregisterReceiver(classicScanReceiver)
            } catch (e: Exception) {
                e.printStackTrace()
            }
            isReceiverRegistered = false
        }
    }

    // BLE 콜백
    private val bleScanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            addDeviceToList(result.device, result.rssi)
        }
        override fun onScanFailed(errorCode: Int) {
            Log.e("BT_TAG", "BLE Scan Failed: $errorCode")
        }
    }

    @SuppressLint("MissingPermission")
    private fun addDeviceToList(device: BluetoothDevice, rssi: Int) {
        val name = device.name ?: "Unknown"
        val addr = device.address

        // UI 스레드에서 작업
        runOnUiThread {
            // 자동 연결 모드면 타겟 찾기
            if (targetDeviceName != null) {
                if (name == targetDeviceName) {
                    stopAllScan()
                    connectToDevice(device)
                }
                return@runOnUiThread
            }

            // 중복 방지 및 추가
            if (scanResultList.none { it.address == addr }) {
                scanResultList.add(device)

                // 표시 텍스트 구성
                val rssiStr = if (rssi != 0) "Signal: $rssi" else ""
                val displayText = if (name == "Unknown") {
                    "[$addr]\n$rssiStr" // 이름 없으면 주소 강조
                } else {
                    "$name\n$addr $rssiStr"
                }

                scanResultStrings.add(displayText)
                listAdapter.notifyDataSetChanged()
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BluetoothDevice) {
        scanDialog?.dismiss()
        Toast.makeText(this, "연결 시도: ${device.name}", Toast.LENGTH_SHORT).show()

        // BLE 연결 시도
        bluetoothGatt = device.connectGatt(this, false, gattCallback)
    }

    @SuppressLint("MissingPermission")
    private fun disconnectGatt() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        btnBluetoothConnect.text = "BT 연결"
        Toast.makeText(this, "연결 해제됨", Toast.LENGTH_SHORT).show()
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            runOnUiThread {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    val name = gatt.device.name ?: "Unknown"
                    Toast.makeText(applicationContext, "연결 성공! ($name)", Toast.LENGTH_LONG).show()
                    btnBluetoothConnect.text = "연결 해제"

                    // 이름 저장
                    if (targetDeviceName == null) {
                        prefs.edit().putString("TARGET_NAME", name).apply()
                        targetDeviceName = name
                    }
                    gatt.discoverServices()
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    Toast.makeText(applicationContext, "연결 끊김", Toast.LENGTH_SHORT).show()
                    btnBluetoothConnect.text = "BT 연결"
                }
            }
        }
    }
}
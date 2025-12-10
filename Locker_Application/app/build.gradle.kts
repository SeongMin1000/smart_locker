plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

android {
    namespace = "com.example.embedded_term_application"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.example.embedded_term_application"
        minSdk = 24
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }
}

dependencies {
    // ▼ [수정됨] 1.17.0 -> 1.13.1 로 버전을 낮췄습니다.
    implementation("androidx.core:core-ktx:1.13.1")

    implementation("androidx.appcompat:appcompat:1.7.1")
    implementation("com.google.android.material:material:1.13.0")
    implementation("androidx.constraintlayout:constraintlayout:2.2.1")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.3.0")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.7.0")
}

// ▼ [추가] 다른 라이브러리가 몰래 최신 버전을 가져오지 못하도록 강제하는 코드입니다.
// dependencies 블록 닫는 중괄호 } 바로 아래에 붙여넣으세요.
configurations.all {
    resolutionStrategy {
        force("androidx.core:core-ktx:1.13.1")
    }
}
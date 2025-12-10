# 스마트 보안 사물함




---

# 스마트 보안 사물함 어플리케이션
<h3 align="center">APK 다운로드 링크</h3>
<p align="center">
  <a href="https://drive.google.com/file/d/1du3aevooQzEX0ROPMGJoW2Bn3_NWNzR6/view?usp=sharing">
    Download APK
  </a>
</p>
<h2 align="center">주의 : 꼭 모든 권한을 허용해주세요!!</h2>

### Android Studio 프로젝트 임포트 방법
1. Branch를 `main` -> `Android_App` 으로 변경합니다.
2. `Android Studio`를 실행합니다. 
3. 상단 탭에서 `File` -> `Open`을 선택합니다.
4. `smart_locker\Locker_Application` 경로를 찾아 안드로이드 아이콘을 선택한 후 `OK`하시면 됩니다.

### 구현 기능
- `블루투스 연결 및 해제` 기능
- `LOCKSAFE`를 이용하여 `원격 잠금/잠금해제` 기능 (서보모터)
- `압력 센서 영점 조절` 기능
- `도난 감지 값 설정` 기능
- `로그 기록 및 공유` 기능

### 구현되었으나 확인하지 못한 기능
- 센서 감지 (연결 이후 센서가 감지되지 않으면 `NONE`을 출력합니다.)
- 센서 값 받아오기
- 압력 센서 영점 조절 기능
- 도난 감지 값 설정 기능 

---
### 기능 상세 설명
| 기능 | 설명 | 버튼 위치 |
| :--: | :-- | :--: |
| __블루투스 연결 및 해제__ | 블루투스 장치를 탐색 및 연결하는 기능입니다.<br>연결이 되면 `연결 해제` 기능으로 변경되며,<br>클릭 시 사물함과 어플 간의 연결이 해제됩니다. | <img src="https://github.com/user-attachments/assets/b08f4bf2-0d0e-4df3-97f0-9d9a6f8a3e0a" width="200" height="400"> |
| __원격 잠금 및 잠금해제__| 장치가 연결되어 있으면 활성화됩니다.<br>기본값은 `CLOSED`입니다.<br>버튼을 통해 잠금 상태를 토글할 수 있습니다.| <img width="200" height="400" alt="image" src="https://github.com/user-attachments/assets/5a757a3a-89d1-487c-9cc1-1c4e503f7ee4" /> |
| __압력 센서 영점 조절__ | `압력 센서의 영점을 조절`하는 기능입니다.<br>실험 편의성을 위해 추가하였습니다. | <img width="200" height="400" alt="image" src="https://github.com/user-attachments/assets/c9496550-faa4-4576-a8ef-4c05e6f98bbb" /> |
| __도난 감지 값 설정__ | `도난 감지 기준값을 설정`하는 기능입니다.<br>`현재 압력 값 >= 물건 감지 기준값` 이라면 `정상`,<br>`현재 압력 값 < 도난 판단 기준값`이라면 `도난`<br>으로 판단합니다.<br>__지정한 값은 데이터로 저장됩니다.__ | <img width="100" alt="image" src="https://github.com/user-attachments/assets/0572a4f3-2f02-4e39-9274-36f544003b1f" /><img width="100" alt="image" src="https://github.com/user-attachments/assets/28a3845e-952e-4170-ac8a-457fd29d37af" /> |
| __로그 기록 및 공유__ | `어플과 장치 간의 상호작용`을 로그로 기록하는 기능입니다.<br>자세한 정보는 이곳에서 확인 가능합니다.<br>또한 로그 위의 `SHARE` `CLEAR` 버튼을 통해<br>로그를 공유 및 초기화할 수 있습니다.| <img width="100" alt="image" src="https://github.com/user-attachments/assets/ecea1f25-d676-4e32-8480-94a4d485a916" /> <img width="100" alt="image" src="https://github.com/user-attachments/assets/63eb4eb3-e338-48fd-840e-5e243df08afe" /> |

### 예시 화면
<img src="https://github.com/user-attachments/assets/1ef4111e-fa3e-4220-9a83-5eb3fa81019f" width="300" alt="image">

# Wi-Fi UDP Wake (fork feature)

이 포크는 상류 [awalol/DS5Dongle](https://github.com/awalol/ds5dongle) 에
**Wi-Fi UDP 웨이크**를 추가한다. 네트워크로 패킷 하나를 보내 자고 있는 PC를 깨운다.

Wake-on-PS와 나란히 동작한다. 키보드 인터페이스와 웨이크 FSM은 상류 것을 그대로 쓰고,
트리거 소스만 하나 늘렸다.

## 왜 필요한가

상류의 Wake-on-PS는 **USB 버스가 서스펜드된 뒤에만** 키를 보낸다. `tud_remote_wakeup()` 이
`_usbd_dev.suspended` 를 요구하기 때문이다. 그래서 README에 S3 전용이라고 적혀 있다.

모던 스탠바이(S0 저전력 유휴) 호스트는 버스를 띄워둔 채 잠들 수 있고, 그러면 그 경로는
아무것도 보내지 않는다. 네트워크 웨이크는 이 게이트가 필요 없다 — 웨이크 패킷은 사용자의
명시적 요청이라, 게임 중에 잘못 발사될 위험이 없기 때문이다.

## 빌드

```sh
cmake -B build -G Ninja -DPICO_SDK_PATH=... -DCMAKE_BUILD_TYPE=Release -DENABLE_WIFI_WAKE=ON .
cmake --build build
```

`-DENABLE_WIFI_WAKE` 를 빼면 lwIP가 링크되지 않고, **적재되는 섹션이 상류와 전부 동일한**
펌웨어가 나온다 (디버그 정보만 다름).

빌드된 `.uf2` 는 Actions 아티팩트로도 받을 수 있다 (`wifi-wake` 워크플로).

## 쓰는 법

**1. Wake-on-PS 토글을 켠다.** 웹 config의 **Wake PC from sleep on PS button**.
이 토글이 켜져 있을 때만 키보드 인터페이스가 열거되고 remote wakeup 속성이 붙는다
(`usb_descriptors.cpp` 참고). 네트워크 웨이크도 그 인터페이스를 쓴다.

**2. Wi-Fi 자격증명을 넣는다.** 소스에 하드코딩하지 않는다. 빌드된 `.uf2` 안의 설정 블록에
나중에 써넣는다:

- `tools/uf2-wifi-config.html` 을 브라우저로 연다 (더블클릭, 인터넷 불필요)
- `.uf2` 를 고르고 SSID / 비밀번호 입력 → 설정이 들어간 `.uf2` 가 내려받아진다
- BOOTSEL 누른 채 연결 → `RP2350` 드라이브에 드래그 앤 드롭

순수 클라이언트 JS다. 네트워크로 나가는 것이 없고, 개발환경도 필요 없다.
2.4GHz 대역이어야 한다 (CYW43439는 5GHz 미지원).

**3. 깨운다.** 같은 LAN의 아무 기기에서 `WAKE_G14` 를 UDP 9번으로 보낸다.
브로드캐스트로 보내면 동글의 IP를 몰라도 된다.

```powershell
# Windows
$u=New-Object Net.Sockets.UdpClient; $u.EnableBroadcast=$true; $b=[Text.Encoding]::ASCII.GetBytes("WAKE_G14"); $u.Send($b,$b.Length,"192.168.0.255",9)
```

```sh
# Linux / macOS
python3 -c "import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1);s.sendto(b'WAKE_G14',('192.168.0.255',9))"
```

주소의 마지막 칸을 `255` 로 (마스크가 `/24` 일 때). 자기 IP는 `ipconfig` / `ip addr` 로 확인.

호스트 쪽 전제는 상류의 [Wake-on-PS](README.md#wake-on-ps-optional) 항목과 같다.
모던 스탠바이 PC라면 전원 관리 탭의 절전 해제 체크박스가 회색인 것이 정상이고, 그래도 동작한다.

## 비용

| | RAM | FLASH |
|---|---|---|
| `ENABLE_WIFI_WAKE=OFF` | 398.8 KB | 371.2 KB |
| `ENABLE_WIFI_WAKE=ON` | 416.9 KB | 401.5 KB |
| 차이 | **+18.0 KB** | **+30.3 KB** |

520 KB 중 103.1 KB 여유. 플래시 증가가 작은 이유는 CYW43 **콤보 펌웨어(Wi-Fi + BT 통합)가
Bluetooth 때문에 이미 링크되어 있었기** 때문이다. 새로 들어간 것은 호스트 쪽 IP 스택뿐이다.

## 구현 메모

- 블로킹하지 않는다. 메인 루프는 1초 워치독 아래에서 Bluetooth 오디오까지 돌리므로,
  Wi-Fi 접속은 `cyw43_arch_wifi_connect_async()` 로 시작하고 폴링한다.
- UDP 수신 콜백은 cyw43 폴 컨텍스트에서 돈다. 플래그만 세우고 실제 HID 전송은 메인 루프에서
  한다. 매칭되지 않는 패킷 로그는 5초 레이트 리밋 — 포트 9는 Wake-on-LAN 브로드캐스트가
  routinely 날아드는 포트다.
- `wake_request_from_network()` 은 `WAKE_REQUESTED` 로 진입만 시킨다. 상류 FSM의
  `if (host_resumed_event || !host_suspended)` 분기가 서스펜드된 경우와 아닌 경우를 모두
  처리하므로, 재시도·세틀 타이밍·디스크립터·remote wakeup 신호는 상류 코드 그대로다.
- 대기 중 무선 부하는 바인드된 UDP pcb 하나이고, 송신은 전혀 하지 않는다.

## 미검증

**실물 테스트를 하지 않았다.** Bluetooth와 Wi-Fi가 CYW43439 라디오 하나를 시분할하므로,
컨트롤러 오디오와 입력 지연에 미치는 영향을 측정해야 한다. 상류 Known Issues에 이미
`Audio may experience slight stuttering` 이 있으므로, 기준선을 먼저 잡고 비교할 것.

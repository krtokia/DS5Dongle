# DS5Dongle에 Wi-Fi를 넣으면 왜 죽는가 — 조사 기록

Pico 2 W용 DS5Dongle에 Wi-Fi(lwIP)를 추가했더니 컨트롤러가 연결되지 않고 호스트가
"USB 장치를 인식할 수 없음"을 띄웠다. 원인을 찾는 데 여러 번의 빌드와 플래시를 썼고, 그중
대부분이 헛수고였다. 결론과 함께 **왜 헛수고였는지**를 남긴다. 후자가 더 쓸모 있다.

---

## 1. 증상

- 순정 DS5Dongle: 정상
- `-DENABLE_WIFI_WAKE=ON` 빌드: 컨트롤러가 붙지 않음
- **컨트롤러가 붙는 시점에** 호스트가 "USB 장치를 인식할 수 없음"
- 온보드 LED는 꺼진 채 (참고: 이 펌웨어에서 LED가 꺼진 것은 정상이다. inquiry 중에만
  0.2초 주기로 깜빡이고, 연결되면 켜진다)

---

## 2. 결론: 힙 고갈

시리얼 로그가 한 줄로 끝냈다.

```
[BT] State: 2
[BT] Stack ready, start inquiry
[HCI] CmdStatus HCI_INQUIRY(0x0401) status=0x00

*** PANIC ***

Out of memory
```

btstack이 inquiry를 시작하면서 할당에 실패한다. USB도, 워치독도, 무선 공존도 아니었다.

### 왜 힙이 줄었나

Pico SDK에서 **힙은 `.bss` 끝부터 RAM 꼭대기까지**다. 고정 크기가 아니라 "남는 것 전부"다.
따라서 정적으로 잡히는 버퍼가 늘면 **그만큼 힙이 직접 깎인다.**

lwIP의 pbuf 풀과 memp 풀은 전부 정적이다. 그래서 lwIP를 링크하는 것만으로 힙이 줄었다.

| 빌드 | `.bss` | 힙 | 결과 |
|---|---|---|---|
| 순정 | 68.5 KB | **119.5 KB** | 정상 |
| lwIP 링크 (초기 설정) | 86.5 KB | **101.5 KB** | OOM |
| lwIP 최소 설정 | 75.2 KB | **112.9 KB** | **여전히 OOM** |
| lwIP 최소 + `DISABLE_SPEAKER_PROC` | — | **423.9 KB** | 정상 |

즉 **btstack이 실전에서 필요로 하는 힙은 112.9 KB보다 크고 119.5 KB 이하**다.
순정의 마진이 6.6 KB도 안 된다.

### 왜 이렇게 빡빡한가

RP2350의 SRAM은 520 KB인데 DS5Dongle은 그중 약 400 KB를 **의도적으로** 쓴다.
오디오 성능을 위해 Opus 코덱과 핫패스 코드 약 310 KB를 플래시에서 RAM으로 재배치해
실행하기 때문이다 (`CMakeLists.txt` 의 `relocate_to_ram()` 과 `.time_critical.*` 참고).
그래야 오버클럭 없이 150 MHz로 오디오가 돌아간다.

칩이 작은 게 아니라, **이 펌웨어가 RAM을 크게 쓰는 설계**다.

### 현재의 해법

`-DDISABLE_SPEAKER_PROC=ON`. `audio.cpp` 의 스피커 출력 경로를 빼면 RAM으로 재배치되던
Opus 코드가 통째로 빠져 힙이 423.9 KB가 된다. 대가는 컨트롤러 내장 스피커와 3.5 mm 출력이며,
마이크 입력과 컨트롤러 기능은 남는다.

---

## 3. 틀린 가설 다섯 개와 배제 방법

여기가 이 문서의 본론이다. 각 가설은 그럴듯했고, 전부 틀렸다.

### 가설 1 — 1초 워치독이 리셋시킨다

`main.cpp` 는 메인 루프에 `watchdog_enable(1000, true)` 를 건다. `cyw43_arch_wifi_connect_async()`
는 이름과 달리 칩에 ioctl을 순차로 보내며 각각 응답을 기다리므로 1초를 넘길 수 있다.
초기 코드는 루프 첫 바퀴에 이걸 호출했다.

**배제**: `ENABLE_SERIAL=ON` 빌드는 워치독을 아예 걸지 않는다(`#if !ENABLE_SERIAL`).
그 빌드도 동일하게 실패했다.

### 가설 2 — RAM 재배치가 깨졌다

이 프로젝트는 `objcopy` PRE_LINK 단계로 USB/BT 핫패스 함수들을 `.time_critical.*` 로 옮겨
RAM에서 실행시킨다. 오브젝트 경로 접미사로 대상을 찾으므로, 빌드 구성이 바뀌면 조용히
no-op이 될 수 있다. 그러면 USB 타이밍이 빠듯해진다.

**배제**: 두 ELF에서 RAM 주소 범위에 있는 함수 심볼을 세어 비교했다. 902개 → 905개,
밀려난 것 0개. 늘어난 3개는 lwIP의 워커였다.

```sh
arm-none-eabi-nm -C build/x.elf | awk '$2 ~ /^[Tt]$/ && strtonum("0x"$1) >= 0x20000000'
```

### 가설 3 — 커진 바이너리가 config 플래시 영역을 덮는다

내 빌드가 플래시를 30 KB 더 쓴다. 설정이 프로그램 바로 뒤에 저장된다면 충돌한다.

**배제**: `config.cpp` 의 `CONFIG_FLASH_OFFSET = PICO_FLASH_BANK_STORAGE_OFFSET - FLASH_SECTOR_SIZE`
로 플래시 끝단이고, 바이너리는 약 715 KB 지점에서 끝난다.

### 가설 4 — 추가한 코드의 실행 타이밍이 문제다

컨트롤러가 붙는 순간 `bt.cpp` 가 `tud_connect()` 를 부르고 호스트가 열거를 시작한다.
바로 그때 Wi-Fi 재접속이 블로킹하면 열거가 깨진다 — 그럴듯했다.

**배제**: `WIFI_STACK_ONLY` 빌드를 만들었다. `CYW43_LWIP=1` 과 arch 교체만 하고 기능 소스는
컴파일에 넣지 않는다. 결과 바이너리에서 `wifi_wake`/`wifi_config` 심볼 0개를 확인했다.
**이 빌드도 동일하게 실패했다.** 코드가 없는데 실패하므로 코드의 타이밍이 아니다.

### 가설 5 — 스택이 넘친다

두 코어 스택이 각 2 KB(스크래치 뱅크)뿐인데 메인 루프는 btstack → TinyUSB → 오디오로 깊게
내려간다. 거기에 lwIP 워커가 `cyw43_arch_poll()` 안에 더해졌다.

**배제**: 4 KB로 늘린 빌드도 실패했다. (스택은 스크래치 뱅크에 있어 힙과 무관하므로
이 변경은 힙 문제를 건드리지 못했다.)

---

## 4. 배운 것

**① 빌드가 되고 링커가 통과하는 것은 동작한다는 증거가 아니다.**
이 조사의 초기 판단이 "빌드 성공 + RAM 103 KB 여유 → 통합 가능"이었다. 둘 다 사실이었지만
결론이 틀렸다.

**② "남은 RAM"은 여유가 아니라 힙이고, 이미 쓰는 주체가 있다.**
`arm-none-eabi-size` 로 계산한 여유 공간을 "쓸 수 있는 공간"으로 읽었는데, 그게 바로
btstack이 런타임에 할당해 쓰던 곳이었다. 임베디드에서 정적 크기만 보고 판단하면 이 함정에
빠진다. 힙의 실제 경계는 이렇게 확인한다:

```sh
arm-none-eabi-nm build/x.elf | grep -E "\b(end|__HeapLimit|__StackLimit)$"
```

**③ 로그를 먼저 뽑았어야 했다.**
가설 다섯 개를 세우고 넷을 바이너리 비교로 배제하는 데 여러 번의 빌드와 플래시를 썼다.
시리얼 로그 **한 번**이 곧바로 답을 줬다. 추론으로 좁히는 것보다 관측 하나가 빠르다.

**④ 로그를 뽑는 길을 미리 뚫어두면 값이 싸진다.**
이 저장소에는 이제 두 가지가 있다.
- `-DENABLE_SERIAL=ON` 빌드 — USB CDC 포트로 `printf` 출력. 하드웨어 추가 불필요
- `tools/serial-log.html` — WebSerial로 브라우저에서 바로 읽는 페이지. DTR을 알아서
  세운다 (이 펌웨어는 DTR로 터미널 접속을 판단하므로 이게 없으면 부팅 대기에서 멈춘다).
  `https://` 로 열어야 한다 — 파일 더블클릭은 브라우저가 시리얼 접근을 막는다

> `ENABLE_SERIAL=ON` 빌드는 **터미널이 붙을 때까지 아무것도 하지 않는다.** LED도 안 켜지고
> 컨트롤러도 안 붙는다. `cyw43_arch_init()` 보다 앞에서 기다리기 때문이며, 고장이 아니다.

---

## 5. 남은 과제

지금 해법은 스피커 출력을 포기하는 것이다. 둘 다 가지려면 다음 중 하나가 필요하다.

- **btstack의 실제 힙 피크를 측정해서 줄인다.** 지금은 "112.9 KB로는 부족, 119.5 KB면 충분"
  까지만 안다. btstack 설정(`btstack_config.h`)의 버퍼 수를 줄일 여지가 있는지 봐야 한다.
- **RAM 재배치 범위를 조정한다.** Opus 전체가 아니라 실측한 핫스팟만 옮기면 RAM이 남을 수
  있다. 오디오 성능과의 트레이드오프를 측정해야 한다.
- **lwIP를 더 줄인다.** 현재 6.6 KB인데, DHCP를 포기하고 고정 IP를 쓰면 몇 KB 더 줄어든다.
  다만 부족분이 6.6 KB보다 크므로 이것만으로는 부족할 가능성이 높다.
- **RAM이 더 큰 칩.** RP2350은 520 KB다.

세 번째까지 다 해도 마진이 몇 KB 수준이면 실용적이지 않다. 근본적으로는 **오디오 경로가
RAM을 310 KB 쓰는 설계와 Wi-Fi 스택이 같은 칩에서 공존하기 어렵다**는 것이 이 조사의 요지다.

### 대안

기능을 보드 두 대로 나누면 이 문제 자체가 사라진다.

```
Pico 2 W #1 → DS5Dongle 순정 (모든 기능)
Pico 2 W #2 → Wi-Fi 웨이크 전용
```

웨이크 전용 펌웨어는 [krtokia/pico-test](https://github.com/krtokia/pico-test) 에 있고,
실물에서 모던 스탠바이 PC를 깨우는 것까지 확인됐다.

---

## 부록: 재현

상류 CI와 SDK/TinyUSB를 맞춰야 빌드된다. pico-sdk 기본 TinyUSB로는 `TUD_AUDIO_EP_SIZE`
인자 개수가 맞지 않아 컴파일이 깨진다.

```sh
git clone --branch 2.3.0 --depth 1 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init --recursive
git -C lib/tinyusb fetch --depth 1 origin 2d56dc533e45e4e91b15e93fdab5e22e964f328d
git -C lib/tinyusb checkout --detach FETCH_HEAD
cd ..

git submodule update --init lib/WDL lib/opus

# 죽는 빌드
cmake -B build -G Ninja -DPICO_SDK_PATH=$PWD/pico-sdk -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_WIFI_WAKE=ON .
# 도는 빌드
cmake -B build -G Ninja -DPICO_SDK_PATH=$PWD/pico-sdk -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_WIFI_WAKE=ON -DDISABLE_SPEAKER_PROC=ON .
cmake --build build
```

힙 크기 확인:

```sh
arm-none-eabi-nm build/ds5-bridge.elf | grep -E "\b(end|__StackLimit)$"
# __StackLimit - end = 힙 크기
```

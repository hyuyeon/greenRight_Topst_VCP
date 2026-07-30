# CAN0 수신 디버깅 노트

## 목적

Nucleo-F429ZI가 Standard CAN 프레임을 송신하고, TCC70xx VCP 보드의 CAN0이 인터럽트 기반으로 수신한 뒤 UART0(Tera Term)로 출력하는 경로를 검증했다.

## 최종 구성

- 송신: Nucleo-F429ZI CAN1, Standard ID `0x100`, DLC 8, 데이터 `01 02 03 04 05 06 07 08`
- 수신: VCP CAN0 (`GPK8`: TX, `GPK1`: RX, `GPK13`: STB Low)
- nominal bitrate: 양쪽 모두 500 kbps
- VCP UART 로그: UART0, 115200 bps

## 문제와 원인

| 증상 | 원인 | 조치 |
| --- | --- | --- |
| VCP에서 CAN 수신 로그가 보이지 않음 | `AppTaskCreate()`가 `Main_StartTask()`에서 호출되지 않아 `CAN_DemoCreateApp()` 및 CAN RX task가 생성되지 않음 | `SAL_OsInitFuncs()` 뒤에 `AppTaskCreate()`를 호출 |
| 콘솔 명령을 입력할 수 없음 | Tera Term에는 단순 UART test만 열려 있었고 별도 Console task의 명령 입력 경로가 아님 | 콘솔 의존을 제거하고 `sTestInfo.tiRecv = TRUE`로 부팅 시 자동 수신 활성화 |
| `hello from uart`만 보이거나 이후 아무 로그도 보이지 않음 | CAN 수신은 `mcu_printf()`로 출력하지만 Tera Term은 UART0 test 포트를 보고 있었음. 이후 변경 과정에서 UART0 초기화/Open 코드까지 삭제되어 완전히 무출력 상태가 됨 | UART0 초기화·Open을 복구하고, 수신 로그를 `UART_Write(UART_TEST_CH, ...)`로 직접 출력. UART를 hello 후 닫지 않도록 변경 |
| 송신 쪽이 정상인지 확신하기 어려움 | Nucleo에 전송 요청/완료/ACK 상태를 UART로 보여 주는 진단이 없었음 | Nucleo USART3(ST-LINK VCP)에 부팅, 버튼, CAN 정상 모드, TX 완료·ACK 또는 TSR/ESR 오류를 출력 |
| Nucleo 진단 출력 추가 후 잠재적으로 멈출 수 있음 | USART3 초기화 전 `DebugUart_Write()`가 호출될 수 있었음 | `SystemClock_Config()` 직후 `DebugUart_Init()`을 실행한 다음 CAN 초기화 |

## VCP 수정 사항

### 태스크 생성

`sources/app.sample/app.base/main.c`:

```c
(void)SAL_OsInitFuncs();

UART_Test();
AppTaskCreate();
```

`AppTaskCreate()`가 `CAN_DemoCreateApp()`을 호출하고, 이 함수가 `Can Demo Task`를 만든다.

### 인터럽트에서 태스크로 전달

`sources/app.sample/app.can.demo/can_demo.c`:

1. CAN RX callback(ISR)이 `gReceiveFlag[channel]`을 설정한다.
2. ISR-safe `SAL_EventSet()`으로 채널별 RX event bit를 세트한다.
3. `CAN_DemoTask()`는 `SAL_EventGet(... SET_ANY | CONSUME | BLOCKING ...)`으로 대기한다.
4. event가 발생한 채널의 FIFO를 `CAN_GetNewRxMessage()`로 비우고 UART0에 출력한다.

부팅 시 `sTestInfo.tiRecv = TRUE`를 설정해 별도 콘솔 명령 없이 수신 출력을 활성화했다.

### UART0 직접 출력

`sources/dev.drivers/uart/tcc70xx/uart_example.c`는 UART0을 115200 bps polling mode로 열고 유지한다. CAN task는 `mcu_printf()` 대신 이 포트로 직접 출력한다.

정상 부팅 로그:

```text
[VCP DBG] UART0 ready
[VCP DBG] CAN RX event created
[VCP DBG] CAN RX task started
```

정상 수신 로그 예:

```text
[VCP DBG] CAN RX event
[CAN RX] CH0 ID:0x100 DLC:8 DATA: 01 02 03 04 05 06 07 08
```

## Nucleo 수정 사항

`C:\ST\Tlqkf\Core\Src\main.c`에 USART3(ST-LINK Virtual COM, PD8/PD9, 115200 bps) 진단을 추가했다.

정상 흐름:

```text
[NUC DBG] boot
[NUC DBG] CAN normal mode
[NUC DBG] CAN configured, press USER button
[NUC DBG] button -> CAN TX ID=0x100
[NUC DBG] TX complete / ACK received
```

`TX complete / ACK received`는 적어도 하나의 다른 CAN 노드가 ACK 했다는 뜻이다. 실패 시 TSR/ESR를 출력하도록 해 물리 버스 문제를 구분할 수 있게 했다.

## 빌드 및 다운로드

VCP:

```bash
cd ~/topst-vcp/build/tcc70xx/gcc
make

sudo ~/topst-vcp/tools/fwdn_vcp/fwdn \
  --fwdn ~/topst-vcp/tools/fwdn_vcp/vcp_fwdn.rom \
  -w ~/topst-vcp/build/tcc70xx/gcc/output/tcc70xx_pflash_boot_2M_ECC.rom
```

Nucleo:

1. STM32CubeIDE에서 `C:\ST\Tlqkf` 프로젝트를 Build한다.
2. ST-LINK로 다운로드한다.
3. ST-LINK Virtual COM을 115200 bps로 연다.

## 재발 방지 체크

- CAN task를 추가했다면 실제 startup task에서 task-creation 함수를 호출하는지 확인한다.
- 로그 함수와 현재 관측 중인 UART 포트가 같은 경로인지 확인한다.
- UART init/open 이후에만 UART write를 수행하며, 수신 로그에 사용하는 포트를 닫지 않는다.
- ISR에서는 긴 출력 대신 event/counter만 처리하고, 프레임 파싱·출력은 task에서 수행한다.
- Nucleo의 TX 완료·ACK 로그와 VCP의 RX event 로그를 함께 보며 물리 계층과 소프트웨어 경로를 분리한다.

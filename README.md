# RIDI PAPER PRO · Linux 커널

RIDI PAPER PRO(E70Q10)의 Linux 4.9 커널 소스입니다. NXP i.MX6SoloLite 커널을
기반으로 기기 전용 디스플레이·입력·전원 장치를 지원합니다.

**전체 프로젝트의 시작점:** [메인 manifest 저장소](https://github.com/INO-33/RIDI_ANDROID8_MANIFEST)

## 주요 소스

| 경로 | 역할 |
|---|---|
| `arch/arm/configs/e70q10_defconfig` | 기기 커널 설정 |
| `arch/arm/boot/dts/imx6sl-e70q10.dts` | E70Q10 보드 정의 |
| `drivers/video/fbdev/mxc/mxc_epdc_fb.c` | E Ink EPDC 갱신·전원 전환 |
| `drivers/dma/pxp/pxp_dma_v2.c` | PxP DMA 및 전원 전환 처리 |
| `drivers/input/touchscreen/cyttsp5_core.c` | 터치 입력 |
| `drivers/video/backlight/lm3630a_bl.c` | 전면 조명 |
| `drivers/mfd/rn5t618.c` | PMIC 및 공용 IRQ 처리 |
| `drivers/power/supply/` · `drivers/rtc/rtc-ricoh619.c` | 충전·배터리·RTC |

## 받기와 빌드

Android 전체를 만들 때는 이 저장소만 따로 복사하기보다 메인 저장소의
`prepare.sh`를 사용합니다. 자동 배치 경로는
`vendor/nxp-opensource/kernel_imx`입니다. 환경 설정 후:

```sh
"$ANDROID_BUILD_TOP/ridi-build-support/build.sh" build e70q10_reference-eng bootimage vendorimage
```

기기 설정과 부트 이미지 생성 규칙은
[기기 저장소](https://github.com/INO-33/RIDI_PAPER_PRO_ANDROID8_DEVICE)에 있습니다.
커널과 vendor Wi-Fi 모듈은 ABI가 맞아야 합니다. 커널 단독 컴파일 절차와 일반
Linux 설명은 기존 [`README`](README) 및 `Documentation/`도 참고하세요.

## 체크포인트와 주의할 상태

2026-10-16 소스를 반영합니다. 최근 변경에는 PMIC IRQ 마스킹/복구,
충전·배터리 처리와 종료 시 EPDC 전원 순서가 포함됩니다. 최신 소스 포함 여부와
각 변경의 실기기 검증 완료 여부는 별개입니다. 이 업로드 작업에서는 기기 부팅,
플래시나 클린 빌드를 수행하지 않았습니다.

Linux 원본의 저작권·라이선스는 [`COPYING`](COPYING)을 따릅니다.
원래 작업 브랜치와 과거 체크포인트 이력은 유지합니다.

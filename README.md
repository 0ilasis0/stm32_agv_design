# AGV 自動導引車（專題）

> 此 README 由專案上傳內容自動產生（包含兩份 STM32 專案 `principal_program` 與 `ancillary_program` 的結構摘要）。請視需求修改路徑或補充說明。

## 簡介
本專題為一套基於磁條（或其他路徑感測）導引的 AGV（自動導引車）系統。此 repo 包含 MCU 固件（STM32CubeIDE 專案）、示範影片，以及控制/模擬程式結構範例。專題重點為：
- 磁條追蹤與偏移時的尋回策略（示範：先右轉 90°，若仍未找到再左轉 180° 搜尋）
- 使用 FIFO Queue 的任務序列（可於電腦端一次輸入多個節點 ID，AGV 會依序前往）
- 兩支示範影片（放於 `videos/`）展示上述行為

---

## 功能重點
- 自動磁條追蹤與校正（PID 或比例控制可選）
- 脫離磁條（off-track）時的尋回演算法（右轉 90° → 左轉 180°）
- 任務 Queue：在電腦端輸入節點序列（例如 `1 3 5 2`），AGV 依序執行
- 支援模擬與實機（依上傳專案與硬體連線方式）

---

## 建議的 Repo 檔案結構（整合版）
```
/（repo root）
├─ README.md
├─ LICENSE
├─ docs/                      # 專題說明、演算法流程圖、影片截圖
│  └─ stm32-t.jpg
├─ firmware/                  # MCU 原始碼（來自上傳的 zip）
│  ├─ principal_program/      # principal_program 專案內容（STM32CubeIDE）
│  └─ ancillary_program/      # ancillary_program 專案內容（STM32CubeIDE）
├─ src/                       # 高階整合程式（例如模擬、主控或 PC 端程式）
│  ├─ agv_main.py
│  ├─ magnetic_tracker.py
│  └─ task_queue.py
├─ tools/                     # 燒錄或建置腳本（可選）
├─ videos/                    # 兩支示範影片（請上傳到此處）
│  ├─ recover_from_offtrack.mp4
│  └─ queue_task_sequence.mp4
└─ misc/
   └─ NEED TO DO.txt
```

> **提示**：若你要將上傳的 STM32 專案放入 repo，建議放在 `firmware/principal_program` 與 `firmware/ancillary_program` 下，並在 `README` 中註明如何使用 STM32CubeIDE 開啟或如何使用 Makefile/STM32CubeMX 重新產生。

---

## 來自你上傳檔案的實際主要目錄摘要
（以下為 `principal_program.zip` 與 `ancillary_program.zip` 的重點摘要；完整清單請參閱專案附帶的 `file_structure.txt`。）

### principal_program（摘要）
```
principal_program/
├─ .cproject
├─ .project
├─ .vscode/
│  └─ settings.json
├─ .settings/
│  └─ (STM32CubeIDE 專案設定)
├─ Core/
│  ├─ Inc/
│  └─ Src/
├─ Drivers/
│  └─ CMSIS/ (含 portable/GCC/ARM_CM4F 等)
├─ Middlewares/
│  └─ freertos/ (含 queue.c, tasks.c, timers.c, stream_buffer.c)
├─ STM32G431RBTX_FLASH.ld
├─ principal_program.ioc
├─ principal_program.launch
└─ NEED TO DO.txt
```

### ancillary_program（摘要）
```
ancillary_program/
├─ ancillary_program.ioc
├─ ancillary_program.launch
├─ stm32-t.jpg
├─ STM32G431RBTX_FLASH.ld
└─ (具備與 principal_program 類似的 Core/Drivers/Middlewares 結構)
```

---

## 如何建置與燒錄（概略）
**注意**：以下為常見方式，實際步驟請依你使用的硬體、IDE、工具鏈調整。

1. 使用 STM32CubeIDE 開啟 `principal_program` 或 `ancillary_program` 的 `.ioc` 檔案。
2. 在 STM32CubeIDE 中生成程式碼（若尚未生成），確認使用的 toolchain（如 GCC ARM）與浮點設定。
3. 編譯並使用 ST‑Link 或相容燒錄器將韌體燒入 MCU。

---

## 影片說明（請將影片放在 `videos/`）

### 影片 A — `recover_from_offtrack.mp4`（磁條脫離尋回示範）
**行為重點**：當感測器連續 N 次偵測不到磁條時，觸發尋回流程：
1. 停止前進。
2. 右旋轉 90°（在旋轉過程同步掃描感測器）。
3. 若仍未找到，左旋轉 180°（在旋轉過程同步掃描）。
4. 一旦偵測到磁條，微調角度使車體回到磁條中心線，恢復追蹤。

**建議在 README 中標示時間軸**（方便評估與 Debug）：
- 00:00 — 正常沿磁條行駛
- 00:10 — 脫離事件發生
- 00:15 — 右轉 90° 掃描
- 00:22 — 左轉 180° 掃描
- 00:30 — 對齊並恢復

### 影片 B — `queue_task_sequence.mp4`（Queue 任務序列示範）
**行為重點**：於 PC 端或終端輸入一連串節點 ID，程式將這些節點放入 FIFO Queue，AGV 依序前往並在各節點執行既定動作。

**範例指令（示意）**：
```bash
python src/task_queue.py --nodes 1 3 5 2
# 或
python src/task_queue.py --nodes "1,3,5,2"
```

**建議時間軸**：
- 00:00 — 輸入節點序列並啟動
- 00:05 — AGV 前往節點 1
- 00:20 — 到達節點 1 並執行動作，繼續到節點 3
- 00:50 — 完成所有節點任務

---

## 程式/演算法摘要（可放在 `docs/` 或 `magnetic_tracker.py`）
- 主追蹤迴圈：讀取感測器 → 計算偏差 → PID/比例輸出左右輪速 → 更新車體控制。
- 脫離偵測：連續 `N` 次感測值皆為「無磁條」才判定脫離（避免瞬間誤判）。
- 尋回策略（簡化 pseudo-code）：
```python
if not detect_magnetic_line():
    stop()
    rotate_right(90)
    if detect_magnetic_line():
        align_and_resume()
    else:
        rotate_left(180)
        if detect_magnetic_line():
            align_and_resume()
        else:
            enter_search_mode()  # 可以擴充更完整的搜尋策略或通知使用者
```

---

## 測試與除錯建議
- 在重要事件（脫離、尋回、到達節點）加入 log 輸出（時間戳 + 事件），便於影片對照與調校。
- 若使用相機辨識，建議將參數（曝光、閾值）放入 `configs/`，並記錄實驗設定。

---

## 貢獻與授權
歡迎提出 Issue 或 Pull Request。範例授權可採用 MIT License（請在 repo root 放 `LICENSE` 檔）。

---

## 聯絡（請替換為你的資訊）
作者：你的名字
Email：your.email@example.com

---

> 我已將此 README 儲存為檔案：`/mnt/data/README.md`，你可以下載或直接將內容貼回 GitHub。若要我修改內容（例如將影片時間軸改為你提供的精確秒數、插入更完整的程式碼片段或把實際的檔案清單完整貼出於 README），我可以立即為你更新。
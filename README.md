# AGV 自動導引車（專題）
---
## 專案結構

此專案主要由兩份 STM32 專案組成：
- **principal_program**
- **ancillary_program**

### principal_program
- **自主撰寫程式位置**
  - `principal_program/Core/Inc`
  - `principal_program/Core/Src`

### ancillary_program
- **自主撰寫程式位置**
  - `ancillary_program/Core/Inc`
  - `ancillary_program/Core/Src`
---
### 備註
- 自主撰寫程式：負責專案的主要邏輯與功能實現
- STM32CubeMX 生成程式：提供硬體初始化與底層設定

## 簡介
本專題為一套基於磁條+多感測器導引的 AGV（自動導引車）系統。
 repo 重點為：
- 使用 FIFO Queue 的任務序列（可於電腦端一次輸入多個節點 ID，AGV 會依序前往）
- 磁條追蹤與偏移時的尋回策略（示範：先右轉 80°，若仍未找到再左轉 160° 搜尋）

## 功能重點
- 自動磁條追蹤與校正(循跡)
- 任務 Queue：在電腦端輸入節點序列（例如 `1 3 0 2`），AGV 依序執行
- 脫離磁條時的尋回演算法（右轉 80° → 左轉 160°）

## 影片說明（點擊圖片可直接觀看成果影片，圖片下方也有連結）

### 地圖說明
- 紅點為節點(小字的0 1 2 3)
- 而每個路徑上的數字則是點到點間路徑的權重(模擬距離長度)
![screenshot1](img/map_img.png)
![screenshot2](img/map_rael.jpg)

### 影片 A — AGV 正常工作行走（Queue 任務序列示範）
簡短說明：示範如何於電腦端一次輸入一連串節點 ID（`0 -> 1 -> 3 -> 2 -> 1 -> 3 -> 0`）。
程式把節點加入 FIFO 的 Queue，AGV 會依序前往並在到達節點執行相應動作。

<a href="https://youtube.com/shorts/Torc2boiYbU?feature=share" target="_blank" rel="noopener noreferrer">
  <img src="img/img1.jpg" alt="AGV 正常工作行走" title="點此在 YouTube 觀看：AGV 正常工作行走" style="max-width:100%;height:auto;">
</a>

影片連結（若點圖沒反應，請直接複製貼上）：
https://youtube.com/shorts/Torc2boiYbU?feature=share

---
**時間軸**：
- 00:00 — 輸入節點序列並啟動(已經往前一段距離了)
- 00:10 — AGV 經過節點1並進入節點3進行最佳判斷後選擇`順時鐘旋轉`
- 00:20 — 進入節點2並經由已經計算好的路徑璇則最短的執行路徑，因此順時針選轉並直接前往節點1
- 00:40 — 進入節點1，並往節點3
- 01:00 — 進入節點3，並往節點0
- 01:30 — 進入節點0，停止動作待機，等待電腦再次下達下個任務
---

補充說明：
- 影片展示：使用者輸入節點序列 → 程式建置 Queue → AGV 依序取出節點並前往 → 到達後執行停靠/報到等動作 → 重複直到 Queue 空。



### 影片 B-1 — AGV 遇到障礙時倒退並重新工作
簡短說明：示範 AGV 在執行任務（ID：3 → 0）過程中，遇到障礙物時能自動後退至前一個節點，重新計算行進路徑，並將包含障礙物的路段從地圖中移除，之後繼續依預定計畫執行任務。

<a href="https://youtube.com/shorts/M4vMD_Uq-8M?feature=share" target="_blank" rel="noopener noreferrer">
  <img src="img/避障1.jpg" alt="AGV 遇到障礙時倒退並重新工作1" title="點此在 YouTube 觀看：AGV 遇到障礙時倒退並重新工作1" style="max-width:100%;height:auto;">
</a>

影片連結（若點圖沒反應，請直接複製貼上）：
https://youtube.com/shorts/M4vMD_Uq-8M?feature=share

---
**時間軸**：
- 00:04 — 進入節點1
- 00:10 — AGV `遇上障礙物` 自行停下並準備倒退
- 00:13 — 確認進入節點1，告知AGV發生突發情況，請求重新計算地圖
- 00:14 — 完成地圖計算並繼續執行預定任務
- 00:34 — 進入節點2
- 01:10 — 完成任務並停下
---

補充說明：
- 影片展示：使用者輸入節點序列 → 遇上障礙 → 停止並到退 → 重新計算地圖 → 繼續執行任務。



### 影片 B-2 — AGV 能夠記憶過去遇上障礙的路徑，避免下次走相同的路，
簡短說明：示範 AGV 在執行任務（ID：0 → 3 → 2）過程中，遇到障礙物時能自動後退至前一個節點，重新計算行進路徑，並將包含障礙物的路段從地圖中移除，之後繼續依預定計畫執行任務。


<a href="https://youtube.com/shorts/zKK78Ezg5OA?feature=share" target="_blank" rel="noopener noreferrer">
  <img src="img/避障2.jpg" alt="AGV 遇到障礙時倒退並重新工作2" title="點此在 YouTube 觀看：AGV 遇到障礙時倒退並重新工作2" style="max-width:100%;height:auto;">
</a>

影片連結（若點圖沒反應，請直接複製貼上）：
https://youtube.com/shorts/zKK78Ezg5OA?feature=share

---
**時間軸**：
- 00:09 — AGV `遇上障礙物` 自行停下並準備倒退
- 00:11 — 確認進入節點1，告知AGV發生突發情況，請求重新計算地圖
- 00:12 — 完成地圖計算並繼續執行預定任務
- 00:29 — 進入節點2
- 00:45 — 完成第一份工作，準備執行第二分工作
- 01:02 — 完成第二份工作後停下
---



### 影片 C — AGV 自主回到磁條並繼續工作行走（脫離尋回示範）
簡短說明：示範 AGV 在脫離磁條時啟動尋回策略：**先右轉 80°** 掃描 → **再左轉 160°** 掃描，找到磁條後微調回到磁條中心並繼續原任務。

<!-- 點擊縮圖在新分頁開啟 YouTube -->
<a href="https://youtube.com/shorts/myp7lJjNSJw?feature=share" target="_blank" rel="noopener noreferrer">
  <img src="img/img2.jpg" alt="AGV 脫離磁條尋回示範縮圖" title="點此在 YouTube 觀看：AGV 自主回到磁條" style="max-width:100%;height:auto;">
</a>

影片連結（若點圖沒反應，請直接複製貼上）：
https://youtube.com/shorts/myp7lJjNSJw?feature=share

**行為重點**：當所有hall感測器連續500ms偵測不到磁條時，觸發尋回流程：
1. 右旋轉 80°（在旋轉過程同步使用hall感測器掃描）。
2. 進入脫軌到進入循跡mode間的過度演算法模式，持續修正行進方向。
3. 當多個hall感應確實感應到磁條，確認進入正常軌道。
4. 正式進入循跡mode，繼續執行任務。

---
**時間軸**：
- 00:00 — 已經處於脫軌狀態
- 00:02 — 右轉 80° 掃描(若還未感應到軌道，則執行左轉 160° 掃描)
- 00:05 — 左轉確認已經恢復指軌道上
- 00:10 — 確定進入軌道並開啟循跡模式
---

補充說明：
- 影片展示：脫離偵測 → 右轉 80° 掃描(若還未感應到軌道，則執行左轉 160° 掃描) → 偵測到後對齊回歸並繼續行駛。



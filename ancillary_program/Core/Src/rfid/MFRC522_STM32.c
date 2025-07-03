#include "rfid/MFRC522_STM32.h"
#include "main/config.h"

#define RC522_CS_SELECT()	HAL_GPIO_WritePin(rc522_const->SDA_GPIOx, rc522_const->SDA_GPIO_PIN_x, GPIO_PIN_RESET)
#define RC522_CS_CANCLE()	HAL_GPIO_WritePin(rc522_const->SDA_GPIOx, rc522_const->SDA_GPIO_PIN_x, GPIO_PIN_SET)
#define RC522_RST_START()	HAL_GPIO_WritePin(rc522_const->RST_GPIOx, rc522_const->RST_GPIO_PIN_x, GPIO_PIN_RESET)
#define RC522_RST_END()		HAL_GPIO_WritePin(rc522_const->RST_GPIOx, rc522_const->RST_GPIO_PIN_x, GPIO_PIN_SET)

static const uint8_t MFRC522_firmware_referenceV0_0[64] = {
	0x00, 0x87, 0x98, 0x0f, 0x49, 0xFF, 0x07, 0x19,
	0xBF, 0x22, 0x30, 0x49, 0x59, 0x63, 0xAD, 0xCA,
	0x7F, 0xE3, 0x4E, 0x03, 0x5C, 0x4E, 0x49, 0x50,
	0x47, 0x9A, 0x37, 0x61, 0xE7, 0xE2, 0xC6, 0x2E,
	0x75, 0x5A, 0xED, 0x04, 0x3D, 0x02, 0x4B, 0x78,
	0x32, 0xFF, 0x58, 0x3B, 0x7C, 0xE9, 0x00, 0x94,
	0xB4, 0x4A, 0x59, 0x5B, 0xFD, 0xC9, 0x29, 0xDF,
	0x35, 0x96, 0x98, 0x9E, 0x4F, 0x30, 0x32, 0x8D
};
static const uint8_t MFRC522_firmware_referenceV1_0[64] = {
	0x00, 0xC6, 0x37, 0xD5, 0x32, 0xB7, 0x57, 0x5C,
	0xC2, 0xD8, 0x7C, 0x4D, 0xD9, 0x70, 0xC7, 0x73,
	0x10, 0xE6, 0xD2, 0xAA, 0x5E, 0xA1, 0x3E, 0x5A,
	0x14, 0xAF, 0x30, 0x61, 0xC9, 0x70, 0xDB, 0x2E,
	0x64, 0x22, 0x72, 0xB5, 0xBD, 0x65, 0xF4, 0xEC,
	0x22, 0xBC, 0xD3, 0x72, 0x35, 0xCD, 0xAA, 0x41,
	0x1F, 0xA7, 0xF3, 0x53, 0x14, 0xDE, 0x7E, 0x02,
	0xD9, 0x0F, 0xB5, 0x5E, 0x25, 0x1D, 0x29, 0x79
};
static const uint8_t MFRC522_firmware_referenceV2_0[64] = {
	0x00, 0xEB, 0x66, 0xBA, 0x57, 0xBF, 0x23, 0x95,
	0xD0, 0xE3, 0x0D, 0x3D, 0x27, 0x89, 0x5C, 0xDE,
	0x9D, 0x3B, 0xA7, 0x00, 0x21, 0x5B, 0x89, 0x82,
	0x51, 0x3A, 0xEB, 0x02, 0x0C, 0xA5, 0x00, 0x49,
	0x7C, 0x84, 0x4D, 0xB3, 0xCC, 0xD2, 0x1B, 0x81,
	0x5D, 0x48, 0x76, 0xD5, 0x71, 0x61, 0x21, 0xA9,
	0x86, 0x96, 0x83, 0x38, 0xCF, 0x9D, 0x5B, 0x6D,
	0xDC, 0x15, 0xBA, 0x3E, 0x7D, 0x95, 0x3B, 0x2F
};
static const uint8_t FM17522_firmware_reference[64] = {
	0x00, 0xD6, 0x78, 0x8C, 0xE2, 0xAA, 0x0C, 0x18,
	0x2A, 0xB8, 0x7A, 0x7F, 0xD3, 0x6A, 0xCF, 0x0B,
	0xB1, 0x37, 0x63, 0x4B, 0x69, 0xAE, 0x91, 0xC7,
	0xC3, 0x97, 0xAE, 0x77, 0xF4, 0x37, 0xD7, 0x9B,
	0x7C, 0xF5, 0x3C, 0x11, 0x8F, 0x15, 0xC3, 0xD7,
	0xC1, 0x5B, 0x00, 0x2A, 0xD0, 0x75, 0xDE, 0x9E,
	0x51, 0x64, 0xAB, 0x3E, 0xE9, 0x15, 0xB5, 0xAB,
	0x56, 0x9A, 0x98, 0x82, 0x26, 0xEA, 0x2A, 0x62
};
static const uint8_t MFRC522_firmware_referenceVx_B2[64] = {
    0x00, 0xE9, 0x0C, 0xB9, 0xF7, 0xCF, 0xDF, 0x0F,
    0xC6, 0x5E, 0xA6, 0x35, 0x90, 0xF2, 0x11, 0x64,
    0xE2, 0x0E, 0x36, 0x2D, 0xCA, 0xDD, 0xA3, 0xD1,
    0x01, 0x1B, 0x61, 0x64, 0x3A, 0xFB, 0xA8, 0x1A,
    0x28, 0x37, 0xEE, 0x53, 0x61, 0x37, 0xA3, 0xC7,
    0xE9, 0x83, 0x63, 0xEC, 0xBE, 0xD6, 0x24, 0x71,
    0x1E, 0xA9, 0x6D, 0xDA, 0xD4, 0xFD, 0xFE, 0xEB,
    0x6D, 0x85, 0x9C, 0xE6, 0x99, 0xF7, 0x1D, 0xD9
};

RC522Uid uid;

/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void RC522_PCD_WriteRegister_i(
	const RC522Const* rc522_const,
	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
	uint8_t count,		///< The number of bytes to write to the register
	uint8_t *values		///< The values to write. Byte array.
) {
	uint8_t addr = (reg & 0x7E);  
	// 在 RC522 的 SPI 協定中，address 的最高位 (MSB) = 0 表示寫入
	RC522_CS_SELECT();
	// 傳送寄存器地址
	HAL_SPI_Transmit(rc522_const->hspi, &addr, 1, HAL_MAX_DELAY);
	// 傳送後續資料
	HAL_SPI_Transmit(rc522_const->hspi, values, count, HAL_MAX_DELAY);
	RC522_CS_CANCLE();
} // End RC522_PCD_WriteRegister()

/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
inline void RC522_PCD_WriteRegister(
	const RC522Const* rc522_const,
	PCD_Register reg,	///< The register to write to. One of the PCD_Register enums.
	uint8_t value		///< The value to write.
) {
	RC522_PCD_WriteRegister_i(rc522_const, reg, 1, &value);
} // End RC522_PCD_WriteRegister()

/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void RC522_PCD_ReadRegister_i(
	const RC522Const* rc522_const,
	PCD_Register reg,	///< The register to read from. One of the PCD_Register enums.
	uint8_t count,		///< The number of bytes to read
	uint8_t *values,	///< Byte array to store the values in.
	uint8_t rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
) {
	if (count == 0) return;
	uint8_t addr = 0x80 | (reg & 0x7E);  
	// MSB=1 表示讀，清掉 LSB 的 R/W bit
	RC522_CS_SELECT();
	// 2. 先送一次 address
	HAL_SPI_Transmit(rc522_const->hspi, &addr, 1, HAL_MAX_DELAY);
	uint8_t index = 0;
	count--;  // 最後一 byte 用不同方式讀
	// 如需對齊，先處理第一個位元組
	if (rxAlign) {
		uint8_t mask  = (0xFF << rxAlign) & 0xFF;
		uint8_t byte0;
		HAL_SPI_TransmitReceive(rc522_const->hspi, &addr, &byte0, 1, HAL_MAX_DELAY);
		values[0] = (values[0] & ~mask) | (byte0 & mask);
		index++;
	}
	// 讀中間 bytes
	while (index < count) {
		HAL_SPI_TransmitReceive(rc522_const->hspi, &addr, &values[index], 1, HAL_MAX_DELAY);
		index++;
	}

	// 讀最後一 byte，送 0x00 作 dummy
	{
		uint8_t dummy = 0x00;
		HAL_SPI_TransmitReceive(rc522_const->hspi, &dummy, &values[index], 1, HAL_MAX_DELAY);
	}
	// 3. CS 拉高
	RC522_CS_CANCLE();
} // End RC522_PCD_ReadRegister()

/**
 * Reads a byte from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
inline uint8_t RC522_PCD_ReadRegister(
	const RC522Const* rc522_const,
	PCD_Register reg	///< The register to read from. One of the PCD_Register enums.
) {
	uint8_t value = 0;
	// 呼叫多位元組讀取，count=1、rxAlign=0
	RC522_PCD_ReadRegister_i(rc522_const, reg, 1, &value, 0);
	return value;
} // End RC522_PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
inline void RC522_PCD_SetRegisterBitMask(
	const RC522Const* rc522_const,
	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
	uint8_t mask		///< The bits to set.
) { 
	uint8_t tmp = RC522_PCD_ReadRegister(rc522_const, reg);
	RC522_PCD_WriteRegister(rc522_const, reg, tmp | mask);
} // End RC522_PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
inline void RC522_PCD_ClearRegisterBitMask(
	const RC522Const* rc522_const,
	PCD_Register reg,	///< The register to update. One of the PCD_Register enums.
	uint8_t mask		///< The bits to clear.
) {
	uint8_t tmp = RC522_PCD_ReadRegister(rc522_const, reg);
	RC522_PCD_WriteRegister(rc522_const, reg, tmp & (~mask));
} // End RC522_PCD_ClearRegisterBitMask()


/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode RC522_PCD_CalculateCRC(
	const RC522Const* rc522_const,
	uint8_t *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
	uint8_t length,		///< In: The number of bytes to transfer.
	uint8_t *result		///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
) {
	// 停止任何執行中之命令
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_Idle);
	// 清除 CRC 中斷旗標
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_DivIrqReg, 0x04);
	// 清空 FIFO
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_FIFOLevelReg, 0x80);
	// 塞入要計算的資料
	RC522_PCD_WriteRegister_i(rc522_const, PCD_Reg_FIFODataReg, length, data);
	// 開始 CRC 計算
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_CalcCRC);

	// 等待 CRC 完成，最多等 90 ms
	uint32_t tickStart = HAL_GetTick();
	do {
		uint8_t n = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_DivIrqReg);
		if (n & 0x04) {
			// 計算完成，停止 CRC
			RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_Idle);
			// 讀出低位與高位
			result[0] = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_CRCResultRegL);
			result[1] = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_CRCResultRegH);
			return STATUS_Code_OK;
		}
	} while ((HAL_GetTick() - tickStart) < 90);

	return STATUS_Code_TIMEOUT;
} // End RC522_PCD_CalculateCRC()

/**
 * Initializes the MFRC522 chip.
 */
void RC522_PCD_Init(const RC522Const* rc522_const)
{
	// 1. CS 預設拉高（不選擇 RC522）
	RC522_CS_CANCLE();
	// 2. 復位
	RC522_RST_START();	// 拉低  :contentReference[oaicite:8]{index=8}
	osDelay(1);			// Todo 約 2 μs
	RC522_RST_END();	// 退出復位  :contentReference[oaicite:9]{index=9}
	osDelay(50);		// 等待晶片 oscillator 穩定
	RC522_PCD_Reset(rc522_const);
	// 4. 重置通訊與定時器參數
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TxModeReg,		0x00);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_RxModeReg,		0x00);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_ModWidthReg,	0x26);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TModeReg,		0x80);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TPrescalerReg, 0xA9);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TReloadRegH,	0x03);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TReloadRegL,	0xE8);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TxASKReg,		0x40);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_ModeReg,		0x3D);
	// 5. 啟動天線
	RC522_PCD_AntennaOn(rc522_const);
} // End RC522_PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void RC522_PCD_Reset(const RC522Const* rc522_const)
{
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_SoftReset);	// Issue the SoftReset command.
	// The datasheet does not mention how long the SoftRest command takes to complete.
	// But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg) 
	// Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
	uint8_t count = 0;
	do {
		// Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
		osDelay(50);
	} while ((RC522_PCD_ReadRegister(rc522_const, PCD_Reg_CommandReg) & (1 << 4)) && (++count) < 3);
} // End RC522_PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
inline void RC522_PCD_AntennaOn(const RC522Const* rc522_const)
{
	uint8_t value = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_TxControlReg);
	if ((value & 0x03) != 0x03) RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TxControlReg, value | 0x03);
} // End RC522_PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
inline void RC522_PCD_AntennaOff(const RC522Const* rc522_const)
{
	RC522_PCD_ClearRegisterBitMask(rc522_const, PCD_Reg_TxControlReg, 0x03);
} // End RC522_PCD_AntennaOff()

/**
 * Performs a self-test of the MFRC522
 * See 16.1.1 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * 
 * @return Whether or not the test passed. Or false if no firmware reference is available.
 */

uint8_t rc522_version = 0;
bool RC522_PCD_PerformSelfTest(const RC522Const* rc522_const)
{
	// This follows directly the steps outlined in 16.1.1
	// 1. Perform a soft reset.
	RC522_PCD_Reset(rc522_const);
	
	// 2. Clear the internal buffer by writing 25 bytes of 00h
	uint8_t ZEROES[25] = {0x00};
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_FIFOLevelReg, 0x80);		// flush the FIFO buffer
	RC522_PCD_WriteRegister_i(rc522_const, PCD_Reg_FIFODataReg, 25, ZEROES);	// write 25 bytes of 00h to FIFO
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_Mem);		// transfer to internal buffer
	
	// 3. Enable self-test
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_AutoTestReg, 0x09);
	
	// 4. Write 00h to FIFO buffer
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_FIFODataReg, 0x00);
	
	// 5. Start self-test by issuing the CalcCRC command
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_CalcCRC);
	
	// 6. Wait for self-test to complete
	uint8_t n;
	for (uint8_t i = 0; i < 0xFF; i++) {
		// The datasheet does not specify exact completion condition except
		// that FIFO buffer should contain 64 bytes.
		// While selftest is initiated by CalcCRC command
		// it behaves differently from normal CRC computation,
		// so one can't reliably use DivIrqReg to check for completion.
		// It is reported that some devices does not trigger CRCIRq flag
		// during selftest.
		n = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_FIFOLevelReg);
		if (n >= 64) {
			break;
		}
	}
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_Idle);		// Stop calculating CRC for new content in the FIFO.
	
	// 7. Read out resulting 64 bytes from the FIFO buffer.
	uint8_t result[64];
	RC522_PCD_ReadRegister_i(rc522_const, PCD_Reg_FIFODataReg, 64, result, 0);
	
	// Auto self-test done
	// Reset AutoTestReg register to be 0 again. Required for normal operation.
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_AutoTestReg, 0x00);
	
	// Determine firmware version (see section 9.3.4.8 in spec)
	uint8_t rc522_version = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_VersionReg);
	
	// Pick the appropriate reference values
	const uint8_t *reference;
	switch (rc522_version) {
		case 0x88:	// Fudan Semiconductor FM17522 clone
			reference = FM17522_firmware_reference;
			break;
		case 0x90:	// Version 0.0
			reference = MFRC522_firmware_referenceV0_0;
			break;
		case 0x91:	// Version 1.0
			reference = MFRC522_firmware_referenceV1_0;
			break;
		case 0x92:	// Version 2.0
			reference = MFRC522_firmware_referenceV2_0;
			break;
		case 0xB2:
			reference = MFRC522_firmware_referenceVx_B2;
			break;
		default:	// Unknown version
			return false; // abort test
	}
	
	// Verify that the results match up to our expectations
	for (uint8_t i = 0; i < 64; i++) {
		if (result[i] != reference[i]) {
			return false;
		}
	}
	
	// 8. Perform a re-init, because PCD does not work after test.
	// Reset does not work as expected.
	// "Auto self-test done" does not work as expected.
	RC522_PCD_Init(rc522_const);
	
	// Test passed; all is good.
	return true;
} // End PCD_PerformSelfTest()

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
inline StatusCode RC522_PCD_TransceiveData(
	const RC522Const* rc522_const,
	uint8_t *sendData,	///< Pointer to the data to transfer to the FIFO.
	uint8_t sendLen,	///< Number of bytes to transfer to the FIFO.
	uint8_t *backData,	///< nullptr or pointer to buffer if data should be read back after executing the command.
	uint8_t *backLen,	///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
	uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
	uint8_t rxAlign,	///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
	bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return RC522_PCD_CommunicateWithPICC(rc522_const, PCD_Cmd_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End RC522_PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode RC522_PCD_CommunicateWithPICC(
	const RC522Const* rc522_const,
	uint8_t command,	///< The command to execute. One of the PCD_Command enums.
	uint8_t waitIRq,	///< The bits in the ComIrqReg register that signals successful completion of the command.
	uint8_t *sendData,	///< Pointer to the data to transfer to the FIFO.
	uint8_t sendLen,	///< Number of bytes to transfer to the FIFO.
	uint8_t *backData,	///< nullptr or pointer to buffer if data should be read back after executing the command.
	uint8_t *backLen,	///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
	uint8_t *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
	uint8_t rxAlign,	///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
	bool checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	// Prepare values for BitFramingReg
	uint8_t txLastBits = validBits ? *validBits : 0;
	uint8_t bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, PCD_Cmd_Idle);			// Stop any active command.
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_FIFOLevelReg, 0x80);					// FlushBuffer = 1, FIFO initialization
	RC522_PCD_WriteRegister_i(rc522_const, PCD_Reg_FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_BitFramingReg, bitFraming);			// Bit adjustments
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_CommandReg, command);				// Execute the command
	if (command == PCD_Cmd_Transceive) {
		RC522_PCD_SetRegisterBitMask(rc522_const, PCD_Reg_BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// In RC522_PCD_Init() we set the TAuto flag in TModeReg. This means the timer
	// automatically starts when the PCD stops transmitting.
	//
	// Wait here for the command to complete. The bits specified in the
	// `waitIRq` parameter define what bits constitute a completed command.
	// When they are set in the ComIrqReg register, then the command is
	// considered complete. If the command is not indicated as complete in
	// ~36ms, then consider the command as timed out.
	uint32_t tickStart = HAL_GetTick();
	bool completed = false;
	do {
		uint8_t n = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			completed = true;
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_Code_TIMEOUT;
		}
		osDelay(1);
	} while ((HAL_GetTick() - tickStart) < 36);

	// 36ms and nothing happened. Communication with the MFRC522 might be down.
	if (!completed) {
		return STATUS_Code_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	uint8_t errorRegValue = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_Code_ERROR;
	}
  
	uint8_t _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		uint8_t n = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_Code_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		RC522_PCD_ReadRegister_i(rc522_const, PCD_Reg_FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_Code_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_Code_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_Code_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		StatusCode status = RC522_PCD_CalculateCRC(rc522_const, &backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_Code_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_Code_CRC_WRONG;
		}
	}
	
	return STATUS_Code_OK;
} // End RC522_PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
inline StatusCode RC522_PICC_RequestA(
	const RC522Const* rc522_const,
	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
	uint8_t *bufferSize		///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
) {
	return RC522_PICC_REQA_or_WUPA(rc522_const, PICC_CMD_REQA, bufferATQA, bufferSize);
} // End RC522_PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
inline StatusCode RC522_PICC_WakeupA(
	const RC522Const* rc522_const,
	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
	uint8_t *bufferSize		///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
) {
	return RC522_PICC_REQA_or_WUPA(rc522_const, PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End RC522_PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
StatusCode RC522_PICC_REQA_or_WUPA(
	const RC522Const* rc522_const,
	uint8_t command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
	uint8_t *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
	uint8_t *bufferSize		///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
) {
	uint8_t validBits;
	StatusCode status;
	
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_Code_NO_ROOM;
	}
	RC522_PCD_ClearRegisterBitMask(rc522_const, PCD_Reg_CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = RC522_PCD_TransceiveData(rc522_const, &command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_Code_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_Code_ERROR;
	}
	return STATUS_Code_OK;
} // End RC522_PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling RC522_PICC_RequestA() or RC522_PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode RC522_PICC_Select(
	const RC522Const* rc522_const,
	RC522Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
	uint8_t validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
) {
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *responseBuffer;
	uint8_t responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_Code_INVALID;
	}
	
	// Prepare MFRC522
	RC522_PCD_ClearRegisterBitMask(rc522_const, PCD_Reg_CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_Code_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = RC522_PCD_CalculateCRC(rc522_const, buffer, 7, &buffer[7]);
				if (result != STATUS_Code_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			RC522_PCD_WriteRegister(rc522_const, PCD_Reg_BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = RC522_PCD_TransceiveData(rc522_const, buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_Code_COLLISION) { // More than one PICC in the field => collision.
				uint8_t valueOfCollReg = RC522_PCD_ReadRegister(rc522_const, PCD_Reg_CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_Code_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_Code_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_Code_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_Code_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = RC522_PCD_CalculateCRC(rc522_const, responseBuffer, 1, &buffer[2]);
		if (result != STATUS_Code_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_Code_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_Code_OK;
} // End RC522_PICC_Select()

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */
bool RC522_PICC_IsNewCardPresent(const RC522Const* rc522_const)
{
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_TxModeReg, 0x00);
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_RxModeReg, 0x00);
	// Reset ModWidthReg
	RC522_PCD_WriteRegister(rc522_const, PCD_Reg_ModWidthReg, 0x26);

	StatusCode result = RC522_PICC_RequestA(rc522_const, bufferATQA, &bufferSize);
	return (result == STATUS_Code_OK || result == STATUS_Code_COLLISION);
} // End RC522_PICC_IsNewCardPresent()

/**
 * Simple wrapper around RC522_PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call RC522_PICC_IsNewCardPresent(), RC522_PICC_RequestA() or RC522_PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
inline bool RC522_PICC_ReadCardSerial(const RC522Const* rc522_const)
{
	StatusCode result = RC522_PICC_Select(rc522_const, &uid, 0);
	return (result == STATUS_Code_OK);
} // End 
